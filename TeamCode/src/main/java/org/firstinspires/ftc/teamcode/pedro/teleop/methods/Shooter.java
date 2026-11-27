package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Bang-bang flywheel with hysteresis, ready check, feed burst offset, and shot tracking.
 *
 *   velocity &lt; target − HYSTERESIS → ON_POWER
 *   velocity &gt; target + HYSTERESIS → coast
 *   inside the band                 → hold last state
 */
public class Shooter {

    public static final int MAX_SHOTS = 8;
    private static final double FALLBACK_VOLTAGE = 12.5;
    private static final int WINDOW_SIZE = 64;

    private final DcMotorEx shooter;
    private final List<VoltageSensor> voltageSensors = new ArrayList<>();

    private boolean running = false;
    private boolean feeding = false;
    private boolean bangOn = false;

    private double velocity = 0.0;
    private double smoothVelocity = 0.0;
    private boolean smoothSeeded = false;
    private double batteryVoltage;
    private double lastPower = 0.0;
    private double sentPower = Double.NaN;
    private double readyTime = 0.0;
    private double loopSec = 0.0;
    private long lastNs = 0L;
    private long lastVoltageNs = 0L;
    private String mode = "OFF";

    private final long[] windowNs = new long[WINDOW_SIZE];
    private final double[] windowVel = new double[WINDOW_SIZE];
    private int windowHead = 0;
    private int windowCount = 0;
    private boolean inDip = false;
    private double dipMin = 0.0;
    private boolean hasFed = false;
    private long feedEndNs = 0L;
    private final double[] shotSpeed = new double[MAX_SHOTS];
    private final double[] shotRecoverySec = new double[MAX_SHOTS];
    private int shotCount = 0;
    private long lastShotNs = 0L;
    private boolean awaitingRecovery = false;

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltageSensors.add(sensor);
        }
        double volts = readBatteryVoltage();
        batteryVoltage = Double.isNaN(volts) ? FALLBACK_VOLTAGE : volts;
        sendPower(0.0);
    }

    public void setTargetVelocity(double velocity) {
        Constants.SHOOTER_TARGET_VELOCITY = Math.max(0, velocity);
    }

    public double getTargetVelocity() {
        return Constants.SHOOTER_TARGET_VELOCITY;
    }

    public void setRunning(boolean running) {
        this.running = running;
        if (running) {
            bangOn = true;
        } else {
            resetState();
            bangOn = false;
            lastPower = 0.0;
            mode = "OFF";
            sendPower(0.0);
        }
    }

    public void toggle() {
        setRunning(!running);
    }

    public boolean isRunning() {
        return running;
    }

    public void stop() {
        setRunning(false);
    }

    /**
     * Transfer is pushing balls in. Raises target by {@link Constants#SHOOTER_BURST_OFFSET}
     * and watches for each ball's speed drop.
     */
    public void setFeeding(boolean feeding) {
        if (feeding == this.feeding) {
            return;
        }
        long now = System.nanoTime();
        if (feeding) {
            if (!hasFed || (now - feedEndNs) / 1.0e9 > Constants.SHOOTER_BURST_GAP_SEC) {
                clearShots();
            }
            hasFed = true;
        } else {
            feedEndNs = now;
        }
        this.feeding = feeding;
    }

    public boolean isFeeding() {
        return feeding;
    }

    public void update() {
        long now = System.nanoTime();
        double dt = (lastNs == 0L) ? 0.02 : Range.clip((now - lastNs) / 1.0e9, 0.002, 0.1);
        lastNs = now;
        loopSec = dt;

        velocity = shooter.getVelocity();
        if (!smoothSeeded) {
            smoothVelocity = velocity;
            smoothSeeded = true;
        } else {
            double alpha = dt / (Math.max(Constants.SHOOTER_SPEED_SMOOTH_SEC, 0.0) + dt);
            smoothVelocity += alpha * (velocity - smoothVelocity);
        }
        updateVoltage(now);

        double targetVelocity = Constants.SHOOTER_TARGET_VELOCITY;
        if (!running || targetVelocity <= 0) {
            mode = "OFF";
            bangOn = false;
            lastPower = 0.0;
            readyTime = 0.0;
            sendPower(0.0);
            return;
        }

        double target = targetVelocity
                + (feeding ? Math.max(0.0, Constants.SHOOTER_BURST_OFFSET) : 0.0);
        double hyst = Math.max(0.0, Constants.SHOOTER_HYSTERESIS);

        if (velocity < target - hyst) {
            bangOn = true;
        } else if (velocity > target + hyst) {
            bangOn = false;
        }

        mode = bangOn ? "FULL" : "COAST";
        lastPower = bangOn ? Range.clip(Constants.SHOOTER_ON_POWER, 0.0, 1.0) : 0.0;
        sendPower(lastPower);

        boolean inTolerance =
                Math.abs(target - smoothVelocity) <= Constants.SHOOTER_READY_TOL * target;
        readyTime = inTolerance ? readyTime + dt : 0.0;

        trackShots(now);
    }

    /** Smoothed speed within READY_TOL of target for READY_SEC. */
    public boolean atSpeed() {
        return running && readyTime >= Constants.SHOOTER_READY_SEC;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getSmoothedVelocity() {
        return smoothVelocity;
    }

    public double getLastPower() {
        return lastPower;
    }

    public boolean isBangOn() {
        return bangOn;
    }

    public double getBatteryVoltage() {
        return batteryVoltage;
    }

    public double getLoopMs() {
        return loopSec * 1000.0;
    }

    /** OFF, FULL, or COAST. */
    public String getMode() {
        return mode;
    }

    public DcMotorEx getShooter() {
        return shooter;
    }

    public int getShotCount() {
        return shotCount;
    }

    public double getShotSpeed(int index) {
        return shotSpeed[index];
    }

    public double getShotRecoverySec(int index) {
        return shotRecoverySec[index];
    }

    public double getShotSpreadPercent() {
        double targetVelocity = Constants.SHOOTER_TARGET_VELOCITY;
        if (shotCount < 2 || targetVelocity <= 0) {
            return 0.0;
        }
        double min = shotSpeed[0];
        double max = shotSpeed[0];
        for (int i = 1; i < shotCount; i++) {
            min = Math.min(min, shotSpeed[i]);
            max = Math.max(max, shotSpeed[i]);
        }
        return (max - min) / targetVelocity * 100.0;
    }

    public double getShotDropPercent() {
        double targetVelocity = Constants.SHOOTER_TARGET_VELOCITY;
        if (shotCount < 2 || targetVelocity <= 0) {
            return 0.0;
        }
        return (shotSpeed[0] - shotSpeed[shotCount - 1]) / targetVelocity * 100.0;
    }

    public String shotSpeedsText() {
        if (shotCount == 0) {
            return "none yet";
        }
        StringBuilder text = new StringBuilder();
        for (int i = 0; i < shotCount; i++) {
            if (i > 0) {
                text.append(" / ");
            }
            text.append(String.format(Locale.US, "%.0f", shotSpeed[i]));
        }
        return text.toString();
    }

    public String shotRecoveryText() {
        if (shotCount == 0) {
            return "none yet";
        }
        StringBuilder text = new StringBuilder();
        for (int i = 0; i < shotCount; i++) {
            if (i > 0) {
                text.append(" / ");
            }
            text.append(Double.isNaN(shotRecoverySec[i])
                    ? "--"
                    : String.format(Locale.US, "%.2f", shotRecoverySec[i]));
        }
        return text.toString();
    }

    public void clearShots() {
        shotCount = 0;
        awaitingRecovery = false;
    }

    private void resetState() {
        readyTime = 0.0;
        inDip = false;
        windowCount = 0;
        windowHead = 0;
        awaitingRecovery = false;
    }

    private void sendPower(double power) {
        if (power != sentPower) {
            shooter.setPower(power);
            sentPower = power;
        }
    }

    private void updateVoltage(long now) {
        double since = (now - lastVoltageNs) / 1.0e9;
        if (lastVoltageNs != 0L && since < Constants.SHOOTER_VOLTAGE_READ_SEC) {
            return;
        }
        double raw = readBatteryVoltage();
        if (!Double.isNaN(raw)) {
            if (lastVoltageNs == 0L) {
                batteryVoltage = raw;
            } else {
                double alpha = since / (Math.max(Constants.SHOOTER_VOLTAGE_TAU_SEC, 0.0) + since);
                batteryVoltage += alpha * (raw - batteryVoltage);
            }
        }
        lastVoltageNs = now;
    }

    private double readBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : voltageSensors) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                min = Math.min(min, voltage);
            }
        }
        return Double.isInfinite(min) ? Double.NaN : min;
    }

    private void trackShots(long now) {
        if (awaitingRecovery && atSpeed()) {
            shotRecoverySec[shotCount - 1] = (now - lastShotNs) / 1.0e9;
            awaitingRecovery = false;
        }

        boolean watching = feeding
                || (hasFed && (now - feedEndNs) / 1.0e9 < Constants.SHOOTER_SHOT_WATCH_AFTER_FEED_SEC);
        if (!watching) {
            windowCount = 0;
            windowHead = 0;
            inDip = false;
            return;
        }

        double speed = smoothVelocity;
        double targetVelocity = Constants.SHOOTER_TARGET_VELOCITY;
        if (inDip) {
            dipMin = Math.min(dipMin, speed);
            if (speed >= dipMin + Constants.SHOOTER_SHOT_REARM * targetVelocity) {
                inDip = false;
                windowCount = 0;
                windowHead = 0;
                pushSample(now, speed);
            }
            return;
        }

        pushSample(now, speed);
        double before = windowMax(now);
        if (before - speed >= Constants.SHOOTER_SHOT_DROP * targetVelocity) {
            recordShot(before, now);
            inDip = true;
            dipMin = speed;
        }
    }

    private void recordShot(double speedBefore, long now) {
        if (shotCount >= MAX_SHOTS) {
            return;
        }
        shotSpeed[shotCount] = speedBefore;
        shotRecoverySec[shotCount] = Double.NaN;
        shotCount++;
        lastShotNs = now;
        awaitingRecovery = true;
    }

    private void pushSample(long now, double value) {
        windowNs[windowHead] = now;
        windowVel[windowHead] = value;
        windowHead = (windowHead + 1) % WINDOW_SIZE;
        if (windowCount < WINDOW_SIZE) {
            windowCount++;
        }
    }

    private double windowMax(long now) {
        long oldest = now - (long) (Constants.SHOOTER_SHOT_WINDOW_SEC * 1.0e9);
        double max = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < windowCount; i++) {
            int index = (windowHead - 1 - i + WINDOW_SIZE) % WINDOW_SIZE;
            if (windowNs[index] < oldest) {
                break;
            }
            max = Math.max(max, windowVel[index]);
        }
        return max;
    }

    public Command updateCommand() {
        return infinite(this::update);
    }

    public Command toggleCommand() {
        return instant(this::toggle);
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    public Command setVelocityCommand(double velocity) {
        return instant(() -> setTargetVelocity(velocity));
    }

    public Command spinUpCommand(double velocity) {
        return instant(() -> {
            setTargetVelocity(velocity);
            setRunning(true);
            update();
        });
    }
}
