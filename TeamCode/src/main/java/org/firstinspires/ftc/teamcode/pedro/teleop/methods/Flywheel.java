package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.teleop.constants.FlywheelConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Flywheel speed control that behaves the same from a full battery to a tired one.
 *
 * Every loop:
 * - RECOVER: more than a few percent below target -> full power.
 * - HOLD: volts = KS + KV*target + KP*error + KI*integral, then power = volts / battery.
 * - Output is 0..1, so the wheel coasts down and never brakes.
 *
 * The motor control uses the raw speed reading. The ready check and the shot log use a
 * lightly smoothed speed (SPEED_SMOOTH_SEC), because single readings scatter by a few percent.
 * Also records the wheel speed at the moment each ball hits, for telemetry.
 */
public class Flywheel {

    public static final int MAX_SHOTS = 8;
    private static final double FALLBACK_VOLTAGE = 12.5;
    private static final int WINDOW_SIZE = 64;

    private final DcMotorEx motor;
    private final List<VoltageSensor> voltageSensors = new ArrayList<>();

    private boolean running = false;
    private boolean openLoop = false;
    private boolean feeding = false;
    private double openLoopVolts = 0.0;
    private double targetVelocity = FlywheelConstants.TARGET_VELOCITY;

    private double velocity = 0.0;
    private double smoothVelocity = 0.0;
    private boolean smoothSeeded = false;
    private double batteryVoltage;
    private double integral = 0.0;
    private double lastPower = 0.0;
    private double sentPower = Double.NaN;
    private double readyTime = 0.0;
    private double loopSec = 0.0;
    private long lastNs = 0L;
    private long lastVoltageNs = 0L;
    private String mode = "OFF";

    // Shot tracking
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

    public Flywheel(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "shooter");
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        // Our loop writes motor power; the encoder speed is still readable in this mode.
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltageSensors.add(sensor);
        }
        double volts = readBatteryVoltage();
        batteryVoltage = Double.isNaN(volts) ? FALLBACK_VOLTAGE : volts;

        sendPower(0.0);
    }

    // ---------------- Commands from the OpMode ----------------

    public void setTargetVelocity(double velocity) {
        targetVelocity = Math.max(0, velocity);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    /** Closed-loop speed control on/off. */
    public void setRunning(boolean running) {
        this.running = running;
        openLoop = false;
        if (!running) {
            resetState();
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
        lastPower = 0.0;
        mode = "OFF";
        sendPower(0.0);
    }

    /** Tuner only: apply a fixed voltage (battery compensated) with no speed control. */
    public void runOpenLoopVolts(double volts) {
        running = true;
        openLoop = true;
        openLoopVolts = Math.max(0, volts);
        readyTime = 0.0;
    }

    /**
     * Tell the flywheel the transfer is pushing balls in. While true it recovers harder
     * (BURST_RECOVER_BAND) and watches for the speed drop of each ball.
     */
    public void setFeeding(boolean feeding) {
        if (feeding == this.feeding) {
            return;
        }
        long now = System.nanoTime();
        if (feeding) {
            if (!hasFed || (now - feedEndNs) / 1.0e9 > FlywheelConstants.BURST_GAP_SEC) {
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

    // ---------------- Control loop: call once per OpMode loop ----------------

    public void update() {
        long now = System.nanoTime();
        double dt = (lastNs == 0L) ? 0.02 : Range.clip((now - lastNs) / 1.0e9, 0.002, 0.1);
        lastNs = now;
        loopSec = dt;

        velocity = motor.getVelocity(); // the only speed read per loop
        if (!smoothSeeded) {
            smoothVelocity = velocity;
            smoothSeeded = true;
        } else {
            double alpha = dt / (Math.max(FlywheelConstants.SPEED_SMOOTH_SEC, 0.0) + dt);
            smoothVelocity += alpha * (velocity - smoothVelocity);
        }
        updateVoltage(now);
        double battery = Math.max(batteryVoltage, FlywheelConstants.MIN_VOLTAGE);

        if (!running || (!openLoop && targetVelocity <= 0)) {
            mode = "OFF";
            lastPower = 0.0;
            readyTime = 0.0;
            sendPower(0.0);
            return;
        }

        if (openLoop) {
            mode = "OPEN";
            lastPower = Range.clip(openLoopVolts / battery, 0.0, 1.0);
            readyTime = 0.0;
            sendPower(lastPower);
            return;
        }

        // While feeding, aim a bit high so the dip across the burst centers on the real target.
        double target = targetVelocity
                + (feeding ? Math.max(0.0, FlywheelConstants.BURST_OFFSET) : 0.0);
        double error = target - velocity;
        double band = (feeding ? FlywheelConstants.BURST_RECOVER_BAND : FlywheelConstants.RECOVER_BAND)
                * Math.max(target, 1.0);

        double power;
        if (error > band) {
            // Far below target: nothing recovers faster than full power. Integral is frozen.
            mode = "RECOVER";
            power = 1.0;
        } else {
            mode = "HOLD";
            double ffVolts = FlywheelConstants.KS + FlywheelConstants.KV * target;
            double pVolts = FlywheelConstants.KP * error;
            double kI = FlywheelConstants.KI;

            if (kI > 0) {
                double limit = FlywheelConstants.I_MAX_VOLTS / kI;
                if (Math.abs(error) <= FlywheelConstants.I_ZONE * target) {
                    double unclipped = (ffVolts + pVolts + kI * integral) / battery;
                    boolean pushingPastLimit = (unclipped >= 1.0 && error > 0)
                            || (unclipped <= 0.0 && error < 0);
                    if (!pushingPastLimit) {
                        integral += error * dt;
                    }
                }
                integral = Range.clip(integral, -limit, limit);
            } else {
                integral = 0.0;
            }

            power = (ffVolts + pVolts + kI * integral) / battery;
        }

        lastPower = Range.clip(power, 0.0, 1.0); // never brake
        sendPower(lastPower);

        boolean inTolerance =
                Math.abs(target - smoothVelocity) <= FlywheelConstants.READY_TOL * target;
        readyTime = inTolerance ? readyTime + dt : 0.0;

        trackShots(now);
    }

    // ---------------- Status ----------------

    /** Smoothed speed has been within READY_TOL of target for READY_SEC. */
    public boolean atSpeed() {
        return running && !openLoop && readyTime >= FlywheelConstants.READY_SEC;
    }

    /** Power needed just to hold the target at the present battery voltage. */
    public double holdPowerNeeded() {
        double battery = Math.max(batteryVoltage, FlywheelConstants.MIN_VOLTAGE);
        return (FlywheelConstants.KS + FlywheelConstants.KV * targetVelocity) / battery;
    }

    /** False when the battery is too low to hold the target speed. */
    public boolean canHoldTarget() {
        return holdPowerNeeded() <= FlywheelConstants.MAX_HOLD_POWER;
    }

    /** Speed from the most recent update(), ticks/s. */
    public double getVelocity() {
        return velocity;
    }

    /** Speed averaged over about SPEED_SMOOTH_SEC; used by the ready check and the shot log. */
    public double getSmoothedVelocity() {
        return smoothVelocity;
    }

    public double getLastPower() {
        return lastPower;
    }

    /** Smoothed battery voltage used for compensation. */
    public double getBatteryVoltage() {
        return batteryVoltage;
    }

    /** Voltage actually being applied to the motor (power * battery). */
    public double getAppliedVolts() {
        return lastPower * batteryVoltage;
    }

    public double getLoopMs() {
        return loopSec * 1000.0;
    }

    /** OFF, OPEN, RECOVER or HOLD. */
    public String getMode() {
        return mode;
    }

    public DcMotorEx getMotor() {
        return motor;
    }

    // ---------------- Shot log (current burst) ----------------

    public int getShotCount() {
        return shotCount;
    }

    /** Wheel speed just before ball {@code index} hit it. */
    public double getShotSpeed(int index) {
        return shotSpeed[index];
    }

    /** Seconds from ball {@code index} until the wheel was ready again; NaN if the next ball came first. */
    public double getShotRecoverySec(int index) {
        return shotRecoverySec[index];
    }

    /** Fastest minus slowest shot in the burst, as a percent of target. */
    public double getShotSpreadPercent() {
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

    /** First shot minus last shot, as a percent of target. */
    public double getShotDropPercent() {
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

    // ---------------- Internals ----------------

    private void resetState() {
        readyTime = 0.0;
        inDip = false;
        windowCount = 0;
        windowHead = 0;
        awaitingRecovery = false;
    }

    private void sendPower(double power) {
        // Skip the hub write when nothing changed (saves a few ms per loop when stopped).
        if (power != sentPower) {
            motor.setPower(power);
            sentPower = power;
        }
    }

    private void updateVoltage(long now) {
        double since = (now - lastVoltageNs) / 1.0e9;
        if (lastVoltageNs != 0L && since < FlywheelConstants.VOLTAGE_READ_SEC) {
            return;
        }
        double raw = readBatteryVoltage();
        if (!Double.isNaN(raw)) {
            if (lastVoltageNs == 0L) {
                batteryVoltage = raw;
            } else {
                double alpha = since / (Math.max(FlywheelConstants.VOLTAGE_TAU_SEC, 0.0) + since);
                batteryVoltage += alpha * (raw - batteryVoltage);
            }
        }
        lastVoltageNs = now;
    }

    /** Lowest valid reading across the hubs, or NaN if none answered. */
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
                || (hasFed && (now - feedEndNs) / 1.0e9 < FlywheelConstants.SHOT_WATCH_AFTER_FEED_SEC);
        if (!watching) {
            windowCount = 0;
            windowHead = 0;
            inDip = false;
            return;
        }

        // Shot tracking uses the smoothed speed so reading scatter is not counted as a ball.
        double speed = smoothVelocity;
        if (inDip) {
            // Wait for the speed to turn back up before looking for the next ball.
            dipMin = Math.min(dipMin, speed);
            if (speed >= dipMin + FlywheelConstants.SHOT_REARM * targetVelocity) {
                inDip = false;
                windowCount = 0;
                windowHead = 0;
                pushSample(now, speed);
            }
            return;
        }

        pushSample(now, speed);
        double before = windowMax(now);
        if (before - speed >= FlywheelConstants.SHOT_DROP * targetVelocity) {
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

    /** Highest speed seen in the last SHOT_WINDOW_SEC. */
    private double windowMax(long now) {
        long oldest = now - (long) (FlywheelConstants.SHOT_WINDOW_SEC * 1.0e9);
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

    // ---------------- Ivy helper commands ----------------

    public Command updateCommand() {
        return infinite(this::update);
    }

    public Command toggleCommand() {
        return instant(this::toggle);
    }

    public Command stopCommand() {
        return instant(this::stop);
    }

    public Command spinUpCommand(double velocity) {
        return instant(() -> {
            setTargetVelocity(velocity);
            setRunning(true);
        });
    }
}
