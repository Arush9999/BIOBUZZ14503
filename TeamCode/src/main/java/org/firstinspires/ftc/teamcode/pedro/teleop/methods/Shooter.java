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

/**
 * Flywheel velocity control with:
 * - software PIDF (P/I/F on raw velocity; D on low-pass-filtered dV/dt)
 * - derivative-on-measurement dampening
 * - battery voltage compensation on F and P (tune at NOMINAL volts)
 */
public class Shooter {

    private final DcMotorEx shooter;
    private final HardwareMap hardwareMap;
    private boolean running = false;
    private double targetVelocity = Constants.SHOOTER_TARGET_VELOCITY;
    private double filteredVoltage = Constants.SHOOTER_NOMINAL_VOLTAGE;
    private double compensatedF = Constants.SHOOTER_F;
    private double compensatedP = Constants.SHOOTER_P;

    private double lastRawVelocity = 0.0;
    private double filteredDerivative = 0.0;
    private double integral = 0.0;
    private double lastPower = 0.0;
    private long lastNs = 0L;
    private boolean derivativeSeeded = false;

    public Shooter(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        // Software PID writes motor power; encoder velocity is still available.
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        filteredVoltage = readBatteryVoltage();
        applyPIDF();
        stop();
    }

    /** Refresh voltage-compensated P/F used by the software loop. */
    public void applyPIDF() {
        updateVoltage();
        double voltage = Math.max(filteredVoltage, Constants.SHOOTER_MIN_VOLTAGE);
        double scale = Constants.SHOOTER_NOMINAL_VOLTAGE / voltage;
        compensatedF = Constants.SHOOTER_F * scale;
        compensatedP = Constants.SHOOTER_P * scale;
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = Math.max(0, velocity);
        Constants.SHOOTER_TARGET_VELOCITY = targetVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getBatteryVoltage() {
        return filteredVoltage;
    }

    public double getCompensatedF() {
        return compensatedF;
    }

    public double getCompensatedP() {
        return compensatedP;
    }

    /** Low-pass filtered velocity derivative (ticks/s²) used by D. */
    public double getFilteredDerivative() {
        return filteredDerivative;
    }

    public double getRawVelocity() {
        return shooter.getVelocity();
    }

    public double getLastPower() {
        return lastPower;
    }

    public void setRunning(boolean running) {
        this.running = running;
        if (!running) {
            stop();
        }
    }

    public void toggle() {
        setRunning(!running);
    }

    public boolean isRunning() {
        return running;
    }

    public void update() {
        if (running && targetVelocity > 0) {
            applyPIDF();
            lastPower = computePidfPower();
            shooter.setPower(lastPower);
        } else {
            stopMotor();
        }
    }

    public void stop() {
        running = false;
        stopMotor();
    }

    public DcMotorEx getShooter() {
        return shooter;
    }

    private void stopMotor() {
        integral = 0.0;
        lastNs = 0L;
        lastPower = 0.0;
        filteredDerivative = 0.0;
        derivativeSeeded = false;
        shooter.setPower(0.0);
    }

    /**
     * P/I/F use raw velocity. Only the D term is low-pass filtered
     * (filtered dV/dt) so dampening does not amplify encoder noise.
     */
    private double computePidfPower() {
        long now = System.nanoTime();
        double dt = (lastNs == 0L) ? 0.02 : (now - lastNs) / 1.0e9;
        lastNs = now;
        dt = Range.clip(dt, 0.005, 0.05);

        double rawVelocity = shooter.getVelocity();
        double error = targetVelocity - rawVelocity;
        integral += error * dt;
        integral = Range.clip(integral, -Constants.SHOOTER_I_MAX, Constants.SHOOTER_I_MAX);

        double rawDerivative = 0.0;
        if (!derivativeSeeded) {
            lastRawVelocity = rawVelocity;
            filteredDerivative = 0.0;
            derivativeSeeded = true;
        } else {
            rawDerivative = (rawVelocity - lastRawVelocity) / dt;
            double alpha = Range.clip(Constants.SHOOTER_D_FILTER, 0.01, 1.0);
            filteredDerivative += alpha * (rawDerivative - filteredDerivative);
            lastRawVelocity = rawVelocity;
        }

        double pTerm = compensatedP * error;
        double iTerm = Constants.SHOOTER_I * integral;
        double dTerm = -Constants.SHOOTER_D * filteredDerivative; // dampening on filtered dV/dt
        double fTerm = compensatedF * targetVelocity;

        return Range.clip(
                (pTerm + iTerm + dTerm + fTerm) / Constants.SHOOTER_PID_SCALE,
                -1.0,
                1.0);
    }

    private void updateVoltage() {
        double raw = readBatteryVoltage();
        double alpha = Constants.SHOOTER_VOLTAGE_FILTER;
        filteredVoltage += alpha * (raw - filteredVoltage);
    }

    private double readBatteryVoltage() {
        double min = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                min = Math.min(min, voltage);
            }
        }
        return Double.isFinite(min) ? min : Constants.SHOOTER_NOMINAL_VOLTAGE;
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
