package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

public class Shooter {

    private final DcMotorEx shooter;
    private boolean running = false;
    private double targetVelocity = Constants.SHOOTER_TARGET_VELOCITY;

    public Shooter(HardwareMap hardwareMap) {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorEx.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        applyPIDF();
        stop();
    }

    public void applyPIDF() {
        shooter.setVelocityPIDFCoefficients(
                Constants.SHOOTER_P,
                Constants.SHOOTER_I,
                Constants.SHOOTER_D,
                Constants.SHOOTER_F
        );
    }

    public void setTargetVelocity(double velocity) {
        targetVelocity = Math.max(0, velocity);
        Constants.SHOOTER_TARGET_VELOCITY = targetVelocity;
    }

    public double getTargetVelocity() {
        return targetVelocity;
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
            shooter.setVelocity(targetVelocity);
        } else {
            stop();
        }
    }

    public void stop() {
        shooter.setVelocity(0);
    }

    public DcMotorEx getShooter() {
        return shooter;
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
