package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

import java.util.function.DoubleSupplier;

public class Intake {

    private final DcMotor intake;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.dcMotor.get("Intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run(double forwardTrigger, double reverseTrigger) {
        if (forwardTrigger > Constants.INTAKE_TRIGGER_DEADZONE) {
            setPower(Constants.INTAKE_POWER * forwardTrigger);
        } else if (reverseTrigger > Constants.INTAKE_TRIGGER_DEADZONE) {
            setPower(-Constants.INTAKE_POWER * reverseTrigger);
        } else {
            stop();
        }
    }

    public boolean isActive(double forwardTrigger, double reverseTrigger) {
        return forwardTrigger > Constants.INTAKE_TRIGGER_DEADZONE
                || reverseTrigger > Constants.INTAKE_TRIGGER_DEADZONE;
    }

    public void setPower(double power) {
        intake.setPower(power);
    }

    public void stop() {
        intake.setPower(0);
    }

    public DcMotor getIntake() {
        return intake;
    }

    // ---------------- Ivy helper commands ----------------

    public Command teleOpCommand(DoubleSupplier forwardTrigger, DoubleSupplier reverseTrigger) {
        return infinite(() -> run(forwardTrigger.getAsDouble(), reverseTrigger.getAsDouble()));
    }

    public Command onCommand() {
        return instant(() -> setPower(Constants.INTAKE_POWER));
    }

    public Command stopCommand() {
        return instant(this::stop);
    }
}
