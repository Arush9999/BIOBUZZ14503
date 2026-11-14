package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Transfer {

    private final DcMotor transfer;

    public Transfer(HardwareMap hardwareMap) {
        transfer = hardwareMap.dcMotor.get("transfer");
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Transfer reverses whenever intake is active.
     * Bumper override runs transfer forward (wired to RB in teleop).
     */
    public void run(boolean intakeActive, boolean forwardBumper) {
        if (forwardBumper) {
            setPower(Constants.TRANSFER_POWER);
        } else if (intakeActive) {
            setPower(-Constants.TRANSFER_POWER);
        } else {
            stop();
        }
    }

    public void run(double intakeForwardTrigger, double intakeReverseTrigger, boolean forwardBumper) {
        boolean intakeActive =
                intakeForwardTrigger > Constants.INTAKE_TRIGGER_DEADZONE
                        || intakeReverseTrigger > Constants.INTAKE_TRIGGER_DEADZONE;
        run(intakeActive, forwardBumper);
    }

    public void setPower(double power) {
        transfer.setPower(power);
    }

    public void stop() {
        transfer.setPower(0);
    }

    public DcMotor getTransfer() {
        return transfer;
    }

    // ---------------- Ivy helper commands ----------------

    public Command teleOpCommand(DoubleSupplier intakeForward, DoubleSupplier intakeReverse,
                                 BooleanSupplier forwardBumper) {
        return infinite(() -> run(
                intakeForward.getAsDouble(),
                intakeReverse.getAsDouble(),
                forwardBumper.getAsBoolean()
        ));
    }

    public Command forwardCommand() {
        return instant(() -> setPower(Constants.TRANSFER_POWER));
    }

    public Command reverseCommand() {
        return instant(() -> setPower(-Constants.TRANSFER_POWER));
    }

    public Command stopCommand() {
        return instant(this::stop);
    }
}
