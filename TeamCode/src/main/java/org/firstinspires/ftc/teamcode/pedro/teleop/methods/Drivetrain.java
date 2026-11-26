package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

import java.util.function.DoubleSupplier;

public class Drivetrain {

    private final DcMotor LF;
    private final DcMotor LB;
    private final DcMotor RF;
    private final DcMotor RB;

    public Drivetrain(HardwareMap hardwareMap) {
        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[]{LF, LB, RF, RB}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }
    }

    public void drive(double forward, double strafe, double turn) {
        double x = strafe * Constants.STRAFE_MULTIPLIER;
        double rx = turn * Constants.TURN_MULTIPLIER;

        double denominator = Math.max(Math.abs(forward) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (forward + x + rx) / denominator;
        double backLeftPower = (forward - x + rx) / denominator;
        double frontRightPower = (forward - x - rx) / denominator;
        double backRightPower = (forward + x - rx) / denominator;

        setPowers(
                frontLeftPower * Constants.DRIVE_POWER,
                backLeftPower * Constants.DRIVE_POWER,
                frontRightPower * Constants.DRIVE_POWER,
                backRightPower * Constants.DRIVE_POWER
        );
    }

    public void setPowers(double frontLeft, double backLeft, double frontRight, double backRight) {
        LF.setPower(frontLeft);
        LB.setPower(backLeft);
        RF.setPower(frontRight);
        RB.setPower(backRight);
    }

    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    public DcMotor getLF() {
        return LF;
    }

    public DcMotor getLB() {
        return LB;
    }

    public DcMotor getRF() {
        return RF;
    }

    public DcMotor getRB() {
        return RB;
    }

    public Command teleOpCommand(DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn) {
        return infinite(() -> drive(
                forward.getAsDouble(),
                strafe.getAsDouble(),
                turn.getAsDouble()
        ));
    }

    public Command stopCommand() {
        return instant(this::stop);
    }
}
