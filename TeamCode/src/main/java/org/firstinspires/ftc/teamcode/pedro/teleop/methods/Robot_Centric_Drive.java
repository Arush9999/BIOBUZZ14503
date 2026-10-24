package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

public class Robot_Centric_Drive {

    private final DcMotor LF;
    private final DcMotor LB;
    private final DcMotor RF;
    private final DcMotor RB;

    public Robot_Centric_Drive(HardwareMap hardwareMap) {
        LF = hardwareMap.dcMotor.get("LF");
        LB = hardwareMap.dcMotor.get("LB");
        RF = hardwareMap.dcMotor.get("RF");
        RB = hardwareMap.dcMotor.get("RB");

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);
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
}
