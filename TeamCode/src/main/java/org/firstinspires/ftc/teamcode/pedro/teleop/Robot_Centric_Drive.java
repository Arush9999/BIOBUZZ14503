package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Robot_Centric_Drive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor LF = hardwareMap.dcMotor.get("LF");
        DcMotor LB = hardwareMap.dcMotor.get("LB");
        DcMotor RF = hardwareMap.dcMotor.get("RF");
        DcMotor RB = hardwareMap.dcMotor.get("RB");

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            LF.setPower(frontLeftPower);
            LB.setPower(backLeftPower);
            RF.setPower(frontRightPower);
            RB.setPower(backRightPower);
        }
    }
}