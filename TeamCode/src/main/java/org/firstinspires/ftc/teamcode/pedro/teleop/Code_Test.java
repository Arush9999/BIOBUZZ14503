package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Code_Test extends LinearOpMode {

    public static double DRIVE_POWER = 0.8;
    public static double INTAKE_POWER = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor LF = hardwareMap.dcMotor.get("LF");
        DcMotor LB = hardwareMap.dcMotor.get("LB");
        DcMotor RF = hardwareMap.dcMotor.get("RF");
        DcMotor RB = hardwareMap.dcMotor.get("RB");
        //DcMotor Intake = hardwareMap.dcMotor.get("Intake");

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        //Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        //Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            LF.setPower(frontLeftPower * DRIVE_POWER);
            LB.setPower(backLeftPower * DRIVE_POWER);
            RF.setPower(frontRightPower * DRIVE_POWER);
            RB.setPower(backRightPower * DRIVE_POWER);
            
            // Right trigger = intake, left trigger = reverse / unjam
            /* if (gamepad1.right_trigger > 0.1) {
                Intake.setPower(INTAKE_POWER * gamepad1.right_trigger);
            } else if (gamepad1.left_trigger > 0.1) {
                Intake.setPower(-INTAKE_POWER * gamepad1.left_trigger);
            } else {
                Intake.setPower(0);
            } */
        }
    }
}