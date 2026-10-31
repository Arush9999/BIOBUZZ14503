package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor Power Shooter", group = "Testing")
public class Motor_Power_Shooter extends LinearOpMode {

    private static final double FULL_POWER = 1.0;

    @Override
    public void runOpMode() {
        DcMotorEx shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        DcMotorEx shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        shooterLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterRight.setDirection(DcMotor.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterLeft.setPower(0);
        shooterRight.setPower(0);

        telemetry.addLine("MOTOR POWER SHOOTER");
        telemetry.addLine("Both shooter motors run at full power after start.");
        telemetry.update();

        waitForStart();

        shooterLeft.setPower(FULL_POWER);
        shooterRight.setPower(FULL_POWER);

        while (opModeIsActive()) {
            telemetry.addData("shooterLeft", "%.2f", shooterLeft.getPower());
            telemetry.addData("shooterRight", "%.2f", shooterRight.getPower());
            telemetry.update();
        }

        shooterLeft.setPower(0);
        shooterRight.setPower(0);
    }
}
