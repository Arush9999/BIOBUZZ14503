package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Motor_Power_Shooter", group = "Testing")
public class Motor_Power_Shooter extends LinearOpMode {

    private static final double FULL_POWER = 1.0;

    @Override
    public void runOpMode() {
        DcMotorEx shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setPower(0);

        telemetry.addLine("MOTOR POWER SHOOTER");
        telemetry.addLine("Shooter runs at full power after start.");
        telemetry.update();

        waitForStart();

        shooter.setPower(FULL_POWER);

        while (opModeIsActive()) {
            telemetry.addData("shooter", "%.2f", shooter.getPower());
            telemetry.update();
        }

        shooter.setPower(0);
    }
}
