package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Intake;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Robot_Centric_Drive;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;

@TeleOp(name = "TeleOp V1", group = "BIOBUZZ")
public class TeleOpV1 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot_Centric_Drive drive = new Robot_Centric_Drive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Turret turret = new Turret(hardwareMap);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            drive.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            intake.run(gamepad1.right_trigger, gamepad1.left_trigger);

            turret.run(gamepad2.right_stick_x);

            telemetry.addData("Turret", "%.4f", turret.getPosition());
            telemetry.addData("LT", "%.4f", turret.getLT().getPosition());
            telemetry.addData("RT", "%.4f", turret.getRT().getPosition());
            telemetry.update();
        }
    }
}
