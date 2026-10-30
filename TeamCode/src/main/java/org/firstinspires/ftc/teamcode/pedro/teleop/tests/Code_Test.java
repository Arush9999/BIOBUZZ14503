package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Intake;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Robot_Centric_Drive;

@TeleOp
public class Code_Test extends LinearOpMode {

    public static double TURRET_CENTER = 0.5;
    public static double TURRET_MIN = 0.0;
    public static double TURRET_MAX = 1.0;
    public static double TURRET_STEP = 0.02;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot_Centric_Drive drive = new Robot_Centric_Drive(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Servo turretL = hardwareMap.get(Servo.class, "turretL");

        double turretPosition = TURRET_CENTER;
        turretL.setPosition(turretPosition);

        telemetry.addLine("A = increase    B = decrease");
        telemetry.addLine("X = position 0  Y = position 1");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        boolean aWasPressed = false;
        boolean bWasPressed = false;

        while (opModeIsActive()) {
            drive.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
            );

            intake.run(gamepad1.right_trigger, gamepad1.left_trigger);

            if (gamepad1.x) {
                turretPosition = TURRET_MIN;
            } else if (gamepad1.y) {
                turretPosition = TURRET_MAX;
            } else if (gamepad1.a && !aWasPressed) {
                turretPosition = Range.clip(turretPosition + TURRET_STEP, TURRET_MIN, TURRET_MAX);
            } else if (gamepad1.b && !bWasPressed) {
                turretPosition = Range.clip(turretPosition - TURRET_STEP, TURRET_MIN, TURRET_MAX);
            }

            aWasPressed = gamepad1.a;
            bWasPressed = gamepad1.b;

            turretL.setPosition(turretPosition);

            telemetry.addLine("A = increase    B = decrease");
            telemetry.addLine("X = position 0  Y = position 1");
            telemetry.addData("turretL", "%.4f", turretPosition);
            telemetry.addData("step", "%.4f", TURRET_STEP);
            telemetry.update();
        }
    }
}
