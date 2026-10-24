package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Manual test: move the shooter hood with A / B.
 */
@TeleOp(name = "Shooter Hood Servo Test", group = "BIOBUZZ")
public class ShooterHood_Servo_Test extends LinearOpMode {

    public static double HOOD_CENTER = 0.5;
    public static double HOOD_MIN = 0.0;
    public static double HOOD_MAX = 1.0;
    public static double HOOD_STEP = 0.02;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo hood = hardwareMap.get(Servo.class, "hood");
        double position = HOOD_CENTER;
        hood.setPosition(position);

        telemetry.addLine("Shooter Hood Servo Test");
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
            if (gamepad1.x) {
                position = HOOD_MIN;
            } else if (gamepad1.y) {
                position = HOOD_MAX;
            } else if (gamepad1.a && !aWasPressed) {
                position = Range.clip(position + HOOD_STEP, HOOD_MIN, HOOD_MAX);
            } else if (gamepad1.b && !bWasPressed) {
                position = Range.clip(position - HOOD_STEP, HOOD_MIN, HOOD_MAX);
            }

            aWasPressed = gamepad1.a;
            bWasPressed = gamepad1.b;

            hood.setPosition(position);


            telemetry.addLine("Shooter Hood Servo Test");
            telemetry.addLine("A = increase    B = decrease");
            telemetry.addLine("X = position 0  Y = position 1");
            telemetry.addData("hood", "%.4f", position);
            telemetry.addData("step", "%.4f", HOOD_STEP);
            telemetry.update();
        }
    }
}
