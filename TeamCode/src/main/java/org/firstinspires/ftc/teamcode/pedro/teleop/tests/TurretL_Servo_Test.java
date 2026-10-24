package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Manual test: move turretL with A / B.
 */
@TeleOp(name = "TurretL Servo Test", group = "BIOBUZZ")
public class TurretL_Servo_Test extends LinearOpMode {

    public static double TURRET_CENTER = 0.5;
    public static double TURRET_MIN = 0.0;
    public static double TURRET_MAX = 1.0;
    public static double TURRET_STEP = 0.02;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo turretL = hardwareMap.get(Servo.class, "turretL");
        double position = TURRET_CENTER;
        turretL.setPosition(position);

        telemetry.addLine("TurretL Servo Test");
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
                position = TURRET_MIN;
            } else if (gamepad1.y) {
                position = TURRET_MAX;
            } else if (gamepad1.a && !aWasPressed) {
                position = Range.clip(position + TURRET_STEP, TURRET_MIN, TURRET_MAX);
            } else if (gamepad1.b && !bWasPressed) {
                position = Range.clip(position - TURRET_STEP, TURRET_MIN, TURRET_MAX);
            }

            aWasPressed = gamepad1.a;
            bWasPressed = gamepad1.b;

            turretL.setPosition(position);

            telemetry.addLine("TurretL Servo Test");
            telemetry.addLine("A = increase    B = decrease");
            telemetry.addLine("X = position 0  Y = position 1");
            telemetry.addData("turretL", "%.4f", position);
            telemetry.addData("step", "%.4f", TURRET_STEP);
            telemetry.update();
        }
    }
}
