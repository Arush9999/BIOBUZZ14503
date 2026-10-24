package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Manual test: move turretR with A / B.
 */
@TeleOp(name = "TurretR Servo Test", group = "BIOBUZZ")
public class TurretR_Servo_Test extends LinearOpMode {

    public static double TURRET_CENTER = 0.5;
    public static double TURRET_MIN = 0.0;
    public static double TURRET_MAX = 1.0;
    public static double TURRET_STEP = 0.02;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo turretR = hardwareMap.get(Servo.class, "turretR");
        double position = TURRET_CENTER;
        turretR.setPosition(position);

        telemetry.addLine("TurretR Servo Test");
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

            turretR.setPosition(position);

            telemetry.addLine("TurretR Servo Test");
            telemetry.addLine("A = increase    B = decrease");
            telemetry.addLine("X = position 0  Y = position 1");
            telemetry.addData("turretR", "%.4f", position);
            telemetry.addData("step", "%.4f", TURRET_STEP);
            telemetry.update();
        }
    }
}
