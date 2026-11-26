package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

/**
 * Manual test for positional (non-continuous) turretL.
 * Steps by degrees within TURRET_MIN_ANGLE..TURRET_MAX_ANGLE.
 */
@TeleOp(name = "TurretL_Servo_Test", group = "BIOBUZZ")
public class TurretL_Servo_Test extends LinearOpMode {

    private static final double ANGLE_STEP = 5.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo turretL = hardwareMap.get(Servo.class, "turretL");

        double angleDeg = 0.0;
        double position = angleToPos(angleDeg);
        turretL.setPosition(position);

        telemetry.addLine("TurretL positional (NOT continuous) — PS5");
        telemetry.addLine("Cross/Circle = +/-5 deg   Square = min   Triangle = max   Share = 0 deg");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        boolean crossWasPressed = false;
        boolean circleWasPressed = false;
        boolean squareWasPressed = false;
        boolean triangleWasPressed = false;
        boolean shareWasPressed = false;

        while (opModeIsActive()) {
            if (gamepad1.square && !squareWasPressed) {
                angleDeg = Constants.TURRET_MIN_ANGLE;
            } else if (gamepad1.triangle && !triangleWasPressed) {
                angleDeg = Constants.TURRET_MAX_ANGLE;
            } else if (gamepad1.share && !shareWasPressed) {
                angleDeg = 0.0;
            } else if (gamepad1.cross && !crossWasPressed) {
                angleDeg = Range.clip(
                        angleDeg + ANGLE_STEP,
                        Constants.TURRET_MIN_ANGLE,
                        Constants.TURRET_MAX_ANGLE);
            } else if (gamepad1.circle && !circleWasPressed) {
                angleDeg = Range.clip(
                        angleDeg - ANGLE_STEP,
                        Constants.TURRET_MIN_ANGLE,
                        Constants.TURRET_MAX_ANGLE);
            }

            crossWasPressed = gamepad1.cross;
            circleWasPressed = gamepad1.circle;
            squareWasPressed = gamepad1.square;
            triangleWasPressed = gamepad1.triangle;
            shareWasPressed = gamepad1.share;

            position = Range.clip(angleToPos(angleDeg), Constants.TURRET_MIN, Constants.TURRET_MAX);
            turretL.setPosition(position);

            telemetry.addLine("TurretL positional (NOT continuous) — PS5");
            telemetry.addLine("Cross/Circle = +/-5 deg   Square = min   Triangle = max   Share = 0 deg");
            telemetry.addData("angle deg", "%.1f", angleDeg);
            telemetry.addData("servo pos", "%.4f", position);
            telemetry.addData("pos limits", "%.2f .. %.2f",
                    Constants.TURRET_MIN, Constants.TURRET_MAX);
            telemetry.update();
        }
    }

    /** Same mapping as Turret.set_pos — positional travel only. */
    private static double angleToPos(double angle) {
        double raw = Constants.TURRET_POS_AT_MIN_ANGLE
                - (angle - Constants.TURRET_MIN_ANGLE) * Constants.TURRET_DEGREE_TO_SERVO;
        return Math.min(
                Constants.TURRET_POS_AT_MIN_ANGLE,
                Math.max(Constants.TURRET_POS_AT_MAX_ANGLE, raw));
    }
}
