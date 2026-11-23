package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

/**
 * Manual test for positional (non-continuous) turretR.
 * Steps by degrees within TURRET_MIN_ANGLE..TURRET_MAX_ANGLE.
 */
@TeleOp(name = "TurretR_Servo_Test", group = "BIOBUZZ")
public class TurretR_Servo_Test extends LinearOpMode {

    private static final double ANGLE_STEP = 5.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo turretR = hardwareMap.get(Servo.class, "turretR");

        double angleDeg = 0.0;
        double position = angleToPos(angleDeg);
        turretR.setPosition(position);

        telemetry.addLine("TurretR positional (NOT continuous)");
        telemetry.addLine("A/B = +/-5 deg   X = min   Y = max   BACK = 0 deg");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        boolean aWasPressed = false;
        boolean bWasPressed = false;
        boolean xWasPressed = false;
        boolean yWasPressed = false;
        boolean backWasPressed = false;

        while (opModeIsActive()) {
            if (gamepad1.x && !xWasPressed) {
                angleDeg = Constants.TURRET_MIN_ANGLE;
            } else if (gamepad1.y && !yWasPressed) {
                angleDeg = Constants.TURRET_MAX_ANGLE;
            } else if (gamepad1.back && !backWasPressed) {
                angleDeg = 0.0;
            } else if (gamepad1.a && !aWasPressed) {
                angleDeg = Range.clip(
                        angleDeg + ANGLE_STEP,
                        Constants.TURRET_MIN_ANGLE,
                        Constants.TURRET_MAX_ANGLE);
            } else if (gamepad1.b && !bWasPressed) {
                angleDeg = Range.clip(
                        angleDeg - ANGLE_STEP,
                        Constants.TURRET_MIN_ANGLE,
                        Constants.TURRET_MAX_ANGLE);
            }

            aWasPressed = gamepad1.a;
            bWasPressed = gamepad1.b;
            xWasPressed = gamepad1.x;
            yWasPressed = gamepad1.y;
            backWasPressed = gamepad1.back;

            position = Range.clip(angleToPos(angleDeg), Constants.TURRET_MIN, Constants.TURRET_MAX);
            turretR.setPosition(position);

            telemetry.addLine("TurretR positional (NOT continuous)");
            telemetry.addLine("A/B = +/-5 deg   X = min   Y = max   BACK = 0 deg");
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
