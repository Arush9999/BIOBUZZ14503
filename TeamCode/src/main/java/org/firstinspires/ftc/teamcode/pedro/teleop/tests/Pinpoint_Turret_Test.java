package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Robot_Centric_Drive;

/**
 * Test teleop: turret follows Pinpoint heading (theta).
 *
 * Hardware: "turretL" and "turretR" (mirrored / reverse facing).
 * Drive the robot to change heading and watch the turret counter-rotate
 * so it holds a field-relative aim. Press gamepad1 Back to reset pose.
 */
@TeleOp(name = "Pinpoint Turret Test", group = "BIOBUZZ")
public class Pinpoint_Turret_Test extends LinearOpMode {

    // Heading range mapped onto the full servo travel (±90° -> 0..1).
    public static double TURRET_HEADING_RANGE_DEGREES = 90.0;
    public static double TURRET_CENTER = 0.5;
    public static double TURRET_MIN = 0.0;
    public static double TURRET_MAX = 1.0;

    private final PoseFactory degrees = PoseFactory.degrees();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot_Centric_Drive drive = new Robot_Centric_Drive(hardwareMap);
        PinpointLocalizer pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);

        Servo turretL = hardwareMap.get(Servo.class, "turretL");
        Servo turretR = hardwareMap.get(Servo.class, "turretR");

        pinpoint.setPose(degrees.of(0, 0, 0));
        setTurret(turretL, turretR, TURRET_CENTER);

        telemetry.addLine("Pinpoint Turret Test");
        telemetry.addLine("Drive to change heading; turret tracks theta.");
        telemetry.addLine("Back = reset Pinpoint pose / heading");
        telemetry.update();

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

            if (gamepad1.back) {
                pinpoint.setPose(degrees.of(0, 0, 0));
            }

            pinpoint.update();
            Pose pose = pinpoint.state().pose();
            double headingDeg = Math.toDegrees(pose.heading());

            // Counter-rotate with robot heading so the turret holds field direction.
            double leftPos = headingToServo(headingDeg);
            setTurret(turretL, turretR, leftPos);

            telemetry.addData("Heading (deg)", "%.2f", headingDeg);
            telemetry.addData("turretL", "%.3f", leftPos);
            telemetry.addData("turretR", "%.3f", TURRET_CENTER * 2.0 - leftPos);
            telemetry.update();
        }
    }

    private static double headingToServo(double headingDegrees) {
        double normalized = Range.clip(
                headingDegrees / TURRET_HEADING_RANGE_DEGREES,
                -1.0,
                1.0
        );
        // Negative so the turret cancels robot rotation.
        return Range.clip(TURRET_CENTER - normalized * (TURRET_CENTER - TURRET_MIN), TURRET_MIN, TURRET_MAX);
    }

    private static void setTurret(Servo turretL, Servo turretR, double leftPos) {
        leftPos = Range.clip(leftPos, TURRET_MIN, TURRET_MAX);
        // Reverse-facing pair: mirror about center.
        turretL.setPosition(leftPos);
        turretR.setPosition(TURRET_CENTER * 2.0 - leftPos);
    }
}
