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
 * Odometry turret test for the BIOBUZZ field.
 *
 * Pedro origin is the bottom-left corner of the picture. +X is right, +Y is up.
 * Heading 0 faces right. Heading 90 faces up the field.
 *
 * The robot starts at (60, 8) on the bottom edge. The shooter faces forward,
 * up the field, at the red goal. That goal's opening is the front lip in the
 * picture, about (72, 48).
 *
 * Auto-aim is atan2 of the offset from the turret to that point, minus heading.
 * Angle 0 keeps turretL at 0.5, which is shooter-forward.
 */
@TeleOp(name = "Odometry Turret Test", group = "BIOBUZZ")
public class Odometry_Turret_Test extends LinearOpMode {

    // Pedro inches and degrees.
    public static double START_X = 60.0;
    public static double START_Y = 8.0;
    public static double START_HEADING_DEGREES = 135.0;

    // Red goal opening. Measured on the field picture, in Pedro inches.
    public static double GOAL_X = 72.0;
    public static double GOAL_Y = 48.0;

    // Turret pivot measured from the Pinpoint tracking point.
    // +X is robot forward, +Y is robot left.
    public static double TURRET_OFFSET_X = 0.0;
    public static double TURRET_OFFSET_Y = 0.0;

    // The gear reverses the servo. -1 turns the turret the other way.
    public static double TURRET_SIGN = -1.0;
    // Servo : turret. One servo degree moves the turret 1.224 degrees.
    public static double GEAR_RATIO = 1.224;
    // Servo travel of 0..1 covers this many servo degrees, centered on shooter forward.
    public static double TURRET_RANGE_DEGREES = 90.0;

    public static double TURRET_CENTER = 0.5;
    public static double TURRET_MIN = 0.25;
    public static double TURRET_MAX = 0.75;

    private final PoseFactory degrees = PoseFactory.degrees();

    private boolean autoAim = true;
    private boolean lastAim = false;
    private boolean lastPose = false;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot_Centric_Drive drive = new Robot_Centric_Drive(hardwareMap);
        PinpointLocalizer pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);

        Servo turretL = hardwareMap.get(Servo.class, "turretL");
        Servo turretR = hardwareMap.get(Servo.class, "turretR");

        resetPose(pinpoint);
        setTurret(turretL, turretR, TURRET_CENTER);

        telemetry.addLine("ODOMETRY TURRET TEST");
        telemetry.addLine("Pedro (60, 8), shooter facing the red goal.");
        telemetry.addLine("Back = reset pose");
        telemetry.addLine("RB = auto aim");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        try {
            while (opModeIsActive()) {
                drive.drive(
                        -gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x
                );

                readButtons(pinpoint);
                pinpoint.update();

                Pose pose = pinpoint.state().pose();
                double[] turret = turretFieldPosition(pose);
                double offsetX = GOAL_X - turret[0];
                double offsetY = GOAL_Y - turret[1];
                double aimDegrees = Math.toDegrees(normalizeSigned(
                        Math.atan2(offsetY, offsetX) - pose.heading()
                ));
                double leftPos = angleToServo(aimDegrees);

                if (autoAim) {
                    setTurret(turretL, turretR, leftPos);
                }

                showTelemetry(pose, turret, offsetX, offsetY, aimDegrees, leftPos);
            }
        } finally {
            drive.stop();
        }
    }

    private void readButtons(PinpointLocalizer pinpoint) {
        if (gamepad1.right_bumper && !lastAim) {
            autoAim = !autoAim;
        }
        if (gamepad1.back && !lastPose) {
            resetPose(pinpoint);
        }

        lastAim = gamepad1.right_bumper;
        lastPose = gamepad1.back;
    }

    private void resetPose(PinpointLocalizer pinpoint) {
        pinpoint.setPose(degrees.of(START_X, START_Y, START_HEADING_DEGREES));
    }

    private double[] turretFieldPosition(Pose pose) {
        double heading = pose.heading();
        double turretX = pose.x()
                + TURRET_OFFSET_X * Math.cos(heading)
                - TURRET_OFFSET_Y * Math.sin(heading);
        double turretY = pose.y()
                + TURRET_OFFSET_X * Math.sin(heading)
                + TURRET_OFFSET_Y * Math.cos(heading);
        return new double[] {turretX, turretY};
    }

    private double angleToServo(double aimDegrees) {
        double servoDegrees = aimDegrees / GEAR_RATIO;
        double normalized = Range.clip(
                TURRET_SIGN * servoDegrees / TURRET_RANGE_DEGREES,
                -1.0,
                1.0
        );
        double halfTravel = TURRET_CENTER - TURRET_MIN;
        return Range.clip(TURRET_CENTER + normalized * halfTravel, TURRET_MIN, TURRET_MAX);
    }

    private void setTurret(Servo turretL, Servo turretR, double leftPos) {
        leftPos = Range.clip(leftPos, TURRET_MIN, TURRET_MAX);
        turretL.setPosition(leftPos);
        turretR.setPosition(leftPos);
    }

    private void showTelemetry(
            Pose pose,
            double[] turret,
            double offsetX,
            double offsetY,
            double aimDegrees,
            double leftPos
    ) {
        boolean clipped = Math.abs(aimDegrees / GEAR_RATIO) > TURRET_RANGE_DEGREES;

        telemetry.addLine("===== ODOMETRY TURRET =====");
        telemetry.addData("Auto aim", autoAim ? "ON" : "OFF");

        telemetry.addLine("");
        telemetry.addLine("POSE");
        telemetry.addData("X", "%.2f", pose.x());
        telemetry.addData("Y", "%.2f", pose.y());
        telemetry.addData("Heading", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addData("Turret X", "%.2f", turret[0]);
        telemetry.addData("Turret Y", "%.2f", turret[1]);

        telemetry.addLine("");
        telemetry.addLine("OFFSET TO RED GOAL");
        telemetry.addData("Goal X", "%.2f", GOAL_X);
        telemetry.addData("Goal Y", "%.2f", GOAL_Y);
        telemetry.addData("Offset X", "%.2f", offsetX);
        telemetry.addData("Offset Y", "%.2f", offsetY);
        telemetry.addData("Distance", "%.1f", Math.hypot(offsetX, offsetY));
        telemetry.addData("atan2", "%.1f", aimDegrees);
        telemetry.addData("turretL", "%.3f", leftPos);
        telemetry.addData("turretR", "%.3f", leftPos);
        telemetry.addData("In range", clipped ? "CLIPPED" : "YES");

        telemetry.addLine("");
        telemetry.addLine("Back = reset pose    RB = auto aim");
        telemetry.update();
    }

    private static double normalizeSigned(double angle) {
        double tau = Math.PI * 2.0;
        angle = angle % tau;
        if (angle > Math.PI) {
            angle -= tau;
        } else if (angle < -Math.PI) {
            angle += tau;
        }
        return angle;
    }
}
