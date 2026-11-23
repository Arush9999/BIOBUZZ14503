//package org.firstinspires.ftc.teamcode.pedro.teleop.tests;
//
//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.math.Pose;
//import com.pedropathing.revhub.localizers.PinpointLocalizer;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
//import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Drivetrain;
//
///**
// * New BIOBUZZ reflected turret aiming test.
// * Only the Pedro localization and drivetrain APIs are taken from the old file.
// * Servo movement is active when the driver toggles auto aim with RB.
// * Confirm both servo command directions and the turret forward zero before use.
// * No turret encoder: the angle shown is commanded/estimated, NOT measured.
// */
//@TeleOp(name = "BIOBUZZ Turret Reflected", group = "BIOBUZZ")
//public class BiobuzzTurretReflected extends LinearOpMode {
//    // Auto aim starts OFF; RB enables real servo movement.
//    // Both servos are assumed to respond in the same command direction.
//    private static final double START_X = 12.0;
//    private static final double START_Y = 120.0;
//    private static final double START_HEADING_DEG = 90.0;
//
//    // Local robot coordinates: +X forward, +Y left (per supplied code).
//    // Turret is three inches behind the localization tracking point.
//    // IMPORTANT: measure offset from the Pinpoint tracking origin, not chassis center.
//    private static final double TURRET_FORWARD_IN = -3.0;
//    private static final double TURRET_LEFT_IN = 0.0;
//    // Aiming references only; never change the Pedro coordinate origin.
//    // Reference line from earlier team geometry. Verify on field.
//    private static final double RED_REFERENCE_X = 58.0;
//    private static final double BLUE_REFERENCE_X = 86.0;
//    private static final double TARGET_LINE_Y = 72.0;
//
//    // 148-tooth turret gear driven by 145-tooth servo pinions.
//    private static final double SERVO_DEG_PER_TURRET_DEG = 148.0 / 145.0;
//    // Nominal travel; the full +/-180 turret range requires 367.45 servo degrees.
//    // Replace with calibrated servo command-to-angle span before enabling outputs.
//    private static final double SERVO_FULL_TRAVEL_DEG = 360.0;
//    private static final double LEFT_ZERO = 0.5;
//    private static final double RIGHT_ZERO = 0.5;
//    // Both servos use the same sign, as specified by the team.
//    // +1 assumes increasing position rotates the turret counterclockwise.
//    // Change BOTH signs to -1 if a small verified test shows the reverse.
//    private static final double LEFT_SIGN = 1.0;
//    private static final double RIGHT_SIGN = 1.0;
//    // Physically tested turret limits; verify servo command endpoints separately.
//    private static final double MIN_TURRET_DEG = -180.0;
//    private static final double MAX_TURRET_DEG = 180.0;
//    // Standard positional command range. These are NOT calibrated
//    // mechanical endpoints. Commands beyond this range are rejected.
//    private static final double LEFT_MIN = 0.2;
//    private static final double LEFT_MAX = 0.8;
//    private static final double RIGHT_MIN = 0.2;
//    private static final double RIGHT_MAX = 0.8;
//    // Rate limit is per loop, not a constant speed; no position feedback.
//    private static final double MAX_STEP_DEG_PER_LOOP = 1.0;
//
//    private final PoseFactory degrees = PoseFactory.degrees();
//    private boolean redAlliance = true;
//    private boolean autoAim = false;
//    private boolean previousRB = false;
//    private boolean previousX = false;
//    private boolean previousBack = false;
//    private double commandedAngleDeg = 0.0;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        Drivetrain drive = new Drivetrain(hardwareMap);
//        PinpointLocalizer pinpoint =
//                new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
//        Servo turretL = hardwareMap.get(Servo.class, "turretL");
//        Servo turretR = hardwareMap.get(Servo.class, "turretR");
//
//        // Keep Pedro coordinates unchanged; do not recenter around (72,72) or (58,72).
//        // Reset only when physically at the configured start pose.
//        // Do not overwrite an inherited autonomous pose automatically.
//        telemetry.addLine("BIOBUZZ_TurretReflected");
//        telemetry.addLine("RB: toggle auto aim | X: red/blue | BACK: reset pose");
//        telemetry.addLine("BACK only when physically at configured start pose");
//        telemetry.addLine("Servo commands ACTIVE when RB enables auto aim");
//        telemetry.addLine("Align turret forward at servo 0.5 before starting");
//        telemetry.update();
//        waitForStart();
//        if (isStopRequested()) return;
//
//        try {
//            while (opModeIsActive()) {
//                drive.drive(-gamepad1.left_stick_y,
//                        gamepad1.left_stick_x, gamepad1.right_stick_x);
//                if (gamepad1.right_bumper && !previousRB) autoAim = !autoAim;
//                if (gamepad1.x && !previousX) redAlliance = !redAlliance;
//                if (gamepad1.back && !previousBack) {
//                    pinpoint.setPose(degrees.of(START_X, START_Y, START_HEADING_DEG));
//                }
//                previousRB = gamepad1.right_bumper;
//                previousX = gamepad1.x;
//                previousBack = gamepad1.back;
//
//                pinpoint.update();
//                Pose pose = pinpoint.state().pose();
//                double heading = pose.heading(); // radians
//                double turretX = pose.x() + TURRET_FORWARD_IN * Math.cos(heading)
//                        - TURRET_LEFT_IN * Math.sin(heading);
//                double turretY = pose.y() + TURRET_FORWARD_IN * Math.sin(heading)
//                        + TURRET_LEFT_IN * Math.cos(heading);
//                double referenceX = redAlliance ? RED_REFERENCE_X : BLUE_REFERENCE_X;
//                double targetX = 2.0 * referenceX - turretX;
//                double targetY = TARGET_LINE_Y;
//                double dx = targetX - turretX;
//                double dy = targetY - turretY;
//                double desiredDeg = Math.toDegrees(Math.atan2(dy, dx) - heading);
//                double validAngle = shortestValidAngle(desiredDeg, commandedAngleDeg);
//                String status = "AIM OFF";
//
//                if (autoAim) {
//                    if (Math.hypot(dx, dy) < 0.01) {
//                        status = "TARGET AT TURRET - NO ANGLE";
//                    } else if (Double.isNaN(validAngle)) {
//                        status = "UNREACHABLE - HOLD";
//                    } else {
//                        double next = commandedAngleDeg + Range.clip(
//                                validAngle - commandedAngleDeg,
//                                -MAX_STEP_DEG_PER_LOOP, MAX_STEP_DEG_PER_LOOP);
//                        double left = Range.clip(
//                                servoPosition(next, LEFT_ZERO, LEFT_SIGN), LEFT_MIN, LEFT_MAX);
//                        double right = Range.clip(
//                                servoPosition(next, RIGHT_ZERO, RIGHT_SIGN), RIGHT_MIN, RIGHT_MAX);
//                        turretL.setPosition(left);
//                        turretR.setPosition(right);
//                        commandedAngleDeg = next;
//                        boolean limited = left <= LEFT_MIN || left >= LEFT_MAX
//                                || right <= RIGHT_MIN || right >= RIGHT_MAX;
//                        status = limited ? "SERVO LIMIT (CLIPPED)" : "COMMANDING (NO FEEDBACK)";
//                    }
//                }
//
//                telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
//                telemetry.addData("Auto aim", autoAim);
//                telemetry.addData("Status", status);
//                telemetry.addData("Robot", "x=%.1f y=%.1f h=%.1f deg",
//                        pose.x(), pose.y(), Math.toDegrees(heading));
//                telemetry.addData("Turret field", "x=%.1f y=%.1f", turretX, turretY);
//                telemetry.addData("Target", "x=%.1f y=%.1f", targetX, targetY);
//                telemetry.addData("Relative target deg", "%.1f", desiredDeg);
//                telemetry.addData("Valid angle deg", Double.isNaN(validAngle)
//                        ? "NONE" : String.format(java.util.Locale.US, "%.1f", validAngle));
//                telemetry.addData("Commanded angle deg", "%.1f", commandedAngleDeg);
//                telemetry.addData("L/R planned", "%.3f / %.3f",
//                        servoPosition(commandedAngleDeg, LEFT_ZERO, LEFT_SIGN),
//                        servoPosition(commandedAngleDeg, RIGHT_ZERO, RIGHT_SIGN));
//                telemetry.update();
//            }
//        } finally {
//            drive.stop();
//        }
//    }
//
//    // Choose the closest equivalent angle within the safe physical interval.
//    // Never force a 360-degree wrap through an endpoint.
//    private static double shortestValidAngle(double targetDeg, double currentDeg) {
//        double best = Double.NaN;
//        double shortest = Double.POSITIVE_INFINITY;
//        for (int k = -3; k <= 3; k++) {
//            double candidate = targetDeg + 360.0 * k;
//            if (candidate < MIN_TURRET_DEG || candidate > MAX_TURRET_DEG) continue;
//            double distance = Math.abs(candidate - currentDeg);
//            if (distance < shortest) {
//                shortest = distance;
//                best = candidate;
//            }
//        }
//        return best;
//    }
//
//    private static double servoPosition(double turretDeg, double center, double sign) {
//        return center + sign * turretDeg * SERVO_DEG_PER_TURRET_DEG
//                / SERVO_FULL_TRAVEL_DEG;
//    }
//}
