package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.List;

@TeleOp(name = "LLTester")
public class LimelightIDTester extends LinearOpMode {
    private Limelight3A limelight;
    private Follower follower;
    private DcMotor LF,RF,LB,RB;

    // TODO: Measure and set these constants physically on robot (all heights in cm)

    private final double camHeight = 38.1;   // cm
    private final double tagHeight = 135.9;  // cm
    private final double targetHeight = (135.9+166.6)/2;
    private final double downhiveHeight = 77.7;  // cm
    private double smoothed = 0;
    private int lastId = -1;

    private final int[] redIDs = {30,31,32,33,34,35,36,37};
    private final int[] blueIDs = {38,39,40,41,42,43,44,45};

    public static double DRIVE_SPEED = 1.0;
    public static double SLOW_SPEED = 0.40;

    // localization
    private final PoseFactory pf = PoseFactory.degrees();
    private final Pose startPose = pf.of(8, 72, 0);
    private static final double M_TO_IN = 39.3701;
    // Limelight field origin is field center, Pedro origin is a corner.
    private static final double FIELD_OFFSET_X = 72;
    private static final double FIELD_OFFSET_Y = 72;
    private static final double FIELD_SIZE = 144;
    private static final long MAX_STALENESS_MS = 100;
    private static final double MAX_TURN_RATE_DEG_S = 90; // ignore vision while spinning fast
    private static final double STICK_DEADBAND = 0.05;
    public static boolean AUTO_RELOCALIZE = false; // true = correct pose whenever robot is still

    private double lastHeadingDeg = 0;
    private boolean lastA = false;
    private int relocalizeCount = 0;
    private String rejectReason = "";


    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        LF = hardwareMap.get(DcMotorEx.class, "LF");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RB = hardwareMap.get(DcMotorEx.class, "RB");


        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        lastHeadingDeg = Math.toDegrees(follower.pose().heading());
        long lastTime = System.nanoTime();

        while (opModeIsActive()) {
            // update odometry first so manual drive powers below win
            follower.update();
            driveRobot();

            Pose pose = follower.pose();
            double headingDeg = Math.toDegrees(pose.heading());

            long now = System.nanoTime();
            double dt = (now - lastTime) / 1e9;
            lastTime = now;
            double turnRate = dt > 0 ? Math.abs(wrapDeg(headingDeg - lastHeadingDeg)) / dt : 0;
            lastHeadingDeg = headingDeg;

            // MegaTag2 needs the robot's field heading every loop
            boolean orientationSent = limelight.updateRobotOrientation(headingDeg);
            telemetry.addData("Heading sent to LL", "%.1f° (%s)", headingDeg, orientationSent ? "ok" : "FAILED");

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                LLResultTypes.FiducialResult tag = getBestTag(result);

                if (tag != null) {
                    int id = tag.getFiducialId();
                    Pose3D p = tag.getTargetPoseCameraSpace();

                    if (p != null) {
                        double px = p.getPosition().x, py = p.getPosition().y, pz = p.getPosition().z;
                        telemetry.addData("Pose (x,y,z): ","%.3f, %.3f, %.3f", px, py, pz);

                        //straight line euclidian distance from the cam to the tag center multiplied by 100 for cm
                        double distCm = Math.sqrt(px*px + py*py + pz*pz) * 100;

                        if (id != lastId) smoothed = 0;
                        lastId = id;

                        if (distCm > 0) {
                            smoothed = (smoothed == 0) ? distCm : 0.8 * smoothed + 0.2 * distCm;
                        }

                        // Floor distance from camera to the tag
                        double tagRise = tagHeight - camHeight;
                        double floorDist = (smoothed > Math.abs(tagRise))
                                ? Math.sqrt(smoothed * smoothed - tagRise * tagRise):0;

                        double cellRise = targetHeight - camHeight;

                        double aimAngle = Math.toDegrees(Math.atan2(cellRise, floorDist));

                        telemetry.addData("Camera -> tag (cm)", "%.1f", smoothed);
                        telemetry.addData("Floor dist (cm)", "%.1f", floorDist);
                        telemetry.addData("Aim angle to cell top (deg)", "%.1f", aimAngle);
                    } else {
                        telemetry.addLine("Tag has no 3D pose (enable Full 3D in pipeline)");
                    }

                    telemetry.addData("Tag ID", id);
                    if (contains(redIDs, id))       telemetry.addLine("Alliance: RED");
                    else if (contains(blueIDs, id)) telemetry.addLine("Alliance: BLUE");
                    else                            telemetry.addLine("Alliance: unknown ID");

                    telemetry.addData("tx / ty", "%.2f / %.2f", result.getTx(), result.getTy());
                } else {
                    telemetry.addLine("No AprilTags in view");
                }

                // MegaTag2 localization
                Pose visionPose = getVisionPose(result);
                long staleness = result.getStaleness();
                if (visionPose != null && staleness >= MAX_STALENESS_MS) rejectReason = "stale (" + staleness + " ms)";
                else if (visionPose != null && turnRate >= MAX_TURN_RATE_DEG_S) rejectReason = String.format("turning too fast (%.0f°/s)", turnRate);
                boolean usable = visionPose != null
                        && staleness < MAX_STALENESS_MS
                        && turnRate < MAX_TURN_RATE_DEG_S;

                // raw values so you can see why a reading is rejected
                Pose3D mt1 = result.getBotpose();
                Pose3D mt2 = result.getBotpose_MT2();
                telemetry.addData("MT1 raw (m)", "x=%.2f y=%.2f yaw=%.1f",
                        mt1.getPosition().x, mt1.getPosition().y, mt1.getOrientation().getYaw());
                telemetry.addData("MT2 raw (m)", "x=%.2f y=%.2f yaw=%.1f",
                        mt2.getPosition().x, mt2.getPosition().y, mt2.getOrientation().getYaw());
                telemetry.addData("MT2 tags / staleness / turn", "%d / %d ms / %.0f°/s",
                        result.getBotposeTagCount(), staleness, turnRate);
                if (visionPose != null) {
                    telemetry.addData("MT2 pose (Pedro in)", "x=%.1f y=%.1f", visionPose.y(), visionPose.x());
                }
                telemetry.addData("MT2 usable", usable ? "YES" : "no: " + rejectReason);

                boolean aPressed = gamepad1.a && !lastA;
                if (usable && (aPressed || (AUTO_RELOCALIZE && sticksIdle()))) {
                    // keep Pinpoint heading, it's better than the camera's
                    follower.setPose(new Pose(visionPose.x(), visionPose.y(), pose.heading()));
                    relocalizeCount++;
                }
            } else {
                telemetry.addLine("No valid Limelight result");
            }
            lastA = gamepad1.a;

            Pose current = follower.pose();
            telemetry.addData("Pedro pose", "x=%.1f y=%.1f h=%.1f°",
                    current.x(), current.y(), Math.toDegrees(current.heading()));
            telemetry.addData("Relocalized (A to snap)", relocalizeCount);
            telemetry.update();
        }
        limelight.stop();
    }

    // MegaTag2 botpose converted to Pedro coordinates (inches, corner origin), or null if unusable
    // sets rejectReason when it returns null
    private Pose getVisionPose(LLResult r) {
        if (r.getBotposeTagCount() < 1) {
            rejectReason = "botpose tag count 0 (field map / Full 3D?)";
            return null;
        }
        Pose3D bot = r.getBotpose_MT2();
        if (bot == null) {
            rejectReason = "MT2 null";
            return null;
        }

        double x = bot.getPosition().x * M_TO_IN + FIELD_OFFSET_X;
        double y = bot.getPosition().y * M_TO_IN + FIELD_OFFSET_Y;
        if (x == FIELD_OFFSET_X && y == FIELD_OFFSET_Y) { // limelight returns zeros when it has no pose
            rejectReason = "MT2 all zeros";
            return null;
        }
        if (x < 0 || x > FIELD_SIZE || y < 0 || y > FIELD_SIZE) {
            rejectReason = String.format("off field (x=%.1f y=%.1f in)", x, y);
            return null;
        }
        rejectReason = "";
        return new Pose(x, y, 0);
    }

    private boolean sticksIdle() {
        return Math.abs(gamepad1.left_stick_x) < STICK_DEADBAND
                && Math.abs(gamepad1.left_stick_y) < STICK_DEADBAND
                && Math.abs(gamepad1.right_stick_x) < STICK_DEADBAND;
    }

    private double wrapDeg(double a) {
        while (a > 180)  a -= 360;
        while (a < -180) a += 360;
        return a;
    }

    private void driveRobot() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double speed = DRIVE_SPEED;

        // Left bumper = slow mode
        if (gamepad1.left_bumper) {
            speed = SLOW_SPEED;
        }

        // Mecanum calculations

        double frontLeft = y + x + turn;
        double frontRight = y - x - turn;

        double backLeft = y - x + turn;
        double backRight = y + x - turn;

        // Normalize powers

        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(frontLeft),
                        Math.max(
                                Math.abs(frontRight),
                                Math.max(
                                        Math.abs(backLeft),
                                        Math.abs(backRight)
                                )
                        )
                )
        );

        frontLeft /= max;
        frontRight /= max;
        backLeft /= max;
        backRight /= max;

        // Apply power

        LF.setPower(frontLeft * speed);
        RF.setPower(frontRight * speed);
        LB.setPower(backLeft * speed);
        RB.setPower(backRight * speed);
    }
    private LLResultTypes.FiducialResult getBestTag(LLResult r) {
        List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
        if (f == null || f.isEmpty()) return null;
        LLResultTypes.FiducialResult best = f.get(0);
        for (LLResultTypes.FiducialResult t : f)
            if (t.getTargetArea() > best.getTargetArea()) best = t;
        return best;
    }


    private boolean contains(int[] arr, int v) {
        for (int a : arr) if (a == v) return true;
        return false;
    }
}
