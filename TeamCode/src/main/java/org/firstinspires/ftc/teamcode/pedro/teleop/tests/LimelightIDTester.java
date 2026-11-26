package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;

import java.util.List;

@TeleOp(name = "LLTester")
public class LimelightIDTester extends LinearOpMode {
    private Limelight3A limelight;
    private Drivetrain drive;
    private Turret turret;

    private final double camHeight = 36;   // cm
    private final double tagHeight = 135.9;  // cm
    private final double targetHeight = (135.9 + 166.6) / 2;
    private double smoothed = 0;
    private int lastId = -1;

    private final int[] redIDs = {30, 31, 32, 33, 34, 35, 36, 37};
    public static double SLOW_SPEED = 0.40;

    // change to -1 if the turret turns away from the tag
    public static double AIM_SIGN = 1;
    public static double kp = 0.3;
    public static double error = 0.5;
    private long lastFrameTime = -1;
    private boolean tracking = false;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();


        drive = new Drivetrain(hardwareMap);
        turret = new Turret(hardwareMap);

        waitForStart();

        turret.center();

        while (opModeIsActive()) {
            double speed = gamepad1.left_bumper ? SLOW_SPEED : 1.0;
            drive.drive(-gamepad1.left_stick_y * speed,
                    gamepad1.left_stick_x * speed,
                    gamepad1.right_stick_x * speed);
            tracking = false;

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                LLResultTypes.FiducialResult tag = getGoalTagOrBest(result);

                if (tag != null && contains(redIDs, tag.getFiducialId())) {
                    int id = tag.getFiducialId();
                    Pose3D p = tag.getTargetPoseCameraSpace();

                    if (result.getStaleness() < 100) {
                        long frameTime = result.getControlHubTimeStamp();
                        if (frameTime != lastFrameTime) {
                            lastFrameTime = frameTime;
                            double tx = tag.getTargetXDegrees();
                            if (Math.abs(tx) > error) {
                                turret.setAngle(turret.getAimDegrees() + AIM_SIGN * kp * tx);
                            }
                        }
                        tracking = true;
                    }

                    if (p != null) {
                        double px = p.getPosition().x, py = p.getPosition().y, pz = p.getPosition().z;
                        telemetry.addData("Pose (x,y,z): ", "%.3f, %.3f, %.3f", px, py, pz);

                        // straight line euclidean distance from the cam to the tag center, in cm
                        double distCm = Math.sqrt(px * px + py * py + pz * pz) * 100;

                        if (id != lastId) smoothed = 0;
                        lastId = id;

                        if (distCm > 0) {
                            smoothed = (smoothed == 0) ? distCm : 0.8 * smoothed + 0.2 * distCm;
                        }

                        // Floor distance from camera to the tag
                        double tagRise = tagHeight - camHeight;
                        double floorDist = (smoothed > Math.abs(tagRise))
                                ? Math.sqrt(smoothed * smoothed - tagRise * tagRise) : 0;

                        double cellRise = targetHeight - camHeight;
                        double aimAngle = Math.toDegrees(Math.atan2(cellRise, floorDist));

                        telemetry.addData("Camera -> tag (cm)", "%.1f", smoothed);
                        telemetry.addData("Floor dist (cm)", "%.1f", floorDist);
                        telemetry.addData("Aim angle to cell top (deg)", "%.1f", aimAngle);
                    } else {
                        telemetry.addLine("Tag has no 3D pose (enable Full 3D in pipeline)");
                    }

                    telemetry.addData("Tag ID", id);
                    telemetry.addData("tx", "%.2f", tag.getTargetXDegrees());
                } else {
                    telemetry.addLine("No AprilTags in view");
                }
            } else {
                telemetry.addLine("No valid Limelight result");
            }
            telemetry.addData("Turret", "%.1f° pos %.3f (%s)",
                    turret.getAimDegrees(), turret.getPosition(), tracking ? "TRACKING" : "holding");
            telemetry.update();
        }
        drive.stop();
        limelight.stop();
    }

    private LLResultTypes.FiducialResult getBestTag(LLResult r) {
        List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
        if (f == null || f.isEmpty()) return null;
        LLResultTypes.FiducialResult best = f.get(0);
        for (LLResultTypes.FiducialResult t : f)
            if (t.getTargetArea() > best.getTargetArea()) best = t;
        return best;
    }
    private LLResultTypes.FiducialResult getGoalTagOrBest(LLResult r) {
        List<LLResultTypes.FiducialResult> f = r.getFiducialResults();
        if (f != null) {
            LLResultTypes.FiducialResult goal = null;
            for (LLResultTypes.FiducialResult t : f)
                if (contains(redIDs, t.getFiducialId())
                        && (goal == null || t.getTargetArea() > goal.getTargetArea())) goal = t;
            if (goal != null) return goal;
        }
        return getBestTag(r);
    }

    private boolean contains(int[] arr, int v) {
        for (int a : arr) if (a == v) return true;
        return false;
    }
}

//yeah itll get handed over to kanishka because she's more competent than me
//of course