package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "LLTester")
public class LimelightIDTester extends LinearOpMode {
    private Limelight3A limelight;
    private DcMotor LF, RF, LB, RB;

    // TODO: Measure and set these constants physically on robot (all heights in cm)
    private final double camHeight = 36;   // cm
    private final double tagHeight = 135.9;  // cm
    private final double targetHeight = (135.9 + 166.6) / 2;
    private double smoothed = 0;
    private int lastId = -1;

    private final int[] redIDs = {30, 31, 32, 33, 34, 35, 36, 37};
    public static double DRIVE_SPEED = 1.0;
    public static double SLOW_SPEED = 0.40;


    // turret constants
    private Servo tL;
    private Servo tR;
    public static double TURRET_CENTER = 0.5;
    public static double TURRET_SIGN = -1;
    public static double DEG_PER_SERVO = 315;
    public static double TURRET_MIN = 0.2, TURRET_MAX = 0.8;

    public static double TURRET_KP = 0.3;
    public static double error = 0.5;
    private double turretDeg = 0;
    private boolean tracking = false;

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        LF = hardwareMap.get(DcMotorEx.class, "LF");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RB = hardwareMap.get(DcMotorEx.class, "RB");

        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        tL = hardwareMap.get(Servo.class, "turretL");
        tR = hardwareMap.get(Servo.class, "turretR");
        tR.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            driveRobot();
            tracking = false;

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                LLResultTypes.FiducialResult tag = getGoalTagOrBest(result);

                if (tag != null && contains(redIDs, tag.getFiducialId())) {
                    int id = tag.getFiducialId();
                    Pose3D p = tag.getTargetPoseCameraSpace();

                    if (result.getStaleness() < 100) {
                        double tx = tag.getTargetXDegrees();
                        if (Math.abs(tx) < error) tx = 0;
                        turretDeg -= TURRET_KP * tx;
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

                    setTurretAngle(turretDeg);
                    telemetry.addData("Turret", "%.1f° (%s)", turretDeg, tracking ? "TRACKING" : "holding");
                    telemetry.addData("Tag ID", id);
                    telemetry.addData("tx", "%.2f", tag.getTargetXDegrees());
                } else {
                    telemetry.addLine("No AprilTags in view");
                }
            } else {
                telemetry.addLine("No valid Limelight result");
            }
            telemetry.update();
        }
        limelight.stop();
    }

    private void driveRobot() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double speed = gamepad1.left_bumper ? SLOW_SPEED : DRIVE_SPEED; // left bumper = slow mode

        double frontLeft = y + x + turn;
        double frontRight = y - x - turn;
        double backLeft = y - x + turn;
        double backRight = y + x - turn;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeft), Math.max(Math.abs(frontRight),
                Math.max(Math.abs(backLeft), Math.abs(backRight)))));

        LF.setPower(frontLeft / max * speed);
        RF.setPower(frontRight / max * speed);
        LB.setPower(backLeft / max * speed);
        RB.setPower(backRight / max * speed);
    }

    private void setTurretAngle(double angle) {
        double pos = TURRET_CENTER + TURRET_SIGN * angle / DEG_PER_SERVO;
        pos = Math.max(TURRET_MIN, Math.min(TURRET_MAX, pos));
        turretDeg = (pos - TURRET_CENTER) * DEG_PER_SERVO / TURRET_SIGN;
        tL.setPosition(pos);
        tR.setPosition(pos);
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