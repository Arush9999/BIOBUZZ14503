package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "LLTester")
public class LimelightIDTester extends LinearOpMode {
    private Limelight3A limelight;
    private Servo turret;

    // TODO: Measure and set these constants physically on robot

    private final double camHeight = 15.0;
    private final double tagHeight = 135.9;  // cm
    private final double targetHeight = (135.9+166.6)/2;
    private final double downhiveHeight = 77.7;  // cm
    private double smoothed = 0;
    private int lastId = -1;
    private double lastError = 0;

    // for turret servo
    private final double SERVO_RANGE_DEG = 300;
    private final double CENTER_POS = 0.5;
    private final double MIN_ANGLE = -90, MAX_ANGLE = 90;
    private final double GOAL_OFFSET_DEG = 0;
    private final double STEP = 0.5;
    private final double TOL_DEG = 0.5;
    private double DIRECTION = 1;
    private double turretAngle;
    private boolean hasTarget;
    private final int[] redIDs = {30,31,32,33,34,35,36,37};
    private final int[] blueIDs = {38,39,40,41,42,43,44,45};

    private final ElapsedTime timer = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        turret = hardwareMap.get(Servo.class, "turret");
        waitForStart();

        while (opModeIsActive()) {


            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                LLResultTypes.FiducialResult tag = getBestTag(result);

                if (tag != null) {
                    int id = tag.getFiducialId();
                    Pose3D p = tag.getTargetPoseCameraSpace();
                    double tx = tag.getTargetXDegrees();

                    double error = tag.getTargetXDegrees();
                    if (Math.abs(error) > 0.5) {
                        setTurretAngle(turretAngle + DIRECTION * STEP * error);
                    }
                    telemetry.addData("Turret err (deg)", "%.2f", error);

                    if (p != null) {
                        telemetry.addData("Pose (x,y,z): ","%.3f,%.3f%.3f", p.getPosition().x,p.getPosition().y,p.getPosition().z);
                    }
                    //straight line euclidian distance from the cam to the tag center multiplied by 100 for cm
                    double distCm = Math.sqrt(p.getPosition().x*p.getPosition().x + p.getPosition().y*p.getPosition().y + p.getPosition().z*p.getPosition().z) * 100;

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

                    telemetry.addData("Tag ID", id);
                    telemetry.addData("Camera -> tag (cm)", "%.1f", smoothed);
                    telemetry.addData("Floor dist (cm)", "%.1f", floorDist);
                    telemetry.addData("Aim angle to cell top (deg)", "%.1f", aimAngle);

                    if (contains(redIDs, id))       telemetry.addLine("Alliance: RED");
                    else if (contains(blueIDs, id)) telemetry.addLine("Alliance: BLUE");
                    else                            telemetry.addLine("Alliance: unknown ID");


                    telemetry.addData("tx / ty", "%.2f / %.2f", result.getTx(), result.getTy());
                }


            }else {
                telemetry.addLine("Tag detected but ID not recognized");
            }

            telemetry.update();
        }
        limelight.stop();
    }

    private void setTurretAngle(double angleDeg) {
        turretAngle = Range.clip(angleDeg, MIN_ANGLE, MAX_ANGLE);
        double pos = CENTER_POS + turretAngle / SERVO_RANGE_DEG;
        turret.setPosition(Range.clip(pos, 0, 1));
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

