package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Autonomous (name = "LLBlue")
public class LimelightBlueTester extends LinearOpMode {
    private Limelight3A limelight;

    // TODO: Measure and set these constants physically on robot

    private final double camHeight = 15.0;
    private final double minhiveHeight = 135.9;  // cm
    private final double downhiveHeight = 77.7;  // cm
    private double smoothed = 0;
    private int lastId = -1;

    private final int[] blueIDs = {38,39,40,41,42,43,44,45};


    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                LLResultTypes.FiducialResult tag = getBestTag(result);

                if (tag != null) {//
                    int id = tag.getFiducialId();
                    Pose3D p = tag.getTargetPoseCameraSpace();

                    if (p != null) {
                        telemetry.addData("Pose (x,y,z): ","%.3f,%.3f%.3f", p.getPosition().x,p.getPosition().y,p.getPosition().z);
                    }
                    assert p != null;
                    double distCm = Math.sqrt(p.getPosition().x*p.getPosition().x + p.getPosition().y*p.getPosition().y + p.getPosition().z*p.getPosition().z) * 100;

                    if (id != lastId && contains(blueIDs,id)) smoothed = 0;
                    lastId = id;

                    if (distCm > 0) {
                        smoothed = (smoothed == 0) ? distCm : 0.8 * smoothed + 0.2 * distCm;
                    }

                    // Floor distance from camera to the tag
                    double tagRise = downhiveHeight - camHeight;
                    double floorDist = (smoothed > Math.abs(tagRise))
                            ? Math.sqrt(smoothed * smoothed - tagRise * tagRise):0;

                    double cellRise = minhiveHeight - camHeight;
                    double cellDist = Math.sqrt(floorDist * floorDist + cellRise * cellRise);

                    double aimAngle = Math.toDegrees(Math.atan2(cellRise, floorDist));

                    telemetry.addLine("Blue Alliance data");
                    telemetry.addData("Tag ID", id);
                    telemetry.addData("Camera -> tag (cm)", "%.1f", smoothed);
                    telemetry.addData("Floor dist (cm)", "%.1f", floorDist);
                    telemetry.addData("Camera -> cell top (cm)", "%.1f", cellDist);
                    telemetry.addData("Aim angle to cell top (deg)", "%.1f", aimAngle);


                    telemetry.addData("tx / ty", "%.2f / %.2f", result.getTx(), result.getTy());
                }


            }else {
                telemetry.addLine("Tag detected but ID not recognized");
            }

            telemetry.update();
        }
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


    private boolean contains(int[] arr, int v) {
        for (int a : arr) if (a == v) return true;
        return false;
    }
}

