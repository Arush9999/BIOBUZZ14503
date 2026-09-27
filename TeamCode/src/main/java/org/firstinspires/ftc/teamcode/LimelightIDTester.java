package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Autonomous (name = "LLTester")
public class LimelightIDTester extends LinearOpMode {
    private Limelight3A limelight;

    // TODO: Measure and set these constants physically on robot
    private final double cameraHeight = 15.0; // cm
    private final double cameraAngle = 20.0;  // degrees
    private final double minhiveHeight = 135.9;  // cm
    private final double downhiveHeight = 77.7;  // cm




    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx();
                double ty = result.getTy();
                Pose3D botpose = result.getBotpose();
                Integer seenId = getVisibleTagId(result);
                telemetry.addData("Tag ID seen", seenId != null ? seenId : "none");
                double rawDist = getDistance(ty);
                //double targetRPM = interpolateRPM(distance);
                telemetry.addData("distance", rawDist);
                // telemetry.addData("target RPM", targetRPM);

                if (botpose != null) {
                    telemetry.addData("BotPose (X, Y)", "%.2f, %.2f", botpose.getPosition().x, botpose.getPosition().y);
                    telemetry.addData("BotPose Yaw: ", botpose.getOrientation().getYaw());
                }
                else{
                    telemetry.addLine("No Valid Data Found");
                }
                telemetry.addData("tx", tx);
                telemetry.addData("ty", ty);
                telemetry.addData("Distance: ", rawDist);


            }else {
                telemetry.addLine("Tag detected but ID not recognized");
            }


            telemetry.update();
        }
        limelight.stop();
    }

    private double getDistance(double ty) {
        double angleToTarget = cameraAngle + ty;
        double heightDiff = minhiveHeight-cameraHeight;

        return heightDiff/(Math.tan(Math.toRadians(angleToTarget)));
    }

    //gets id of seen april tag or return null if doesnt see
    private Integer getVisibleTagId(LLResult r){
        List<LLResultTypes.FiducialResult> fiducials = r.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        LLResultTypes.FiducialResult best = fiducials.get(0);
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getTargetArea() > best.getTargetArea()) best = f;
        }
        return best.getFiducialId();
    }

}
//limelight in bottom half of robot
//sense  how many balls are in the cell to calc weight determine how many more balls needed
//turret based on where we are on the field calculate the height

//april tag mapping this friday
//april tag position next friday
