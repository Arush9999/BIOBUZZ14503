package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Autonomous (name = "LLTester")
public class LimelightIDTester extends LinearOpMode {
    private Limelight3A limelight;

    // TODO: Measure and set these constants physically on your robot
    private final double cameraHeight = 15.0; // cm
    private final double cameraAngle = 20.0;  // degrees
    private final double minhiveHeight = 135.9;  // cm
    private final double downhiveHeight = 77.7;  // cm


    private double distance = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
        waitForStart();

        while (opModeIsActive()){
            LLResult result = limelight.getLatestResult();
            if(result != null && result.isValid()) {
                double rawDist = getDistance(result.getTy());
                Pose3D botpose = result.getBotpose();
                telemetry.addData("Distance: ", rawDist);

                if(rawDist>minhiveHeight){
                    telemetry.addLine("Able to shoot in!");
                }else if(rawDist>=downhiveHeight && rawDist<minhiveHeight){
                    telemetry.addLine("hive is down so cant shoot in!");
                }else{
                    telemetry.addLine("invalid");
                }

                if (botpose != null) {
                    telemetry.addData("BotPose (X, Y)", "%.2f, %.2f", botpose.getPosition().x, botpose.getPosition().y);
                    telemetry.addData("BotPose Yaw: ", botpose.getOrientation().getYaw());
                }
            }else{
                telemetry.addLine("No Valid Data Found");
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

}
//limelight in bottom half of robot
//sense  how many balls are in the cell to calc weight determine how many more balls needed
//turret based on where we are on the field calculate the height

//april tag mapping this friday
//april tag position next friday
