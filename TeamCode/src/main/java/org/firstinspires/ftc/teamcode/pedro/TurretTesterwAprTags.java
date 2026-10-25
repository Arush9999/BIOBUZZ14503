package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Disabled
@TeleOp(name = "TurretTesterwAprTags")
public class TurretTesterwAprTags extends OpMode {
    private DcMotorEx turret;

    private double kp = 0.0001;
    private double kd = 0.0001;

    private int goalX = 0;
    private int goalY = 0;

    private double angleTolerance = 0.2;
    private double lastError = 0;

    private Limelight3A limelight;

    private double power = 0;
    private final double MAX_POWER = 0.6;

    private final ElapsedTime timer = new ElapsedTime();


    public void init(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void setkP(double newkP){
        kp=newkP;
    }
    public double getKp(){
        return kp;
    }

    public void setkD(double newkD){
        kd=newkD;
    }
    public double getKd(){
        return kd;
    }


    public void resetTimer(){
        timer.reset();
    }

    public void update(LLResult currID) {
        double deltaTime = timer.seconds();
        timer.reset();

        if (currID == null) {
            turret.setPower(0);
            lastError = 0;
        }

        double error = goalX - currID.getTx();
        double p = error*kp;
        double d = 0;

        if(deltaTime>0){
            d = ((error-lastError)/deltaTime)*kd;
        }
        if(Math.abs((error))< angleTolerance){
            power = 0;
        }else{
            power = Range.clip(p+d, -MAX_POWER, MAX_POWER);
        }

        turret.setPower(power);
        lastError = error;
    }


    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}
