package org.firstinspires.ftc.teamcode.pedro;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "TurretTesterwAprTags")
public class TurretTesterwAprTags extends LinearOpMode {
    private DcMotorEx turret;

    private double kp = 0.0001;
    private double kd = 0.0001;

    private int goalX = 0;
    private int goalY = 0;

    private double angleTolerance = 0.2;

    private final ElapsedTime timer = new ElapsedTime();
    public void resetTimer(){
        timer.reset();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        turret = hardwareMap.get(DcMotorEx.class, "TurretMotor");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        waitForStart();

        while (opModeIsActive()){
            resetTimer();
        }


    }


}
