package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.teleop.methods.LimelightVision;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.VisionTurretAim;

@TeleOp(name = "Vision Turret Aim", group = "tests")
public class VisionTurretAimTest extends OpMode {

    private LimelightVision vision;
    private Turret turret;
    private VisionTurretAim aim;

    private boolean redAlliance = true;
    private boolean aimEnabled = true;
    private boolean lastA = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());

        vision = new LimelightVision(hardwareMap, redAlliance);
        turret = new Turret(hardwareMap);   // starts at TURRET_CENTER
        aim = new VisionTurretAim(vision, turret);

        telemetry.addLine("Vision Turret Aim ready");
        telemetry.update();
    }

    @Override
    public void start() {
        vision.start();
    }

    @Override
    public void loop() {
        // Alliance select.
        if (gamepad1.dpad_left && !redAlliance) {
            redAlliance = true;
            vision.setAlliance(true);
        }
        if (gamepad1.dpad_right && redAlliance) {
            redAlliance = false;
            vision.setAlliance(false);
        }

        // A toggles auto-aim, X recenters.
        if (gamepad1.a && !lastA) aimEnabled = !aimEnabled;
        lastA = gamepad1.a;
        if (gamepad1.x) turret.center();

        vision.update();   // must come before aim.update()

        if (aimEnabled) {
            aim.update();  // vision-only; pass a Follower here for odometry fallback
        } else {
            // Manual nudge with right stick when auto-aim is off.
            turret.nudge(-gamepad1.right_stick_x * 0.01);
        }

        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Auto Aim", aimEnabled ? "ON" : "OFF");
        aim.addTelemetry(telemetry);
        vision.addTelemetry(telemetry);

        telemetry.addLine("DPAD L/R: alliance | A: toggle aim | X: center");
        telemetry.update();
    }

    @Override
    public void stop() {
        vision.stop();
    }
}
