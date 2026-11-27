package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_GOAL_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_GOAL_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_START_OFFSET_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_GOAL_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_GOAL_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_FAR_HEADING_DEG;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_FAR_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_FAR_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_NEAR_HEADING_DEG;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_NEAR_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_NEAR_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_HEADING_DEG;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_Y;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.LimelightVision;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.TurretAutoAim;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.TurretAutoAim.AimMode;

/**
 * Hybrid aim telemetry test (see Hybrid Turret Aim PDF).
 * Y cycles OFF → ODOMETRY → CAMERA → HYBRID. Estimator runs whenever mode ≠ OFF
 * and a follower is available (CAMERA uses vision-only steps).
 *
 * DPAD L/R alliance · A toggles OFF/HYBRID · X center · stick nudge in OFF.
 */
@TeleOp(name = "Turret Auto Aim", group = "tests")
public class TurretAutoAimTest extends OpMode {

    private Follower follower;
    private LimelightVision vision;
    private Turret turret;
    private TurretAutoAim aim;

    private boolean redAlliance = true;
    /** true = near (y=8 @ 90°), false = far (y=136 @ 270°). */
    private boolean startNear = true;
    private boolean lastA = false;
    private boolean lastY = false;

    private long onTargetSinceMs = 0L;
    private long timeToOnTargetMs = -1L;
    private long runStartMs = 0L;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        follower = Constants.create(hardwareMap);
        applyStartPose();

        vision = new LimelightVision(hardwareMap, redAlliance);
        turret = new Turret(hardwareMap);
        turret.setPosition(turret.odometryTargetPosition(follower));
        aim = new TurretAutoAim(vision, turret);
        aim.setFollower(follower);
        aim.setMode(AimMode.HYBRID);

        telemetry.addLine("Hybrid Turret Aim ready (Y cycles modes)");
        telemetry.addLine("DPAD U/D: near/far start | L/R: alliance");
        telemetry.update();
    }

    @Override
    public void start() {
        vision.start();
        runStartMs = System.currentTimeMillis();
        timeToOnTargetMs = -1L;
        onTargetSinceMs = 0L;
    }

    @Override
    public void loop() {
        follower.update();

        if (gamepad1.dpad_left && !redAlliance) {
            redAlliance = true;
            vision.setAlliance(true);
            applyStartPose();
            aim.reset();
        }
        if (gamepad1.dpad_right && redAlliance) {
            redAlliance = false;
            vision.setAlliance(false);
            applyStartPose();
            aim.reset();
        }
        if (gamepad1.dpad_up && !startNear) {
            startNear = true;
            applyStartPose();
            aim.reset();
        }
        if (gamepad1.dpad_down && startNear) {
            startNear = false;
            applyStartPose();
            aim.reset();
        }

        if (gamepad1.a && !lastA) {
            aim.setMode(aim.getMode() == AimMode.OFF ? AimMode.HYBRID : AimMode.OFF);
            restartTimer();
        }
        lastA = gamepad1.a;

        if (gamepad1.y && !lastY) {
            aim.setMode(nextMode(aim.getMode()));
            restartTimer();
        }
        lastY = gamepad1.y;

        if (gamepad1.x) {
            turret.center();
            aim.reset();
        }

        vision.update();
        if (aim.getMode() == AimMode.OFF) {
            turret.nudge(-gamepad1.right_stick_x * 0.01);
        } else {
            aim.tick();
        }

        updateOnTargetTimer();

        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Start", "%s  %.0f, %.0f @ %.0f°",
                startNear ? "NEAR" : "FAR", START_X, START_Y, START_HEADING_DEG);
        telemetry.addData("TIME TO ON-TARGET (ms)",
                timeToOnTargetMs >= 0 ? timeToOnTargetMs : "—");
        aim.addTelemetry(telemetry);
        vision.addTelemetry(telemetry);
        telemetry.addLine("Y: mode | A: OFF/HYBRID | X: center | DPAD L/R alliance | U/D near/far");
        telemetry.update();
    }

    @Override
    public void stop() {
        vision.stop();
    }

    private void restartTimer() {
        runStartMs = System.currentTimeMillis();
        timeToOnTargetMs = -1L;
        onTargetSinceMs = 0L;
    }

    private static AimMode nextMode(AimMode m) {
        switch (m) {
            case OFF: return AimMode.ODOMETRY;
            case ODOMETRY: return AimMode.CAMERA;
            case CAMERA: return AimMode.HYBRID;
            default: return AimMode.OFF;
        }
    }

    private void updateOnTargetTimer() {
        if (aim.getMode() == AimMode.OFF || timeToOnTargetMs >= 0) {
            return;
        }
        double err = aim.getErrorDeg();
        boolean on = Double.isFinite(err) && Math.abs(err) < 1.0;
        long now = System.currentTimeMillis();
        if (on) {
            if (onTargetSinceMs == 0L) {
                onTargetSinceMs = now;
            } else if (now - onTargetSinceMs >= 66L) {
                timeToOnTargetMs = now - runStartMs;
            }
        } else {
            onTargetSinceMs = 0L;
        }
    }

    private void applyStartPose() {
        double sx = startNear ? RED_START_NEAR_X : RED_START_FAR_X;
        double sy = startNear ? RED_START_NEAR_Y : RED_START_FAR_Y;
        double sh = startNear ? RED_START_NEAR_HEADING_DEG : RED_START_FAR_HEADING_DEG;
        if (!redAlliance) {
            sx += BLUE_START_OFFSET_X;
        }

        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_X =
                redAlliance ? RED_GOAL_X : BLUE_GOAL_X;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_Y =
                redAlliance ? RED_GOAL_Y : BLUE_GOAL_Y;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_X = sx;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_Y = sy;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_HEADING_DEG = sh;

        follower.setPose(PoseFactory.degrees().of(START_X, START_Y, START_HEADING_DEG));
        if (turret != null) {
            turret.clearTrim();
            turret.setPosition(turret.odometryTargetPosition(follower));
        }
    }
}
