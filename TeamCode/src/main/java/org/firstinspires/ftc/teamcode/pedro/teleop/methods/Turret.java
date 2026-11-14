package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

/**
 * Dual-servo turret with always-on odometry auto-aim.
 *
 * Servo map (top-down, robot forward up):
 *   0.5 = forward
 *   0 and 1 meet at the rear
 *
 * Relative aim of -180..+180 deg maps linearly onto 0..1:
 *   position = 0.5 + direction * angleDeg / 360
 */
public class Turret {

    private final Servo turretL;
    private final Servo turretR;

    private double position = Constants.TURRET_CENTER;
    private double aimDegrees = 0.0;
    private boolean aimLimited = false;

    public Turret(HardwareMap hardwareMap) {
        turretL = hardwareMap.get(Servo.class, "turretL");
        turretR = hardwareMap.get(Servo.class, "turretR");
        turretR.setDirection(Servo.Direction.REVERSE);
        setPosition(Constants.TURRET_CENTER);
    }

    public void setPosition(double requested) {
        position = Range.clip(requested, Constants.TURRET_MIN, Constants.TURRET_MAX);
        turretL.setPosition(position);
        turretR.setPosition(position);
    }

    public void center() {
        aimLimited = false;
        aimDegrees = 0.0;
        setPosition(Constants.TURRET_CENTER);
    }

    public void nudge(double delta) {
        setPosition(position + delta);
    }

    public boolean isAimLimited() {
        return aimLimited;
    }

    public double getPosition() {
        return position;
    }

    public double getAimDegrees() {
        return aimDegrees;
    }

    /**
     * Convert a robot-relative turret angle (deg) to a servo position.
     * 0 deg = forward (0.5). +/-180 deg = rear (0 or 1).
     */
    public static double angleToPosition(double aimDegrees) {
        double wrapped = wrapDegrees(aimDegrees);
        double requested = Constants.TURRET_CENTER
                + Constants.TURRET_DIRECTION * wrapped / 360.0;
        return Range.clip(requested, Constants.TURRET_MIN, Constants.TURRET_MAX);
    }

    /** Always-on odometry aim at the configured field goal. */
    public void update(Follower follower) {
        aimAt(follower, Constants.GOAL_X, Constants.GOAL_Y);
    }

    public void aimAtGoal(Follower follower) {
        aimAt(follower, Constants.GOAL_X, Constants.GOAL_Y);
    }

    public void aimAt(Follower follower, double goalX, double goalY) {
        Pose pose = follower.pose();
        double heading = pose.heading();

        double turretX = pose.x()
                + Constants.TURRET_OFFSET_X * Math.cos(heading)
                - Constants.TURRET_OFFSET_Y * Math.sin(heading);
        double turretY = pose.y()
                + Constants.TURRET_OFFSET_X * Math.sin(heading)
                + Constants.TURRET_OFFSET_Y * Math.cos(heading);

        double fieldAngle = Math.atan2(goalY - turretY, goalX - turretX);
        double relativeRad = wrapRadians(fieldAngle - heading);
        aimDegrees = wrapDegrees(Math.toDegrees(relativeRad) - Constants.TURRET_MOUNT_OFFSET_DEG);

        double requested = Constants.TURRET_CENTER
                + Constants.TURRET_DIRECTION * aimDegrees / 360.0;
        aimLimited = requested < Constants.TURRET_MIN || requested > Constants.TURRET_MAX;
        setPosition(requested);
    }

    /** Kept for existing call sites. */
    public void aimAtHive(Follower follower) {
        update(follower);
    }

    // ---------------- Ivy helper commands ----------------

    public Command centerCommand() {
        return instant(this::center);
    }

    public Command setPositionCommand(double requested) {
        return instant(() -> setPosition(requested));
    }

    public Command autoAimCommand(Follower follower) {
        return infinite(() -> update(follower));
    }

    private static double wrapDegrees(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }

    private static double wrapRadians(double rad) {
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }
}
