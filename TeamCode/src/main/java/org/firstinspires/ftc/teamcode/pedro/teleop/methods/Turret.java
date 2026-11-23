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
 * Dual positional (non-continuous) turret servos.
 * Both servos receive the same position and spin the same direction.
 *
 * Position number-line motion (never wraps through the rear 0↔1 cut):
 *   low → high = CCW
 *   high → low = CW
 *
 * Aim math (field → robot-relative → servo):
 *   atan2(goal - predictedPose) → subtract heading → normalize
 *   → turret offset → negate → position offset → set_pos → slew on 0..1
 */
public class Turret {

    private final Servo turretL;
    private final Servo turretR;

    private double position = Constants.TURRET_CENTER;
    private double aimDegrees = 0.0;
    private boolean aimLimited = false;
    /** Manual trim (servo units) added on top of auto-aim; does not disable aim. */
    private double trimPos = 0.0;

    public Turret(HardwareMap hardwareMap) {
        turretL = hardwareMap.get(Servo.class, "turretL");
        turretR = hardwareMap.get(Servo.class, "turretR");
        setPosition(Constants.TURRET_CENTER);
    }

    public void setPosition(double requested) {
        position = Range.clip(requested, Constants.TURRET_MIN, Constants.TURRET_MAX);
        aimDegrees = positionToAngle(position);
        turretL.setPosition(position);
        turretR.setPosition(position);
    }

    public void setAngle(double angleDeg) {
        aimDegrees = Range.clip(angleDeg, Constants.TURRET_MIN_ANGLE, Constants.TURRET_MAX_ANGLE);
        setPosition(setPos(aimDegrees));
    }

    public void center() {
        aimLimited = false;
        clearTrim();
        setAngle(0.0);
    }

    /** Face robot-forward at servo 0.5, plus current trim. */
    public void holdForward() {
        setPosition(0.5 + trimPos);
    }

    public void nudge(double delta) {
        // delta > 0 → higher pos → CCW; delta < 0 → lower pos → CW
        setPosition(position + delta);
    }

    /** Minor correction while auto-aim / forward-hold stays on. */
    public void nudgeTrim(double delta) {
        trimPos += delta;
        double maxTrim = Constants.TURRET_MAX - Constants.TURRET_MIN;
        trimPos = Range.clip(trimPos, -maxTrim, maxTrim);
    }

    public void clearTrim() {
        trimPos = 0.0;
    }

    public double getTrimPos() {
        return trimPos;
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
     * Map a robot-relative turret angle (deg) onto the positional servo range.
     * Matches reference set_pos(): both servos use this same value.
     */
    public static double angleToPosition(double angleDeg) {
        return setPos(normalizeAngle(angleDeg));
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
        double headingRad = pose.heading();
        double thetaRobot = Math.toDegrees(headingRad);

        // predictedX/Y = current pose (add velocity lead later if needed).
        double predictedX = pose.x()
                + Constants.TURRET_OFFSET_X * Math.cos(headingRad)
                - Constants.TURRET_OFFSET_Y * Math.sin(headingRad);
        double predictedY = pose.y()
                + Constants.TURRET_OFFSET_X * Math.sin(headingRad)
                + Constants.TURRET_OFFSET_Y * Math.cos(headingRad);

        double dx = goalX - predictedX;
        double dy = goalY - predictedY;

        double angleToNetField = Math.toDegrees(Math.atan2(dy, dx));
        double turretTarget = angleToNetField - thetaRobot;

        turretTarget = normalizeAngle(turretTarget);
        turretTarget += Constants.TURRET_OFFSET;
        turretTarget = normalizeAngle(turretTarget);
        turretTarget = turretTarget * -1.0;
        turretTarget = turretTarget + Constants.TURRET_POSITION_OFFSET;

        double targetPos = setPos(normalizeAngle(turretTarget)) + trimPos;
        aimLimited = targetPos <= Constants.TURRET_MIN || targetPos >= Constants.TURRET_MAX;
        slewPositionTo(targetPos);
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

    public Command setAngleCommand(double angleDeg) {
        return instant(() -> setAngle(angleDeg));
    }

    public Command autoAimCommand(Follower follower) {
        return infinite(() -> update(follower));
    }

    /**
     * Slew along the servo number line only.
     * Increasing pos = CCW, decreasing pos = CW — never wraps 0↔1 through the rear.
     */
    private void slewPositionTo(double targetPos) {
        targetPos = Range.clip(targetPos, Constants.TURRET_MIN, Constants.TURRET_MAX);
        double delta = targetPos - position;
        delta = Range.clip(delta, -Constants.TURRET_MAX_STEP_POS, Constants.TURRET_MAX_STEP_POS);
        setPosition(position + delta);
    }

    /**
     * Reference set_pos: servo pos falls as angle rises.
     * Clamped only to the 0..1 servo command range.
     */
    private static double setPos(double angle) {
        double raw = Constants.TURRET_POS_AT_MIN_ANGLE
                - (angle - Constants.TURRET_MIN_ANGLE) * Constants.TURRET_DEGREE_TO_SERVO;
        return Math.min(
                Constants.TURRET_POS_AT_MIN_ANGLE,
                Math.max(Constants.TURRET_POS_AT_MAX_ANGLE, raw));
    }

    private static double positionToAngle(double pos) {
        return Constants.TURRET_MIN_ANGLE
                + (Constants.TURRET_POS_AT_MIN_ANGLE - pos) / Constants.TURRET_DEGREE_TO_SERVO;
    }

    private static double normalizeAngle(double deg) {
        while (deg > 180.0) deg -= 360.0;
        while (deg < -180.0) deg += 360.0;
        return deg;
    }
}
