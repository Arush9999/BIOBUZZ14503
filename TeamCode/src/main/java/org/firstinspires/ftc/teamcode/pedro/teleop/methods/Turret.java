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
 * Dual positional (non-continuous) turret servos — both get the same position.
 * Number-line motion only (never wraps 0↔1): low→high = CCW, high→low = CW.
 *
 * Aim: atan2(goal − predictedPose) − heading → normalize → offsets → servo pos → slew.
 */
public class Turret {

    private final Servo turretL;
    private final Servo turretR;

    private double position = Constants.TURRET_CENTER;
    private double aimDegrees = 0.0;
    private boolean aimLimited = false;
    /** Manual trim (servo units) on top of auto-aim / forward hold. */
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
        setPosition(angleToServo(aimDegrees));
    }

    public void center() {
        aimLimited = false;
        clearTrim();
        setAngle(0.0);
    }

    /** Face robot-forward ({@link Constants#TURRET_CENTER}) plus current trim. */
    public void holdForward() {
        setPosition(Constants.TURRET_CENTER + trimPos);
    }

    public void nudge(double delta) {
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

    /** Map robot-relative turret angle (deg) onto the positional servo range. */
    public static double angleToPosition(double angleDeg) {
        return angleToServo(normalizeAngle(angleDeg));
    }

    public void update(Follower follower) {
        aimAt(follower, Constants.GOAL_X, Constants.GOAL_Y);
    }

    /**
     * Servo position that points at the goal from the current Pinpoint pose
     * (no trim, no slew). Used by hybrid aim every loop.
     */
    public double odometryTargetPosition(Follower follower) {
        return odometryTargetPosition(follower, Constants.GOAL_X, Constants.GOAL_Y);
    }

    public double odometryTargetPosition(Follower follower, double goalX, double goalY) {
        Pose pose = follower.pose();
        double headingRad = pose.heading();
        double thetaRobot = Math.toDegrees(headingRad);

        double predictedX = pose.x()
                + Constants.TURRET_OFFSET_X * Math.cos(headingRad)
                - Constants.TURRET_OFFSET_Y * Math.sin(headingRad);
        double predictedY = pose.y()
                + Constants.TURRET_OFFSET_X * Math.sin(headingRad)
                + Constants.TURRET_OFFSET_Y * Math.cos(headingRad);

        double dx = goalX - predictedX;
        double dy = goalY - predictedY;

        double turretTarget = Math.toDegrees(Math.atan2(dy, dx)) - thetaRobot;
        turretTarget = normalizeAngle(turretTarget);
        turretTarget += Constants.TURRET_OFFSET;
        turretTarget = normalizeAngle(turretTarget);
        turretTarget = -turretTarget + Constants.TURRET_POSITION_OFFSET;

        return angleToServo(normalizeAngle(turretTarget));
    }

    /** Slew toward a servo target without wrapping through the rear. */
    public void slewTo(double targetPos) {
        aimLimited = targetPos <= Constants.TURRET_MIN || targetPos >= Constants.TURRET_MAX;
        slewPositionTo(targetPos);
    }

    public void aimAt(Follower follower, double goalX, double goalY) {
        double targetPos = odometryTargetPosition(follower, goalX, goalY) + trimPos;
        slewTo(targetPos);
    }

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

    /** Slew along the servo number line only — never wraps through the rear. */
    private void slewPositionTo(double targetPos) {
        targetPos = Range.clip(targetPos, Constants.TURRET_MIN, Constants.TURRET_MAX);
        double delta = targetPos - position;
        delta = Range.clip(delta, -Constants.TURRET_MAX_STEP_POS, Constants.TURRET_MAX_STEP_POS);
        setPosition(position + delta);
    }

    /** Servo pos falls as angle rises; clamped to the 0..1 command range. */
    private static double angleToServo(double angle) {
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
