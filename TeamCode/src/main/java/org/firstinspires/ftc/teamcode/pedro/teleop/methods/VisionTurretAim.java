package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

/**
 * Closes the loop between {@link LimelightVision} and {@link Turret}.
 *
 * On each new Limelight frame, hive AprilTag tx is converted to a servo-position
 * correction. tx &gt; 0 = target right of crosshair → CW → lower servo position.
 * Flip {@link #SIGN} if the camera is mirrored or the turret runs away.
 *
 * If the hive is lost longer than {@link #LOST_TIMEOUT_MS}, falls back to odometry
 * aim when a {@link Follower} is set; otherwise holds last position.
 *
 * Schedule {@link #senseCommand()} then {@link #adjustCommand()} (or call
 * {@code vision.update()} before {@link #update()}). Gate motion with {@link #setActive}.
 */
@Config
public class VisionTurretAim {

    /** +1 or -1. Flip if turret turns away from the hive. */
    public static double SIGN = -1.0;
    /** Fraction of measured error corrected per new frame (0..1). */
    public static double KP = 0.5;
    /** Aim offset in degrees (shifts where the shot centers in the camera). */
    public static double TX_OFFSET_DEG = 0.0;
    /** Ignore |error| below this (deg). */
    public static double DEADBAND_DEG = 1.0;
    /** Max servo-position change per correction. */
    public static double MAX_STEP_POS = 0.015;
    /** Minimum wait after each move before trusting the next frame. */
    public static long SETTLE_MS = 40;
    /** Extra settle time (ms) per 1.0 of servo travel. */
    public static double SETTLE_MS_PER_POS = 1500;
    /** Wait after losing the hive before odometry fallback. */
    public static long LOST_TIMEOUT_MS = 250;
    /** Minimum visible tags in the chosen cell to trust it. */
    public static int MIN_TAGS = 1;

    private final LimelightVision vision;
    private final Turret turret;

    private Follower follower;
    private boolean active = false;

    private boolean locked = false;
    private double lastErrorDeg = Double.NaN;
    private int lastCell = -1;
    private int lastTagCount = 0;
    private long lastSeenMs = 0;
    private long lastMoveMs = 0;
    private long settleNeededMs = 0;

    public VisionTurretAim(LimelightVision vision, Turret turret) {
        this.vision = vision;
        this.turret = turret;
    }

    /** Optional odometry fallback when the hive is lost. */
    public void setFollower(Follower follower) {
        this.follower = follower;
    }

    /** When false, sense still runs but the turret is not moved. */
    public void setActive(boolean active) {
        this.active = active;
    }

    public boolean isActive() {
        return active;
    }

    /** Read Limelight each loop (safe to run while inactive). */
    public Command senseCommand() {
        return infinite(vision::update);
    }

    /** Apply aim corrections only while {@link #isActive()}. */
    public Command adjustCommand() {
        return infinite(() -> {
            if (active) {
                update(follower);
            }
        });
    }

    /** Vision-only aiming; holds position when the hive is not visible. */
    public void update() {
        update(null);
    }

    /** Vision aiming with odometry fallback when the hive is lost. */
    public void update(Follower follower) {
        long now = System.currentTimeMillis();

        int cell = bestCell();
        double avgTx = cell >= 0 ? averageTx(cell) : Double.NaN;

        if (cell >= 0 && Double.isFinite(avgTx)) {
            lastSeenMs = now;
            lastCell = cell;
            locked = true;

            if (vision.hasNewFrame() && now - lastMoveMs >= settleNeededMs) {
                double error = avgTx - TX_OFFSET_DEG;
                lastErrorDeg = error;

                if (Math.abs(error) > DEADBAND_DEG) {
                    double deltaPos = SIGN * -error * KP * Constants.TURRET_DEGREE_TO_SERVO;
                    deltaPos = Range.clip(deltaPos, -MAX_STEP_POS, MAX_STEP_POS);
                    turret.setPosition(turret.getPosition() + deltaPos);
                    lastMoveMs = now;
                    settleNeededMs = SETTLE_MS
                            + (long) (Math.abs(deltaPos) * SETTLE_MS_PER_POS);
                }
            }
            return;
        }

        lastErrorDeg = Double.NaN;
        lastCell = -1;
        lastTagCount = 0;

        if (now - lastSeenMs > LOST_TIMEOUT_MS) {
            locked = false;
            if (follower != null) {
                turret.update(follower);
            }
        }
    }

    /** Cell 0 / 1 for the active alliance; -1 if none meet {@link #MIN_TAGS}. */
    private int bestCell() {
        int c0 = vision.getVisibleTagCount(0);
        int c1 = vision.getVisibleTagCount(1);
        int best = c1 > c0 ? 1 : 0;
        int count = best == 1 ? c1 : c0;
        lastTagCount = count;
        return (vision.hasValidFrame() && count >= MIN_TAGS) ? best : -1;
    }

    private double averageTx(int cell) {
        int base = (vision.isRedAlliance() ? 30 : 38) + cell * 4;
        double sum = 0;
        int n = 0;
        for (int id = base; id < base + 4; id++) {
            double tx = vision.readTx(id);
            if (Double.isFinite(tx)) {
                sum += tx;
                n++;
            }
        }
        return n == 0 ? Double.NaN : sum / n;
    }

    public boolean isLocked() {
        return locked;
    }

    public boolean isCentered() {
        return locked && Double.isFinite(lastErrorDeg)
                && Math.abs(lastErrorDeg) <= DEADBAND_DEG;
    }

    public double getErrorDeg() {
        return lastErrorDeg;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("----- VISION AIM -----");
        telemetry.addData("Aim Active", active);
        telemetry.addData("Hive Locked", locked);
        telemetry.addData("Centered", isCentered());
        telemetry.addData("Using Cell", lastCell);
        telemetry.addData("Tags In Cell", lastTagCount);
        telemetry.addData("Error (deg)", lastErrorDeg);
        telemetry.addData("Turret Position", turret.getPosition());
    }
}
