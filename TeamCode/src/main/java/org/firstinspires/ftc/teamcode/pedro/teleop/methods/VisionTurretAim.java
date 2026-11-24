package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

/**
 * Closes the loop between LimelightVision and Turret.
 *
 * Every NEW Limelight frame, the horizontal error (tx) of the alliance hive AprilTags
 * is converted to a servo-position correction and applied to the turret.
 *
 * Sign convention (from Turret): position HIGH->LOW = CW, LOW->HIGH = CCW.
 * Limelight tx > 0 means the target is to the RIGHT of the crosshair, so the turret
 * must rotate CW => servo position decreases. Flip SIGN if your camera is mounted
 * mirrored / the turret runs away from the target.
 *
 * If the hive is not seen for LOST_TIMEOUT_MS, it falls back to odometry aim
 * (when a Follower is provided), otherwise it holds the last position.
 *
 * Call vision.update() BEFORE calling update() here each loop.
 */
@Config
public class VisionTurretAim {

    // ---------------- TUNABLES (FTC Dashboard) ----------------
    /** +1 or -1. Flip if turret turns away from the hive. */
    public static double SIGN = -1.0;
    /** Fraction of the measured error corrected per new frame (0..1). */
    public static double KP = 0.6;
    /** Aim offset in degrees (shift where the shot "centers" in the camera view). */
    public static double TX_OFFSET_DEG = 0.0;
    /** Do not move when |error| is below this (deg). */
    public static double DEADBAND_DEG = 0.5;
    /** Max servo-position change per loop. */
    public static double MAX_STEP_POS = 0.02;
    /** How long after losing the hive before falling back to odometry. */
    public static long LOST_TIMEOUT_MS = 300;
    /** Minimum visible tags in the chosen cell to trust it. */
    public static int MIN_TAGS = 1;

    private final LimelightVision vision;
    private final Turret turret;

    private boolean locked = false;
    private double lastErrorDeg = Double.NaN;
    private int lastCell = -1;
    private int lastTagCount = 0;
    private long lastSeenMs = 0;

    public VisionTurretAim(LimelightVision vision, Turret turret) {
        this.vision = vision;
        this.turret = turret;
    }

    /** Vision-only aiming: holds position when the hive is not visible. */
    public void update() {
        update(null);
    }

    /** Vision aiming with odometry fallback when the hive is lost. */
    public void update(Follower follower) {
        long now = System.currentTimeMillis();

        // Pick the hive cell (group of 4 tags) with the most visible tags.
        int cell = bestCell();
        double avgTx = cell >= 0 ? averageTx(cell) : Double.NaN;

        if (cell >= 0 && Double.isFinite(avgTx)) {
            lastSeenMs = now;
            lastCell = cell;
            locked = true;

            // Only act on fresh data so stale frames are not applied repeatedly.
            if (vision.hasNewFrame()) {
                double error = avgTx - TX_OFFSET_DEG;
                lastErrorDeg = error;

                if (Math.abs(error) > DEADBAND_DEG) {
                    double deltaPos = SIGN * -error * KP * Constants.TURRET_DEGREE_TO_SERVO;
                    deltaPos = Range.clip(deltaPos, -MAX_STEP_POS, MAX_STEP_POS);
                    turret.setPosition(turret.getPosition() + deltaPos);
                }
            }
            return;
        }

        // Hive not visible.
        lastErrorDeg = Double.NaN;
        lastCell = -1;
        lastTagCount = 0;

        if (now - lastSeenMs > LOST_TIMEOUT_MS) {
            locked = false;
            if (follower != null) {
                turret.aimAtGoal(follower);
            }
        }
    }

    // ---------------- helpers ----------------

    /** Cell 0 = first tag group, cell 1 = second, for the active alliance. -1 if none. */
    private int bestCell() {
        int c0 = vision.getVisibleTagCount(0);
        int c1 = vision.getVisibleTagCount(1);
        int best = c1 > c0 ? 1 : 0;
        int count = best == 1 ? c1 : c0;
        lastTagCount = count;
        return (vision.hasValidFrame() && count >= MIN_TAGS) ? best : -1;
    }

    /** Average tx of the visible tags in a cell (readTx filters out the other alliance). */
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

    // ---------------- status ----------------

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
        telemetry.addData("Hive Locked", locked);
        telemetry.addData("Centered", isCentered());
        telemetry.addData("Using Cell", lastCell);
        telemetry.addData("Tags In Cell", lastTagCount);
        telemetry.addData("Error (deg)", lastErrorDeg);
        telemetry.addData("Turret Position", turret.getPosition());
    }
}
