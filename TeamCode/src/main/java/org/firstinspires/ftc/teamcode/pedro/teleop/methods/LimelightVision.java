package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/**
 * Limelight AprilTag helper for alliance hive cells (4 tags each).
 * After a tip, only the upright cell is scorable — use {@link #getUprightCell()}.
 * Call {@link #update()} once per loop before reading tags / aiming.
 */
public class LimelightVision {

    private static final int PIPELINE = 0;
    private static final int POLL_RATE = 100;
    private static final long STALE_MS = 100;
    private static final double TX_TOLERANCE = 1.0;
    /** |roll| below this (deg) ⇒ tag reads right-side up (scorable cell). */
    private static final double UPRIGHT_ROLL_DEG = 90.0;

    private static final int[][] RED_CELLS = {
            {30, 31, 32, 33},
            {34, 35, 36, 37}
    };
    private static final int[][] BLUE_CELLS = {
            {38, 39, 40, 41},
            {42, 43, 44, 45}
    };

    private final Limelight3A limelight;

    private boolean redAlliance;
    private boolean validFrame = false;
    private boolean newFrame = false;
    private double lastTimestamp = Double.NaN;
    private LLResult result;
    private final int[] visibleTags = new int[2];
    /** +1 upright, -1 upside-down, 0 unknown — for each cell. */
    private final int[] cellUpright = new int[2];

    public LimelightVision(HardwareMap hardwareMap, boolean redAlliance) {
        this.redAlliance = redAlliance;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(POLL_RATE);
        limelight.pipelineSwitch(PIPELINE);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public void setAlliance(boolean redAlliance) {
        this.redAlliance = redAlliance;
        validFrame = false;
        newFrame = false;
        visibleTags[0] = 0;
        visibleTags[1] = 0;
        cellUpright[0] = 0;
        cellUpright[1] = 0;
    }

    public boolean isRedAlliance() {
        return redAlliance;
    }

    private int[][] cells() {
        return redAlliance ? RED_CELLS : BLUE_CELLS;
    }

    /** Cell index for an alliance AprilTag, or -1 if not ours. */
    private int getCell(int id) {
        int[][] cells = cells();
        for (int i = 0; i < cells.length; i++) {
            for (int tagID : cells[i]) {
                if (id == tagID) {
                    return i;
                }
            }
        }
        return -1;
    }

    public void update() {
        result = limelight.getLatestResult();
        validFrame = false;
        newFrame = false;
        visibleTags[0] = 0;
        visibleTags[1] = 0;
        cellUpright[0] = 0;
        cellUpright[1] = 0;

        if (result == null || !result.isValid() || result.getStaleness() > STALE_MS) {
            return;
        }

        validFrame = true;
        double timestamp = result.getTimestamp();
        newFrame = timestamp != lastTimestamp;
        lastTimestamp = timestamp;

        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            int cell = getCell(tag.getFiducialId());
            if (cell >= 0) {
                visibleTags[cell]++;
            }
        }

        cellUpright[0] = classifyUpright(0);
        cellUpright[1] = classifyUpright(1);
    }

    /**
     * Scorable (upright) cell for this alliance, or -1.
     * After a tip the other cell becomes upright — no fixed side preference.
     */
    public int getUprightCell() {
        if (!validFrame) {
            return -1;
        }
        boolean u0 = cellUpright[0] > 0 && visibleTags[0] > 0;
        boolean u1 = cellUpright[1] > 0 && visibleTags[1] > 0;
        if (u0 && u1) {
            return visibleTags[1] > visibleTags[0] ? 1 : 0;
        }
        if (u0) {
            return 0;
        }
        if (u1) {
            return 1;
        }
        return -1;
    }

    public boolean isCellUpright(int cell) {
        return cell >= 0 && cell < 2 && cellUpright[cell] > 0;
    }

    /**
     * Collect upright-cell tags as (slot 0..3, tx). Returns how many were written
     * into the output arrays (max 4). Used by hive line-fit aim.
     */
    public int collectUprightTagTx(int cell, int[] outSlot, double[] outTx) {
        if (!validFrame || cell < 0 || cell >= 2 || cellUpright[cell] <= 0) {
            return 0;
        }
        if (outSlot == null || outTx == null) {
            return 0;
        }
        int[] ids = cells()[cell];
        int n = 0;
        for (int slot = 0; slot < ids.length; slot++) {
            LLResultTypes.FiducialResult tag = findTag(ids[slot]);
            if (tag == null) {
                continue;
            }
            if (!tagLooksUpright(tag) && visibleTags[cell] >= 2) {
                continue;
            }
            if (n >= outSlot.length || n >= outTx.length) {
                break;
            }
            outSlot[n] = slot;
            outTx[n] = tag.getTargetXDegrees();
            n++;
        }
        return n;
    }

    /**
     * Goal-center bearing (deg): fits slot→tx across the 4-tag row and evaluates
     * at the row midpoint (slot 1.5). Works with ≥2 tags even if one side is blocked.
     * Falls back to a single tag's tx when only one is visible.
     */
    public double uprightGoalCenterTx(int cell) {
        int[] slots = new int[4];
        double[] txs = new double[4];
        int n = collectUprightTagTx(cell, slots, txs);
        if (n == 0) {
            return Double.NaN;
        }
        if (n == 1) {
            return txs[0];
        }
        // Least-squares: tx = a + b * slot. Row center at slot 1.5.
        double sumS = 0, sumT = 0, sumSS = 0, sumST = 0;
        for (int i = 0; i < n; i++) {
            double s = slots[i];
            double t = txs[i];
            sumS += s;
            sumT += t;
            sumSS += s * s;
            sumST += s * t;
        }
        double denom = n * sumSS - sumS * sumS;
        if (Math.abs(denom) < 1e-6) {
            // All same slot (shouldn't happen) — average tx.
            return sumT / n;
        }
        double b = (n * sumST - sumS * sumT) / denom;
        double a = (sumT - b * sumS) / n;
        return a + b * 1.5;
    }

    public double averageUprightTx(int cell) {
        return uprightGoalCenterTx(cell);
    }

    /**
     * Average camera-space range (inches) to visible upright-cell tags, or NaN.
     * Used to scale aim deadband with distance.
     */
    public double uprightRangeInches(int cell) {
        if (!validFrame || cell < 0 || cell >= 2 || cellUpright[cell] <= 0) {
            return Double.NaN;
        }
        int[] ids = cells()[cell];
        double sum = 0;
        int n = 0;
        for (int id : ids) {
            LLResultTypes.FiducialResult tag = findTag(id);
            if (tag == null) {
                continue;
            }
            Pose3D pose = tag.getTargetPoseCameraSpace();
            if (pose == null || pose.getPosition() == null) {
                continue;
            }
            Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
            double inches = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (inches > 2.0) {
                sum += inches;
                n++;
            }
        }
        return n == 0 ? Double.NaN : sum / n;
    }

    /**
     * Classify cell orientation without a reference pose:
     * tag IDs increase left→right when upright; decrease when upside-down.
     * Falls back to pose roll when only one tag is visible.
     *
     * @return +1 upright, -1 upside-down, 0 unknown
     */
    private int classifyUpright(int cell) {
        if (visibleTags[cell] <= 0) {
            return 0;
        }

        int[] ids = cells()[cell];
        // Collect (id, tx) for visible tags.
        int count = 0;
        int[] seenIds = new int[4];
        double[] seenTx = new double[4];
        for (int id : ids) {
            LLResultTypes.FiducialResult tag = findTag(id);
            if (tag == null) {
                continue;
            }
            seenIds[count] = id;
            seenTx[count] = tag.getTargetXDegrees();
            count++;
        }
        if (count == 0) {
            return 0;
        }

        if (count >= 2) {
            // Sort by tx ascending (left → right in image; Limelight +tx = right).
            for (int i = 0; i < count - 1; i++) {
                for (int j = i + 1; j < count; j++) {
                    if (seenTx[j] < seenTx[i]) {
                        double t = seenTx[i];
                        seenTx[i] = seenTx[j];
                        seenTx[j] = t;
                        int id = seenIds[i];
                        seenIds[i] = seenIds[j];
                        seenIds[j] = id;
                    }
                }
            }
            int rising = 0;
            int falling = 0;
            for (int i = 1; i < count; i++) {
                if (seenIds[i] > seenIds[i - 1]) {
                    rising++;
                } else if (seenIds[i] < seenIds[i - 1]) {
                    falling++;
                }
            }
            if (rising > falling) {
                return 1;
            }
            if (falling > rising) {
                return -1;
            }
        }

        // Single tag (or tied order): use camera-space roll if available.
        LLResultTypes.FiducialResult any = findTag(seenIds[0]);
        if (any != null && tagLooksUpright(any)) {
            return 1;
        }
        if (any != null && tagLooksUpsideDown(any)) {
            return -1;
        }
        return 0;
    }

    private boolean tagLooksUpright(LLResultTypes.FiducialResult tag) {
        Double roll = readRollDeg(tag);
        return roll != null && Math.abs(roll) < UPRIGHT_ROLL_DEG;
    }

    private boolean tagLooksUpsideDown(LLResultTypes.FiducialResult tag) {
        Double roll = readRollDeg(tag);
        return roll != null && Math.abs(roll) >= UPRIGHT_ROLL_DEG;
    }

    private Double readRollDeg(LLResultTypes.FiducialResult tag) {
        Pose3D pose = tag.getTargetPoseCameraSpace();
        if (pose == null) {
            return null;
        }
        YawPitchRollAngles orientation = pose.getOrientation();
        if (orientation == null) {
            return null;
        }
        return orientation.getRoll(AngleUnit.DEGREES);
    }

    private LLResultTypes.FiducialResult findTag(int tagID) {
        if (!validFrame || getCell(tagID) < 0) {
            return null;
        }
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            if (tag.getFiducialId() == tagID) {
                return tag;
            }
        }
        return null;
    }

    public double readTx(int tagID) {
        LLResultTypes.FiducialResult tag = findTag(tagID);
        return tag == null ? Double.NaN : tag.getTargetXDegrees();
    }

    public double readTy(int tagID) {
        LLResultTypes.FiducialResult tag = findTag(tagID);
        return tag == null ? Double.NaN : tag.getTargetYDegrees();
    }

    public double readTa(int tagID) {
        LLResultTypes.FiducialResult tag = findTag(tagID);
        return tag == null ? Double.NaN : tag.getTargetArea();
    }

    public boolean hasValidFrame() {
        return validFrame;
    }

    public boolean hasNewFrame() {
        return newFrame;
    }

    public double getFrameTimestamp() {
        return validFrame ? lastTimestamp : Double.NaN;
    }

    public long getStalenessMs() {
        if (!validFrame || result == null) {
            return Long.MAX_VALUE;
        }
        return result.getStaleness();
    }

    /**
     * True if this frame was (approximately) captured at or after {@code wallTimeMs}.
     * Uses {@code now - staleness} so buffered frames from during a turret move are rejected.
     */
    public boolean isFrameCapturedAfter(long wallTimeMs) {
        if (!validFrame || result == null) {
            return false;
        }
        long captureApproxMs = System.currentTimeMillis() - result.getStaleness();
        return captureApproxMs >= wallTimeMs;
    }

    /**
     * How old the current frame is (ms): staleness + capture + targeting latency.
     * Used to look up turret/odometry state at capture time for hybrid aim.
     */
    public long getFrameAgeMs() {
        if (!validFrame || result == null) {
            return Long.MAX_VALUE;
        }
        return result.getStaleness()
                + Math.round(result.getCaptureLatency())
                + Math.round(result.getTargetingLatency());
    }

    /** Approximate wall-clock capture time for the current frame. */
    public long getCaptureWallTimeMs() {
        if (!validFrame || result == null) {
            return 0L;
        }
        return System.currentTimeMillis() - getFrameAgeMs();
    }

    public boolean hasAllianceTags() {
        return validFrame && (visibleTags[0] > 0 || visibleTags[1] > 0);
    }

    public int getVisibleTagCount(int cell) {
        if (cell < 0 || cell >= 2) {
            return 0;
        }
        return visibleTags[cell];
    }

    public boolean isCellFullyVisible(int cell) {
        return getVisibleTagCount(cell) == 4;
    }

    public boolean onTarget(int tagID) {
        double tx = readTx(tagID);
        return Double.isFinite(tx) && Math.abs(tx) <= TX_TOLERANCE;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Vision Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Valid / new", "%s / %s", validFrame, newFrame);
        telemetry.addData("Upright cell", getUprightCell());
        telemetry.addData("Cell0 tags / up", "%d / %s",
                visibleTags[0], uprightLabel(cellUpright[0]));
        telemetry.addData("Cell1 tags / up", "%d / %s",
                visibleTags[1], uprightLabel(cellUpright[1]));

        if (result != null) {
            telemetry.addData("Frame Age (ms)", result.getStaleness());
        }
        if (!validFrame) {
            telemetry.addLine("Vision unavailable");
        }
    }

    private static String uprightLabel(int state) {
        if (state > 0) {
            return "UP";
        }
        if (state < 0) {
            return "DOWN";
        }
        return "?";
    }
}
