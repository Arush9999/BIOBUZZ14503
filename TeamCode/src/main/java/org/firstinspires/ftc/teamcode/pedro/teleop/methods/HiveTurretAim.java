package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

/**
 * Aims the turret at the CENTER OF THE HIVE using whichever tag cell the camera sees best.
 *
 * The hive has two cells of 4 tags each (see LimelightVision: cell 0 = first group,
 * cell 1 = second group).
 *
 * Cluster geometry (FTC manual Fig 9-15): the 4 tag centerlines sit at -6.5, -2.75, +2.75,
 * +6.5 inches from the cluster center. Each visible tag gives a point (x inches, tx degrees);
 * a line fit through them gives the cluster center's tx AND the scale (deg per inch), so the
 * center is found correctly even when some tags are blocked. With one tag visible, the last
 * good scale is used.
 *
 * The hive center may not be the cluster center, so each cell has an OFFSET in inches from the
 * cluster center to the hive center:
 *
 *     error = clusterCenterTx + degPerInch * CELLn_HIVE_OFFSET_IN - TX_OFFSET_DEG
 *
 * CELLn_HIVE_OFFSET_IN > 0 means the hive center is to the RIGHT of that cluster's center as
 * seen by the camera (turret must turn CW to reach it); < 0 means to the LEFT. 0 = aim at the
 * cluster center.
 *
 * Best cell = most visible tags (ties broken by total tag area). The choice is sticky: the other
 * cell must stay better for CELL_SWITCH_FRAMES frames before the aim switches, so the turret
 * does not jump back and forth between offsets.
 *
 * Motion is smooth and still:
 *  1. A settled camera frame produces a servo SETPOINT.
 *  2. The turret eases toward it with a velocity limit.
 *  3. After arriving it waits SETTLE_MS before trusting a frame again (camera latency).
 *  4. HARD STOP: inside DEADBAND_DEG the turret is frozen and receives no servo commands
 *     until |error| exceeds RELEASE_DEG.
 *
 * Sign convention (from Turret): position HIGH->LOW = CW, LOW->HIGH = CCW.
 * Limelight tx > 0 means target is RIGHT of the crosshair => rotate CW => position decreases.
 *
 * If the hive is not seen for LOST_TIMEOUT_MS it falls back to odometry aim (when a Follower
 * is provided), otherwise it holds position.
 *
 * Call vision.update() BEFORE calling update() here each loop.
 * Same public API as VisionTurretAim, so it is a drop-in swap.
 */
@Config
public class HiveTurretAim {

    // ---------------- HIVE GEOMETRY (FTC Dashboard) ----------------
    /** Inches from cell 0's cluster center to the hive center. + = hive center is to the RIGHT. */
    public static double CELL0_HIVE_OFFSET_IN = 0.0;
    /** Inches from cell 1's cluster center to the hive center. + = hive center is to the RIGHT. */
    public static double CELL1_HIVE_OFFSET_IN = 0.0;

    /** Outer tag centerline distance from cluster center (inches, manual Fig 9-15). */
    public static double TAG_OUTER_IN = 6.5;
    /** Inner tag centerline distance from cluster center (inches, manual Fig 9-15). */
    public static double TAG_INNER_IN = 2.75;
    /** Set true if tag IDs ascend right-to-left as the camera sees the cluster. */
    public static boolean REVERSE_TAG_ORDER = false;
    /** Starting/fallback scale (degrees of tx per inch) until two tags have been seen. */
    public static double K_FALLBACK_DEG_PER_IN = 1.0;
    /** Minimum visible tags in a cell to use it. */
    public static int MIN_TAGS = 2;
    /** Frames the other cell must stay better before the aim switches to it. */
    public static int CELL_SWITCH_FRAMES = 5;

    // ---------------- CONTROL TUNABLES (FTC Dashboard) ----------------
    /** +1 or -1. Flip if turret turns away from the hive. */
    public static double SIGN = -1.0;
    /** Extra global aim trim in degrees (shifts where the shot "centers"). */
    public static double TX_OFFSET_DEG = 0.0;

    /** Fraction of a large error corrected per move. */
    public static double KP = 0.8;
    /** Fraction of a small error corrected per move (gentler, avoids overshoot). */
    public static double KP_NEAR = 0.5;
    /** Use KP_NEAR when |error| is within this (deg). */
    public static double NEAR_DEG = 6.0;

    /** STOP completely when |error| is at or below this (deg). */
    public static double DEADBAND_DEG = 1.5;
    /** After stopping, resume only if |error| exceeds this (deg). Keep > DEADBAND_DEG. */
    public static double RELEASE_DEG = 2.5;
    /** Small errors must persist this many frames (averaged) before the turret moves. */
    public static int CONFIRM_FRAMES = 2;

    /** Max servo-position change requested per correction. */
    public static double MAX_STEP_POS = 0.04;
    /** Max turret speed in servo-position units per second. Lower = smoother/slower. */
    public static double MAX_VEL = 0.8;
    /** Easing rate (1/s). Higher = snappier approach, lower = softer. */
    public static double EASE_RATE = 10.0;
    /** Slowest allowed speed (pos/s) so the turret always finishes its move. */
    public static double MIN_VEL = 0.12;
    /** Within this of the setpoint counts as arrived (servo-position units). */
    public static double ARRIVE_EPS = 0.0005;

    /** Wait after the turret ARRIVES before trusting a camera frame (camera latency, ms). */
    public static long SETTLE_MS = 60;
    /** How long after losing the hive before falling back to odometry. */
    public static long LOST_TIMEOUT_MS = 250;

    private final LimelightVision vision;
    private final Turret turret;

    private boolean locked = false;
    private boolean tracking = false;
    private boolean holding = false;
    private boolean moving = false;
    private double setpoint = 0.5;
    private long arrivedMs = 0;

    private int currentCell = -1;
    private int switchVotes = 0;
    private int lastTagCount = 0;
    private double lastCellTx = Double.NaN;
    private double lastK = Double.NaN;
    private double usedK = 1.0;
    private double lastErrorDeg = Double.NaN;
    private long lastSeenMs = 0;
    private long lastNs = 0;

    private double errSum = 0.0;
    private int errCount = 0;

    public HiveTurretAim(LimelightVision vision, Turret turret) {
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
        long nowNs = System.nanoTime();
        double dt = lastNs == 0L ? 0.02 : Range.clip((nowNs - lastNs) / 1.0e9, 0.001, 0.1);
        lastNs = nowNs;

        int cell = chooseCell();
        double cellTx = cell >= 0 ? fitCenter(cell) : Double.NaN;

        if (cell >= 0 && Double.isFinite(cellTx)) {
            lastSeenMs = now;
            lastCellTx = cellTx;
            locked = true;

            if (!tracking) {
                tracking = true;
                setpoint = turret.getPosition();
                moving = false;
                holding = false;
                arrivedMs = now;
                resetFilter();
            }

            stepTurret(dt, now);

            // Judge the error only when stationary, settled, and the frame is new.
            if (!moving && vision.hasNewFrame() && now - arrivedMs >= SETTLE_MS) {
                double hiveError = cellTx + usedK * cellOffset(cell) - TX_OFFSET_DEG;
                processFrame(hiveError);
            }
            return;
        }

        // Hive not visible: finish any move in progress, then hold.
        lastErrorDeg = Double.NaN;
        lastCellTx = Double.NaN;
        lastTagCount = 0;
        resetFilter();
        stepTurret(dt, now);

        if (now - lastSeenMs > LOST_TIMEOUT_MS) {
            locked = false;
            tracking = false;
            moving = false;
            holding = false;
            currentCell = -1;
            switchVotes = 0;
            if (follower != null) {
                turret.aimAtGoal(follower);
            }
        }
    }

    /** Decide whether to start a new move based on the latest settled frame. */
    private void processFrame(double error) {
        lastErrorDeg = error;
        double absErr = Math.abs(error);

        // HARD STOP: while holding, ignore everything inside the release band.
        if (holding) {
            if (absErr < RELEASE_DEG) {
                resetFilter();
                return;
            }
            holding = false;
        }

        // Inside the deadband: freeze. No servo commands are sent from here on.
        if (absErr <= DEADBAND_DEG) {
            holding = true;
            resetFilter();
            return;
        }

        // Large errors act immediately; small errors are confirmed over several frames.
        errSum += error;
        errCount++;
        int needed = absErr > NEAR_DEG ? 1 : Math.max(1, CONFIRM_FRAMES);
        if (errCount < needed) {
            return;
        }

        double useError = errSum / errCount;
        resetFilter();

        if (Math.abs(useError) <= DEADBAND_DEG) {
            holding = true;
            return;
        }

        double gain = Math.abs(useError) > NEAR_DEG ? KP : KP_NEAR;
        double deltaPos = SIGN * -useError * gain * Constants.TURRET_DEGREE_TO_SERVO;
        deltaPos = Range.clip(deltaPos, -MAX_STEP_POS, MAX_STEP_POS);

        double pos = turret.getPosition();
        setpoint = Range.clip(pos + deltaPos, Constants.TURRET_MIN, Constants.TURRET_MAX);

        if (Math.abs(setpoint - pos) <= ARRIVE_EPS) {
            setpoint = pos; // already at a travel limit
            holding = true;
        } else {
            moving = true;
        }
    }

    /** Ease the turret toward the setpoint with a velocity limit. Sends nothing when idle. */
    private void stepTurret(double dt, long now) {
        if (!moving) {
            return;
        }

        double pos = turret.getPosition();
        double delta = setpoint - pos;

        if (Math.abs(delta) <= ARRIVE_EPS) {
            if (delta != 0.0) {
                turret.setPosition(setpoint);
            }
            moving = false;
            arrivedMs = now;
            return;
        }

        double step = delta * EASE_RATE * dt;
        double maxStep = MAX_VEL * dt;
        double minStep = MIN_VEL * dt;
        step = Range.clip(step, -maxStep, maxStep);
        if (Math.abs(step) < minStep) {
            step = Math.signum(delta) * Math.min(Math.abs(delta), minStep);
        }
        turret.setPosition(pos + step);
    }

    private void resetFilter() {
        errSum = 0.0;
        errCount = 0;
    }

    // ---------------- cell selection ----------------

    /**
     * Picks the best cell (most visible tags, ties by tag area) with sticky switching.
     * Returns -1 when no cell has at least MIN_TAGS visible tags.
     */
    private int chooseCell() {
        if (!vision.hasValidFrame()) {
            lastTagCount = 0;
            return -1;
        }

        int c0 = vision.getVisibleTagCount(0);
        int c1 = vision.getVisibleTagCount(1);

        boolean ok0 = c0 >= MIN_TAGS;
        boolean ok1 = c1 >= MIN_TAGS;
        if (!ok0 && !ok1) {
            lastTagCount = 0;
            return -1;
        }

        int candidate;
        if (ok0 && ok1) {
            if (c0 != c1) {
                candidate = c0 > c1 ? 0 : 1;
            } else {
                candidate = totalArea(0) >= totalArea(1) ? 0 : 1;
            }
        } else {
            candidate = ok0 ? 0 : 1;
        }

        boolean currentUsable = (currentCell == 0 && ok0) || (currentCell == 1 && ok1);

        if (!currentUsable) {
            // First pick, or the current cell disappeared: switch immediately.
            currentCell = candidate;
            switchVotes = 0;
        } else if (candidate != currentCell) {
            // Only switch when the other cell has strictly MORE tags, for several frames.
            int candCount = candidate == 0 ? c0 : c1;
            int curCount = currentCell == 0 ? c0 : c1;
            if (candCount > curCount) {
                if (vision.hasNewFrame()) {
                    switchVotes++;
                }
                if (switchVotes >= CELL_SWITCH_FRAMES) {
                    currentCell = candidate;
                    switchVotes = 0;
                    holding = false;
                    resetFilter();
                }
            } else {
                switchVotes = 0;
            }
        } else {
            switchVotes = 0;
        }

        lastTagCount = currentCell == 0 ? c0 : c1;
        return currentCell;
    }

    private double cellOffset(int cell) {
        return cell == 0 ? CELL0_HIVE_OFFSET_IN : CELL1_HIVE_OFFSET_IN;
    }

    private int cellBase(int cell) {
        return (vision.isRedAlliance() ? 30 : 38) + cell * 4;
    }

    /** Lateral position (inches from cluster center, + = right) of tag index 0..3 in a cell. */
    private static double tagX(int index) {
        double x;
        switch (index) {
            case 0:  x = -TAG_OUTER_IN; break;
            case 1:  x = -TAG_INNER_IN; break;
            case 2:  x = TAG_INNER_IN;  break;
            default: x = TAG_OUTER_IN;  break;
        }
        return REVERSE_TAG_ORDER ? -x : x;
    }

    /**
     * Fits tx = center + k * x through the visible tags of a cell using their known positions.
     * Returns the cluster center's tx and stores the scale k (deg/inch) in usedK.
     * One tag: uses the last good k. Two or more: least-squares fit.
     */
    private double fitCenter(int cell) {
        int base = cellBase(cell);
        double sx = 0, st = 0;
        int n = 0;
        double[] xs = new double[4];
        double[] ts = new double[4];
        for (int i = 0; i < 4; i++) {
            double tx = vision.readTx(base + i);
            if (Double.isFinite(tx)) {
                xs[n] = tagX(i);
                ts[n] = tx;
                sx += xs[n];
                st += tx;
                n++;
            }
        }
        if (n == 0) {
            return Double.NaN;
        }

        double meanX = sx / n;
        double meanT = st / n;
        double k = Double.isFinite(lastK) ? lastK : K_FALLBACK_DEG_PER_IN;

        if (n >= 2) {
            double sxx = 0, sxt = 0;
            for (int i = 0; i < n; i++) {
                sxx += (xs[i] - meanX) * (xs[i] - meanX);
                sxt += (xs[i] - meanX) * (ts[i] - meanT);
            }
            // Need real spread between tags and a sane (positive) scale to trust the fit.
            if (sxx > 1.0) {
                double fitK = sxt / sxx;
                if (fitK > 0.05 && fitK < 10.0) {
                    k = fitK;
                    lastK = Double.isFinite(lastK) ? 0.7 * lastK + 0.3 * fitK : fitK;
                }
            }
        }

        usedK = k;
        return meanT - k * meanX;
    }

    private double totalArea(int cell) {
        int base = cellBase(cell);
        double sum = 0;
        for (int id = base; id < base + 4; id++) {
            double ta = vision.readTa(id);
            if (Double.isFinite(ta)) {
                sum += ta;
            }
        }
        return sum;
    }

    // ---------------- status ----------------

    public boolean isLocked() {
        return locked;
    }

    /** True when the turret has stopped inside the deadband. */
    public boolean isCentered() {
        return locked && holding;
    }

    public double getErrorDeg() {
        return lastErrorDeg;
    }

    public int getChosenCell() {
        return currentCell;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("----- HIVE AIM -----");
        telemetry.addData("Hive Locked", locked);
        telemetry.addData("Holding (stopped)", holding);
        telemetry.addData("Moving", moving);
        telemetry.addData("Chosen Cell", currentCell);
        telemetry.addData("Tags In Cell", lastTagCount);
        telemetry.addData("Cell Center tx", lastCellTx);
        telemetry.addData("Cell->Hive Offset (in)",
                currentCell >= 0 ? cellOffset(currentCell) : Double.NaN);
        telemetry.addData("Scale (deg/in)", usedK);
        telemetry.addData("Hive Error (deg)", lastErrorDeg);
        telemetry.addData("Turret Position", turret.getPosition());
        telemetry.addData("Setpoint", setpoint);
    }
}
