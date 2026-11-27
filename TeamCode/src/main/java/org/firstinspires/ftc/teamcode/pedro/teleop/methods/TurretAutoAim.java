package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

/**
 * Hybrid turret aim: odometry target every loop + a slowly learned camera bias.
 *
 * {@code target = odometry + bias + trim}
 * Bias is a 1D Kalman filter updated only on new Limelight frames, with latency
 * compensation via a short history ring buffer (see Hybrid Turret Aim PDF).
 *
 * Schedule {@link #senseCommand()} then {@link #adjustCommand()}.
 * Gate with {@link #setActive}. Requires {@link #setFollower} for hybrid mode;
 * without a follower, falls back to camera-only steps.
 */
@Config
public class TurretAutoAim {

    /** What moves the turret. Estimator still updates in every non-OFF mode when fused. */
    public enum AimMode { OFF, ODOMETRY, CAMERA, HYBRID }

    /** +1 or -1. Flip if the camera fights odometry (large constant innovation). */
    public static double SIGN = -1.0;

    // ---- Camera-only fallback (no Follower) ----
    public static double KP = 0.35;
    public static double MAX_STEP_POS = 0.012;
    public static double MAX_STEP_LARGE = 0.04;
    public static double LARGE_ERROR_DEG = 12.0;
    public static long SETTLE_MS = 60;
    public static double SETTLE_MS_PER_POS = 1800;

    // ---- Shared aim / hold ----
    public static double DEADBAND_DEG = 1.5;
    public static double REACQUIRE_DEG = 4.0;
    public static double DEADBAND_REF_IN = 60.0;
    public static double DEADBAND_MIN_DEG = 1.0;
    public static double DEADBAND_MAX_DEG = 3.5;
    public static double MIN_STEP_POS = 0.004;
    public static long LOST_TIMEOUT_MS = 250;
    public static int MIN_TAGS = 2;

    // ---- Kalman bias (deg / deg²) — Dashboard-tunable ----
    public static double P0_DEG2 = 100.0;
    public static double Q_DEG2_PER_SEC = 0.5;
    public static double P_MAX_DEG2 = 100.0;
    public static double P_RESET_DEG2 = 1.0;
    public static double R_3PLUS = 0.6;
    public static double R_2 = 1.5;
    public static double R_1 = 4.0;
    public static double GATE_SIGMAS = 3.0;
    public static double GATE_MIN_DEG = 4.0;
    public static int REJECT_RESET_FRAMES = 3;

    private static final int HISTORY = 96;

    private final LimelightVision vision;
    private final Turret turret;

    private Follower follower;
    private AimMode mode = AimMode.HYBRID;
    private boolean locked = false;
    private boolean holding = false;
    private double lastErrorDeg = Double.NaN;
    private double lastGoalTx = Double.NaN;
    private double lastRangeIn = Double.NaN;
    private int lastCell = -1;
    private int lastTagCount = 0;
    private long lastSeenMs = 0L;
    private long acceptCaptureAfterMs = 0L;

    /** Learned bias (deg): camera correction on top of odometry. */
    private double biasDeg = 0.0;
    private double pDeg2 = P0_DEG2;
    private double lastK = Double.NaN;
    private double lastInnovationDeg = Double.NaN;
    private double lastOdoPos = Double.NaN;
    private int rejectStreak = 0;
    private int accepted = 0;
    private int rejected = 0;
    private int resets = 0;
    private String status = "OFF";

    private long lastLoopMs = 0L;
    private final long[] histTime = new long[HISTORY];
    private final double[] histTurret = new double[HISTORY];
    private final double[] histOdo = new double[HISTORY];
    private int histHead = 0;
    private int histCount = 0;

    private final int[] fitSlots = new int[4];
    private final double[] fitTx = new double[4];

    // Innovation running stats (accepted updates).
    private double innovMean = 0.0;
    private double innovM2 = 0.0;
    private int innovN = 0;

    public TurretAutoAim(LimelightVision vision, Turret turret) {
        this.vision = vision;
        this.turret = turret;
    }

    /** Pinpoint / Pedro follower — required for hybrid odometry + Kalman bias. */
    public void setFollower(Follower follower) {
        this.follower = follower;
    }

    public void setActive(boolean active) {
        setMode(active ? AimMode.HYBRID : AimMode.OFF);
    }

    public void setMode(AimMode mode) {
        this.mode = mode == null ? AimMode.OFF : mode;
        if (this.mode == AimMode.OFF) {
            holding = false;
            status = "OFF";
        }
    }

    public AimMode getMode() {
        return mode;
    }

    public boolean isActive() {
        return mode != AimMode.OFF;
    }

    /** Clear cell / hold / Kalman state (call on alliance change). */
    public void reset() {
        locked = false;
        holding = false;
        lastErrorDeg = Double.NaN;
        lastGoalTx = Double.NaN;
        lastRangeIn = Double.NaN;
        lastCell = -1;
        lastTagCount = 0;
        lastSeenMs = 0L;
        acceptCaptureAfterMs = 0L;
        biasDeg = 0.0;
        pDeg2 = P0_DEG2;
        lastK = Double.NaN;
        lastInnovationDeg = Double.NaN;
        lastOdoPos = Double.NaN;
        rejectStreak = 0;
        accepted = 0;
        rejected = 0;
        resets = 0;
        histHead = 0;
        histCount = 0;
        lastLoopMs = 0L;
        innovMean = 0.0;
        innovM2 = 0.0;
        innovN = 0;
        status = mode == AimMode.OFF ? "OFF" : "RESET";
    }

    public Command senseCommand() {
        return infinite(vision::update);
    }

    public Command adjustCommand() {
        return infinite(this::adjust);
    }

    /** One adjust cycle for test OpModes that don't use Ivy scheduling. */
    public void tick() {
        adjust();
    }

    private void adjust() {
        if (mode == AimMode.OFF) {
            status = "OFF";
            return;
        }

        if (mode == AimMode.CAMERA || follower == null) {
            adjustCameraOnly();
            return;
        }

        adjustHybrid();
    }

    // -------------------------------------------------------------------------
    // Hybrid: odometry + Kalman bias
    // -------------------------------------------------------------------------

    private void adjustHybrid() {
        long now = System.currentTimeMillis();
        double dtSec = lastLoopMs > 0 ? Math.max(0.0, (now - lastLoopMs) / 1000.0) : 0.0;
        lastLoopMs = now;

        // Predict: uncertainty grows while no camera update.
        pDeg2 = Math.min(pDeg2 + Q_DEG2_PER_SEC * dtSec, P_MAX_DEG2);

        double odoPos = turret.odometryTargetPosition(follower);
        lastOdoPos = odoPos;
        recordHistory(now, turret.getPosition(), odoPos);

        int cell = vision.getUprightCell();
        int tagCount = cell >= 0 ? vision.getVisibleTagCount(cell) : 0;

        if (cell >= 0 && tagCount >= MIN_TAGS) {
            lastSeenMs = now;
            lastCell = cell;
            lastTagCount = tagCount;
            locked = true;

            if (vision.hasNewFrame()) {
                processCameraFrame(cell, tagCount, now);
            } else {
                status = "NO CAMERA";
            }
        } else {
            lastErrorDeg = Double.NaN;
            lastGoalTx = Double.NaN;
            lastRangeIn = Double.NaN;
            lastCell = -1;
            lastTagCount = 0;
            if (now - lastSeenMs > LOST_TIMEOUT_MS) {
                locked = false;
            }
            status = "NO CAMERA";
        }

        // Command: ODOMETRY uses pose only; HYBRID adds learned bias + trim.
        double biasPos = (mode == AimMode.HYBRID)
                ? biasDeg * Constants.TURRET_DEGREE_TO_SERVO
                : 0.0;
        double trim = (mode == AimMode.HYBRID) ? turret.getTrimPos() : 0.0;
        commandToward(odoPos + biasPos + trim);
    }

    private void processCameraFrame(int cell, int tagCount, long now) {
        long captureMs = vision.getCaptureWallTimeMs();
        HistorySample then = lookupHistory(captureMs);
        if (then == null) {
            status = "NO HISTORY";
            return;
        }

        double goalTx = fitCenter(cell);
        if (!Double.isFinite(goalTx)) {
            return;
        }

        lastGoalTx = goalTx;
        lastRangeIn = vision.uprightRangeInches(cell);
        lastErrorDeg = goalTx;

        // Camera's opinion of the correct servo position at capture time.
        double camTarget = then.turretPos
                + SIGN * (-goalTx) * Constants.TURRET_DEGREE_TO_SERVO;
        double predicted = then.odoPos + biasDeg * Constants.TURRET_DEGREE_TO_SERVO;
        double innovationDeg = (camTarget - predicted) / Constants.TURRET_DEGREE_TO_SERVO;
        lastInnovationDeg = innovationDeg;

        double r = measurementNoise(tagCount);
        double gate = Math.max(GATE_MIN_DEG, GATE_SIGMAS * Math.sqrt(pDeg2 + r));

        if (Math.abs(innovationDeg) <= gate) {
            double k = pDeg2 / (pDeg2 + r);
            biasDeg += k * innovationDeg;
            pDeg2 = (1.0 - k) * pDeg2;
            lastK = k;
            rejectStreak = 0;
            accepted++;
            noteInnovation(innovationDeg);
            status = "FUSED";
        } else {
            rejected++;
            rejectStreak++;
            lastK = Double.NaN;
            if (rejectStreak >= REJECT_RESET_FRAMES) {
                // Camera consistent, odometry wrong — snap bias to camera.
                biasDeg = (camTarget - then.odoPos) / Constants.TURRET_DEGREE_TO_SERVO;
                pDeg2 = P_RESET_DEG2;
                rejectStreak = 0;
                resets++;
                status = "RESET BIAS";
            } else {
                status = "REJECT";
            }
        }
    }

    private static double measurementNoise(int tagCount) {
        if (tagCount >= 3) {
            return R_3PLUS;
        }
        if (tagCount == 2) {
            return R_2;
        }
        return R_1;
    }

    private void commandToward(double targetPos) {
        double delta = targetPos - turret.getPosition();
        double absDeg = Math.abs(delta) / Constants.TURRET_DEGREE_TO_SERVO;

        double deadband = scaledDeadband(DEADBAND_DEG);
        double reacquire = scaledDeadband(REACQUIRE_DEG);
        if (reacquire < deadband + 0.5) {
            reacquire = deadband + 0.5;
        }

        if (holding) {
            if (absDeg <= reacquire) {
                return;
            }
            holding = false;
        } else if (absDeg <= deadband) {
            holding = true;
            return;
        }

        if (Math.abs(delta) < MIN_STEP_POS) {
            holding = true;
            return;
        }

        turret.slewTo(targetPos);
    }

    // -------------------------------------------------------------------------
    // Camera-only fallback (no Follower)
    // -------------------------------------------------------------------------

    private void adjustCameraOnly() {
        long now = System.currentTimeMillis();
        status = "CAMERA";

        int cell = vision.getUprightCell();
        int tagCount = cell >= 0 ? vision.getVisibleTagCount(cell) : 0;

        if (cell < 0 || tagCount < MIN_TAGS) {
            lastErrorDeg = Double.NaN;
            lastGoalTx = Double.NaN;
            lastRangeIn = Double.NaN;
            lastCell = -1;
            lastTagCount = 0;
            holding = false;
            if (now - lastSeenMs > LOST_TIMEOUT_MS) {
                locked = false;
            }
            return;
        }

        if (lastCell >= 0 && cell != lastCell) {
            acceptCaptureAfterMs = 0L;
            holding = false;
        }

        lastSeenMs = now;
        lastCell = cell;
        lastTagCount = tagCount;
        locked = true;

        if (!vision.hasNewFrame() || !vision.isFrameCapturedAfter(acceptCaptureAfterMs)) {
            return;
        }

        double goalTx = fitCenter(cell);
        if (!Double.isFinite(goalTx)) {
            return;
        }

        lastGoalTx = goalTx;
        lastRangeIn = vision.uprightRangeInches(cell);
        double error = goalTx;
        lastErrorDeg = error;
        double absError = Math.abs(error);

        double deadband = scaledDeadband(DEADBAND_DEG);
        double reacquire = scaledDeadband(REACQUIRE_DEG);
        if (reacquire < deadband + 0.5) {
            reacquire = deadband + 0.5;
        }

        if (holding) {
            if (absError <= reacquire) {
                return;
            }
            holding = false;
        } else if (absError <= deadband) {
            holding = true;
            return;
        }

        double deltaPos = SIGN * -error * KP * Constants.TURRET_DEGREE_TO_SERVO;
        double maxStep = Math.abs(error) >= LARGE_ERROR_DEG ? MAX_STEP_LARGE : MAX_STEP_POS;
        deltaPos = Range.clip(deltaPos, -maxStep, maxStep);

        if (Math.abs(deltaPos) < MIN_STEP_POS) {
            holding = true;
            return;
        }

        turret.setPosition(turret.getPosition() + deltaPos);
        long settleMs = SETTLE_MS + (long) (Math.abs(deltaPos) * SETTLE_MS_PER_POS);
        acceptCaptureAfterMs = now + settleMs;
    }

    // -------------------------------------------------------------------------
    // History / fit / telemetry
    // -------------------------------------------------------------------------

    private void recordHistory(long timeMs, double turretPos, double odoPos) {
        histTime[histHead] = timeMs;
        histTurret[histHead] = turretPos;
        histOdo[histHead] = odoPos;
        histHead = (histHead + 1) % HISTORY;
        if (histCount < HISTORY) {
            histCount++;
        }
    }

    private HistorySample lookupHistory(long captureMs) {
        if (histCount == 0) {
            return null;
        }
        int best = -1;
        long bestDt = Long.MAX_VALUE;
        for (int i = 0; i < histCount; i++) {
            int idx = (histHead - 1 - i + HISTORY) % HISTORY;
            long dt = Math.abs(histTime[idx] - captureMs);
            if (dt < bestDt) {
                bestDt = dt;
                best = idx;
            }
            // Samples are newest→oldest; stop once we've gone past capture.
            if (histTime[idx] < captureMs - 5) {
                break;
            }
        }
        if (best < 0) {
            return null;
        }
        return new HistorySample(histTurret[best], histOdo[best]);
    }

    private double scaledDeadband(double baseDeg) {
        if (!Double.isFinite(lastRangeIn) || DEADBAND_REF_IN <= 1.0) {
            return baseDeg;
        }
        double scaled = baseDeg * (lastRangeIn / DEADBAND_REF_IN);
        return Range.clip(scaled, DEADBAND_MIN_DEG, DEADBAND_MAX_DEG);
    }

    private double fitCenter(int cell) {
        int n = vision.collectUprightTagTx(cell, fitSlots, fitTx);
        if (n < MIN_TAGS) {
            return Double.NaN;
        }

        double sumS = 0, sumT = 0, sumSS = 0, sumST = 0;
        for (int i = 0; i < n; i++) {
            double s = fitSlots[i];
            double t = fitTx[i];
            sumS += s;
            sumT += t;
            sumSS += s * s;
            sumST += s * t;
        }
        double denom = n * sumSS - sumS * sumS;
        if (Math.abs(denom) < 1e-6) {
            return sumT / n;
        }
        double b = (n * sumST - sumS * sumT) / denom;
        double a = (sumT - b * sumS) / n;
        return a + b * 1.5;
    }

    private void noteInnovation(double innov) {
        innovN++;
        double delta = innov - innovMean;
        innovMean += delta / innovN;
        innovM2 += delta * (innov - innovMean);
    }

    public boolean isLocked() {
        return locked;
    }

    public boolean isCentered() {
        return locked && holding;
    }

    public boolean isHolding() {
        return holding;
    }

    public double getErrorDeg() {
        return lastErrorDeg;
    }

    public double getBiasDeg() {
        return biasDeg;
    }

    public void addTelemetry(Telemetry telemetry) {
        String state;
        if (mode == AimMode.OFF) {
            state = "OFF";
        } else if (holding) {
            state = "HOLD";
        } else if (locked) {
            state = "SEEK";
        } else {
            state = "LOST";
        }
        telemetry.addData("AutoAim", "%s | %s | %s", mode, state, status);
        telemetry.addData("Cell / tags", "%d / %d", lastCell, lastTagCount);
        telemetry.addData("Goal tx / err", "%.2f / %.2f", lastGoalTx, lastErrorDeg);
        telemetry.addData("Range in", lastRangeIn);
        if (follower != null && mode != AimMode.CAMERA) {
            telemetry.addData("Odo / bias", "%.3f / %.2f°", lastOdoPos, biasDeg);
            telemetry.addData("Innov / K / P", "%.2f° / %.2f / %.1f",
                    lastInnovationDeg, lastK, pDeg2);
            telemetry.addData("Acc / rej / reset", "%d / %d / %d", accepted, rejected, resets);
            double innovStd = innovN > 1 ? Math.sqrt(innovM2 / (innovN - 1)) : 0.0;
            telemetry.addData("Innov mean/std", "%.2f / %.2f (n=%d)", innovMean, innovStd, innovN);
        }
    }

    private static final class HistorySample {
        final double turretPos;
        final double odoPos;

        HistorySample(double turretPos, double odoPos) {
            this.turretPos = turretPos;
            this.odoPos = odoPos;
        }
    }
}
