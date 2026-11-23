package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.pedro.teleop.constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Flywheel;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Measures the flywheel and works out the FlywheelConstants gains by itself. No graphs.
 *
 * Run with NO balls and hands clear. Takes about two minutes:
 * 1) Feedforward: applies five fixed voltages, waits for the speed to settle, fits KS and KV.
 * 2) KP: starts from a formula, holds speed, halves KP if the motor power jitters.
 * 3) KI: KP / (how long the wheel takes to respond).
 * 4) Recovery band: steps the speed up and keeps the smallest band that does not overshoot.
 * 5) Hold check: holds the target and sets READY_TOL and SHOT_DROP from the measured noise.
 *
 * The results are live until the robot restarts. Type them into FlywheelConstants.java to keep them.
 */
@Config
@TeleOp(name = "Flywheel_Auto_Tune", group = "Testing")
public class Flywheel_Auto_Tune extends LinearOpMode {

    // Feedforward sweep. 6.0 to 10.5 V covers about 1050 to 1900 ticks/s, the speeds used for
    // shooting, so the fit is not pulled off by slow speeds the wheel never shoots at.
    public static double SWEEP_MIN_VOLTS = 6.0;
    public static double SWEEP_MAX_VOLTS = 10.5;
    public static int SWEEP_POINTS = 5;
    public static double SETTLE_MIN_SEC = 1.0;
    public static double SETTLE_WINDOW_SEC = 0.5;
    // Settled when two back-to-back window averages differ by less than this fraction.
    public static double SETTLE_FRACTION = 0.0015;
    public static double SETTLE_TIMEOUT_SEC = 12.0;
    public static double MEASURE_SEC = 1.0;

    // Feedback
    // KP starting rule: this many extra volts for a 2% speed error.
    public static double KP_VOLTS_AT_2_PERCENT = 0.75;
    public static double MAX_POWER_STD = 0.02;
    public static double DEFAULT_RESPONSE_SEC = 1.5;

    // Recovery band
    public static double OVERSHOOT_LIMIT = 0.01;
    public static double STEP_FROM_FRACTION = 0.75;

    // READY_TOL never goes above this. It must stay below the smallest recovery band (0.03),
    // or the wheel would count as ready the moment full-power recovery ends.
    public static double MAX_READY_TOL = 0.025;

    // Pass/fail limits
    public static double MAX_FIT_ERROR_PERCENT = 3.0;
    public static double MAX_HOLD_ERROR_PERCENT = 0.5;
    public static double LOW_BATTERY_VOLTS = 12.0;
    public static double MAX_HOLD_POWER_AT_LOW_BATTERY = 0.90;

    // Pause per loop so the tuner runs at roughly the same loop time as TeleOp.
    public static int LOOP_SLEEP_MS = 8;

    private static final double[] BANDS = {0.03, 0.04, 0.06, 0.08, 0.10};
    private static final int MAX_SAMPLES = 4000;

    private Flywheel flywheel;
    private final List<String> warnings = new ArrayList<>();
    private String failure = null;

    // Values in place before the run, restored if the run does not finish.
    private double oldKs, oldKv, oldKp, oldKi, oldBand, oldReadyTol, oldShotDrop;

    // Results
    private double startVolts, endVolts;
    private double fitErrorPercent, responseSec;
    private double holdErrorPercent, noisePercent, smoothNoisePercent, speedStep, powerStd;
    private double overshootPercent, spinUpSec, holdPower, holdPowerLowBattery;

    private static class HoldResult {
        double meanSpeed;
        double meanPower;
        double powerStd;
        double errorFraction;      // (target - mean) / target
        double rawPeakFraction;    // worst single reading vs target
        double smoothPeakFraction; // worst 3-reading average vs the mean
        double readyNoiseFraction; // worst smoothed speed (what the ready check sees) vs target
        double speedStep;          // smallest change between readings
    }

    private static class StepResult {
        double overshootFraction;
        double readySec;
        boolean reached;
    }

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        flywheel = new Flywheel(hardwareMap);
        saveOriginals();

        telemetry.addLine("FLYWHEEL AUTO TUNE");
        telemetry.addLine("Remove ALL balls. Keep hands clear of the shooter.");
        telemetry.addLine("The wheel spins by itself for about 2 minutes.");
        telemetry.addData("Target", "%.0f ticks/s", FlywheelConstants.TARGET_VELOCITY);
        telemetry.addData("Battery", "%.2f V", flywheel.getBatteryVoltage());
        telemetry.addLine("Press START to begin. STOP at any time to cancel.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        boolean finished = false;
        try {
            finished = runTune();
        } finally {
            flywheel.stop();
            if (!finished) {
                restoreOriginals();
            }
        }

        if (finished) {
            logResults();
        }
        while (opModeIsActive()) {
            if (finished) {
                showResults();
            } else {
                telemetry.addLine("FLYWHEEL AUTO TUNE - FAILED");
                telemetry.addLine(failure != null ? failure : "Stopped before finishing.");
                telemetry.addLine("FlywheelConstants were put back to what they were.");
                telemetry.update();
            }
            sleep(50);
        }
    }

    // ---------------- The tune ----------------

    private boolean runTune() {
        double target = FlywheelConstants.TARGET_VELOCITY;
        if (target <= 0) {
            failure = "TARGET_VELOCITY must be above 0.";
            return false;
        }
        startVolts = flywheel.getBatteryVoltage();

        // 1) Feedforward
        if (!runSweep()) {
            return false;
        }

        // 2) KP
        FlywheelConstants.KP = KP_VOLTS_AT_2_PERCENT / (0.02 * target);
        FlywheelConstants.KI = 0.0;
        FlywheelConstants.RECOVER_BAND = 0.04;
        HoldResult hold = null;
        for (int attempt = 0; attempt < 4; attempt++) {
            hold = holdAndMeasure("2/5 KP check " + (attempt + 1), target, 3.0);
            if (hold == null) {
                return false;
            }
            if (hold.powerStd <= MAX_POWER_STD) {
                break;
            }
            if (attempt < 3) {
                FlywheelConstants.KP *= 0.5;
            } else {
                warnings.add("Motor power still jitters at the lowest KP tried.");
            }
        }
        double smoothNoise = hold.smoothPeakFraction;

        // 3) KI
        FlywheelConstants.KI = FlywheelConstants.KP / responseSec;

        // 4) Recovery band
        StepResult step = null;
        double allowed = OVERSHOOT_LIMIT + smoothNoise;
        for (int i = 0; i < BANDS.length; i++) {
            step = stepTest(BANDS[i], target, i + 1);
            if (step == null) {
                return false;
            }
            if (!step.reached) {
                failure = "Wheel could not reach the target. Battery too low or target too high.";
                return false;
            }
            if (step.overshootFraction <= allowed) {
                break;
            }
            if (i == BANDS.length - 1) {
                warnings.add("Overshoot is above the limit even with the widest recovery band.");
            }
        }
        overshootPercent = step.overshootFraction * 100.0;
        spinUpSec = step.readySec;

        // 5) Hold check with the finished gains
        HoldResult fin = holdAndMeasure("5/5 Hold check", target, 5.0);
        if (fin == null) {
            return false;
        }
        holdErrorPercent = fin.errorFraction * 100.0;
        noisePercent = fin.rawPeakFraction * 100.0;
        speedStep = fin.speedStep;
        powerStd = fin.powerStd;
        holdPower = fin.meanPower;
        holdPowerLowBattery =
                (FlywheelConstants.KS + FlywheelConstants.KV * target) / LOW_BATTERY_VOLTS;

        smoothNoisePercent = fin.readyNoiseFraction * 100.0;

        // The ready check and the shot log both use the smoothed speed, so size them from that.
        FlywheelConstants.READY_TOL =
                Range.clip(1.5 * fin.readyNoiseFraction + 0.005, 0.015, MAX_READY_TOL);
        FlywheelConstants.SHOT_DROP = Math.max(0.03, 2.5 * fin.readyNoiseFraction);
        if (fin.readyNoiseFraction > MAX_READY_TOL) {
            warnings.add("Speed reading is very noisy. The ready check may flicker.");
        }

        if (fitErrorPercent > MAX_FIT_ERROR_PERCENT) {
            warnings.add("Feedforward fit is rough. Run the tune again.");
        }
        if (Math.abs(holdErrorPercent) > MAX_HOLD_ERROR_PERCENT) {
            warnings.add("Wheel settles off target. Run the tune again.");
        }
        if (holdPowerLowBattery > MAX_HOLD_POWER_AT_LOW_BATTERY) {
            warnings.add("Target needs too much power on a low battery. Lower the target.");
        }
        endVolts = flywheel.getBatteryVoltage();
        return true;
    }

    /** Stage 1: fixed voltages -> settled speeds -> straight-line fit for KS and KV. */
    private boolean runSweep() {
        int points = Math.max(3, SWEEP_POINTS);
        double[] volts = new double[points];
        double[] speeds = new double[points];
        double[] sampleTime = new double[MAX_SAMPLES];
        double[] sampleSpeed = new double[MAX_SAMPLES];
        double responseSum = 0.0;
        int responseCount = 0;

        for (int i = 0; i < points; i++) {
            double commanded = SWEEP_MIN_VOLTS + (SWEEP_MAX_VOLTS - SWEEP_MIN_VOLTS) * i / (points - 1);
            String stage = String.format(Locale.US, "1/5 Feedforward %d of %d (%.1f V)", i + 1, points, commanded);

            double startSpeed = flywheel.getVelocity();
            flywheel.runOpenLoopVolts(commanded);

            ElapsedTime timer = new ElapsedTime();
            int count = 0;
            double windowSum = 0.0;
            int windowN = 0;
            double windowStart = 0.0;
            double previousMean = Double.NaN;
            while (step()) {
                double now = timer.seconds();
                double speed = flywheel.getVelocity();
                if (count < MAX_SAMPLES) {
                    sampleTime[count] = now;
                    sampleSpeed[count] = speed;
                    count++;
                }
                windowSum += speed;
                windowN++;
                if (now - windowStart >= SETTLE_WINDOW_SEC) {
                    double mean = windowSum / windowN;
                    boolean steady = !Double.isNaN(previousMean)
                            && Math.abs(mean - previousMean) <= SETTLE_FRACTION * Math.max(Math.abs(mean), 100.0);
                    if (now >= SETTLE_MIN_SEC && steady) {
                        break;
                    }
                    previousMean = mean;
                    windowSum = 0.0;
                    windowN = 0;
                    windowStart = now;
                }
                if (now > SETTLE_TIMEOUT_SEC) {
                    break;
                }
                status(stage, "waiting for speed to settle");
            }
            if (!opModeIsActive()) {
                return false;
            }

            // Average speed and applied volts once settled.
            double speedSum = 0.0;
            double voltSum = 0.0;
            int n = 0;
            timer.reset();
            while (step() && timer.seconds() < MEASURE_SEC) {
                speedSum += flywheel.getVelocity();
                voltSum += flywheel.getAppliedVolts();
                n++;
                status(stage, "measuring");
            }
            if (!opModeIsActive() || n == 0) {
                return false;
            }
            speeds[i] = speedSum / n;
            volts[i] = voltSum / n;

            if (speeds[i] < 50.0) {
                failure = "No speed reading at " + String.format(Locale.US, "%.1f", commanded)
                        + " V. Check the shooter encoder cable and motor direction.";
                return false;
            }
            if (i > 0 && speeds[i] <= speeds[i - 1]) {
                failure = "Speed did not rise with voltage. Check the battery and the shooter.";
                return false;
            }

            // Response time: how long to cover 63% of this step (skip the start from rest).
            double change = speeds[i] - startSpeed;
            if (i > 0 && change > 50.0) {
                double threshold = startSpeed + 0.632 * change;
                for (int k = 0; k < count; k++) {
                    if (sampleSpeed[k] >= threshold) {
                        responseSum += sampleTime[k];
                        responseCount++;
                        break;
                    }
                }
            }
        }

        // Least squares: volts = KS + KV * speed
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (int i = 0; i < points; i++) {
            sx += speeds[i];
            sy += volts[i];
            sxx += speeds[i] * speeds[i];
            sxy += speeds[i] * volts[i];
        }
        double denominator = points * sxx - sx * sx;
        if (Math.abs(denominator) < 1e-9) {
            failure = "Could not fit the feedforward line.";
            return false;
        }
        double kV = (points * sxy - sx * sy) / denominator;
        double kS = (sy - kV * sx) / points;
        if (kS < 0) {
            // Friction cannot be negative: refit a line through zero.
            kS = 0.0;
            kV = sxy / sxx;
        }
        if (kV <= 0) {
            failure = "Feedforward fit gave a KV of zero or less.";
            return false;
        }

        double worst = 0.0;
        for (int i = 0; i < points; i++) {
            double predicted = kS + kV * speeds[i];
            worst = Math.max(worst, Math.abs(volts[i] - predicted) / volts[i]);
        }
        fitErrorPercent = worst * 100.0;

        if (responseCount > 0) {
            responseSec = Range.clip(responseSum / responseCount, 0.3, 5.0);
        } else {
            responseSec = DEFAULT_RESPONSE_SEC;
            warnings.add("Could not time the wheel response. Used a default for KI.");
        }

        FlywheelConstants.KS = kS;
        FlywheelConstants.KV = kV;
        return true;
    }

    /** Hold the target under closed-loop control and measure how steady it is. Null if stopped. */
    private HoldResult holdAndMeasure(String stage, double target, double seconds) {
        flywheel.setTargetVelocity(target);
        flywheel.setRunning(true);

        ElapsedTime timer = new ElapsedTime();
        while (step()) {
            if (Math.abs(target - flywheel.getVelocity()) < 0.05 * target || timer.seconds() > 10.0) {
                break;
            }
            status(stage, "spinning up");
        }
        timer.reset();
        while (step() && timer.seconds() < 2.0) {
            status(stage, "settling");
        }

        double[] speed = new double[MAX_SAMPLES];
        double[] power = new double[MAX_SAMPLES];
        int n = 0;
        double readyNoise = 0.0;
        timer.reset();
        while (step() && timer.seconds() < seconds) {
            if (n < MAX_SAMPLES) {
                speed[n] = flywheel.getVelocity();
                power[n] = flywheel.getLastPower();
                n++;
            }
            readyNoise = Math.max(readyNoise, Math.abs(flywheel.getSmoothedVelocity() - target));
            status(stage, "measuring");
        }
        if (!opModeIsActive() || n < 3) {
            return null;
        }

        HoldResult result = new HoldResult();
        result.readyNoiseFraction = readyNoise / target;
        double speedSum = 0, powerSum = 0;
        for (int i = 0; i < n; i++) {
            speedSum += speed[i];
            powerSum += power[i];
        }
        result.meanSpeed = speedSum / n;
        result.meanPower = powerSum / n;
        result.errorFraction = (target - result.meanSpeed) / target;

        double powerVar = 0, rawPeak = 0, smoothPeak = 0, smallestStep = Double.POSITIVE_INFINITY;
        for (int i = 0; i < n; i++) {
            powerVar += (power[i] - result.meanPower) * (power[i] - result.meanPower);
            rawPeak = Math.max(rawPeak, Math.abs(speed[i] - target));
            if (i >= 2) {
                double mean3 = (speed[i] + speed[i - 1] + speed[i - 2]) / 3.0;
                smoothPeak = Math.max(smoothPeak, Math.abs(mean3 - result.meanSpeed));
            }
            if (i >= 1) {
                double change = Math.abs(speed[i] - speed[i - 1]);
                if (change > 0.5) {
                    smallestStep = Math.min(smallestStep, change);
                }
            }
        }
        result.powerStd = Math.sqrt(powerVar / n);
        result.rawPeakFraction = rawPeak / target;
        result.smoothPeakFraction = smoothPeak / target;
        result.speedStep = Double.isInfinite(smallestStep) ? 0.0 : smallestStep;
        return result;
    }

    /** Stage 4: drop to 75% of target, step back up, measure overshoot. Null if stopped. */
    private StepResult stepTest(double band, double target, int number) {
        String stage = String.format(Locale.US, "4/5 Recovery band %.2f (try %d)", band, number);
        FlywheelConstants.RECOVER_BAND = band;

        double low = STEP_FROM_FRACTION * target;
        flywheel.setTargetVelocity(low);
        flywheel.setRunning(true);
        ElapsedTime timer = new ElapsedTime();
        while (step()) {
            if (flywheel.getVelocity() <= low * 1.03 || timer.seconds() > 15.0) {
                break;
            }
            status(stage, "coasting down");
        }
        timer.reset();
        while (step() && timer.seconds() < 1.0) {
            status(stage, "holding low speed");
        }
        if (!opModeIsActive()) {
            return null;
        }

        StepResult result = new StepResult();
        result.readySec = Double.NaN;
        double a = flywheel.getVelocity(), b = a, c = a;
        double peak = Double.NEGATIVE_INFINITY;
        double reachedAt = Double.NaN;

        flywheel.setTargetVelocity(target);
        timer.reset();
        while (step()) {
            double now = timer.seconds();
            double speed = flywheel.getVelocity();
            a = b;
            b = c;
            c = speed;
            if (Double.isNaN(reachedAt) && speed >= target * (1.0 - FlywheelConstants.READY_TOL)) {
                reachedAt = now;
            }
            if (!Double.isNaN(reachedAt)) {
                peak = Math.max(peak, (a + b + c) / 3.0);
            }
            if (Double.isNaN(result.readySec) && flywheel.atSpeed()) {
                result.readySec = now;
            }
            if (!Double.isNaN(reachedAt) && now - reachedAt > 3.0) {
                break;
            }
            if (now > 10.0) {
                break;
            }
            status(stage, "stepping up to target");
        }
        if (!opModeIsActive()) {
            return null;
        }
        result.reached = !Double.isNaN(reachedAt);
        result.overshootFraction = result.reached ? (peak - target) / target : Double.NaN;
        return result;
    }

    // ---------------- Helpers ----------------

    /** One control loop. False once the OpMode is stopping. */
    private boolean step() {
        flywheel.update();
        if (LOOP_SLEEP_MS > 0) {
            sleep(LOOP_SLEEP_MS);
        }
        return opModeIsActive();
    }

    private void status(String stage, String detail) {
        telemetry.addLine("FLYWHEEL AUTO TUNE - keep clear of the shooter");
        telemetry.addData("Stage", stage);
        telemetry.addData("Doing", detail);
        telemetry.addData("Speed", "%.0f ticks/s", flywheel.getVelocity());
        telemetry.addData("Power", "%.2f", flywheel.getLastPower());
        telemetry.addData("Battery", "%.2f V", flywheel.getBatteryVoltage());
        telemetry.update();
    }

    private void saveOriginals() {
        oldKs = FlywheelConstants.KS;
        oldKv = FlywheelConstants.KV;
        oldKp = FlywheelConstants.KP;
        oldKi = FlywheelConstants.KI;
        oldBand = FlywheelConstants.RECOVER_BAND;
        oldReadyTol = FlywheelConstants.READY_TOL;
        oldShotDrop = FlywheelConstants.SHOT_DROP;
    }

    private void restoreOriginals() {
        FlywheelConstants.KS = oldKs;
        FlywheelConstants.KV = oldKv;
        FlywheelConstants.KP = oldKp;
        FlywheelConstants.KI = oldKi;
        FlywheelConstants.RECOVER_BAND = oldBand;
        FlywheelConstants.READY_TOL = oldReadyTol;
        FlywheelConstants.SHOT_DROP = oldShotDrop;
    }

    private void showResults() {
        telemetry.addLine(warnings.isEmpty()
                ? "FLYWHEEL AUTO TUNE - DONE, ALL CHECKS PASS"
                : "FLYWHEEL AUTO TUNE - DONE, SEE WARNINGS");
        for (String warning : warnings) {
            telemetry.addLine("WARNING: " + warning);
        }
        telemetry.addLine("--- Type these into FlywheelConstants.java ---");
        telemetry.addData("KS", "%.3f", FlywheelConstants.KS);
        telemetry.addData("KV", "%.6f", FlywheelConstants.KV);
        telemetry.addData("KP", "%.5f", FlywheelConstants.KP);
        telemetry.addData("KI", "%.5f", FlywheelConstants.KI);
        telemetry.addData("RECOVER_BAND", "%.2f", FlywheelConstants.RECOVER_BAND);
        telemetry.addData("READY_TOL", "%.3f", FlywheelConstants.READY_TOL);
        telemetry.addData("SHOT_DROP", "%.3f", FlywheelConstants.SHOT_DROP);
        telemetry.addLine("--- Checks ---");
        telemetry.addData("Battery start / end", "%.2f / %.2f V", startVolts, endVolts);
        telemetry.addData("Feedforward fit error", "%.1f %%", fitErrorPercent);
        telemetry.addData("Wheel response time", "%.2f s", responseSec);
        telemetry.addData("Hold error", "%+.2f %%", holdErrorPercent);
        telemetry.addData("Speed noise raw / smooth", "%.1f / %.1f %% (step %.0f)",
                noisePercent, smoothNoisePercent, speedStep);
        telemetry.addData("Power jitter", "%.3f", powerStd);
        telemetry.addData("Overshoot", "%.1f %%", overshootPercent);
        telemetry.addData("75% to ready", "%.2f s", spinUpSec);
        telemetry.addData("Hold power now", "%.2f", holdPower);
        telemetry.addData("Hold power at " + String.format(Locale.US, "%.1f", LOW_BATTERY_VOLTS) + " V",
                "%.2f", holdPowerLowBattery);
        telemetry.update();
    }

    private void logResults() {
        RobotLog.ii("FlywheelTune", String.format(Locale.US,
                "KS=%.3f KV=%.6f KP=%.5f KI=%.5f RECOVER_BAND=%.2f READY_TOL=%.3f SHOT_DROP=%.3f",
                FlywheelConstants.KS, FlywheelConstants.KV, FlywheelConstants.KP, FlywheelConstants.KI,
                FlywheelConstants.RECOVER_BAND, FlywheelConstants.READY_TOL, FlywheelConstants.SHOT_DROP));
        RobotLog.ii("FlywheelTune", String.format(Locale.US,
                "battery=%.2f/%.2f fitErr=%.1f%% response=%.2fs holdErr=%+.2f%% noise=%.1f%% "
                        + "smoothNoise=%.1f%% step=%.0f "
                        + "powerStd=%.3f overshoot=%.1f%% ready=%.2fs holdPower=%.2f warnings=%d",
                startVolts, endVolts, fitErrorPercent, responseSec, holdErrorPercent, noisePercent,
                smoothNoisePercent, speedStep, powerStd, overshootPercent, spinUpSec, holdPower,
                warnings.size()));
    }
}
