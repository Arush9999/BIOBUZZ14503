package org.firstinspires.ftc.teamcode.pedro.teleop.constants;

import com.acmerobotics.dashboard.config.Config;

/**
 * Settings for the voltage-based {@code Flywheel} controller (used by TeleOp V2 and
 * Flywheel_Auto_Tune). The old {@code Shooter} keeps using {@link Constants}.
 *
 * All gains are in VOLTS, so they do not depend on battery charge:
 *   motor power = volts wanted / live battery voltage.
 */
@Config
public class FlywheelConstants {

    // ---------------- TARGET ----------------
    public static double TARGET_VELOCITY = 1800; // ticks/s

    // ---------------- FEEDFORWARD (from Flywheel_Auto_Tune) ----------------
    // volts = KS + KV * target.
    // Tuned Nov 22 (at 1725): KV average of three runs (0.006079, 0.006011, 0.006075).
    // Feedforward scales with target, so no retune is required for 1800.
    public static double KS = 0.0;       // volts to overcome friction
    public static double KV = 0.006055;  // volts per tick/s

    // ---------------- FEEDBACK (from Flywheel_Auto_Tune) ----------------
    // Tuned Nov 22: KP same in all three runs, KI from the fresh-battery run.
    public static double KP = 0.02174;   // volts per tick/s of error
    public static double KI = 0.02006;   // volts per (tick/s of error * second)
    // I only builds up within this fraction of target, and never adds more than I_MAX_VOLTS.
    public static double I_ZONE = 0.03;
    public static double I_MAX_VOLTS = 0.6;

    // ---------------- RECOVERY ----------------
    // More than this fraction below target -> full power.
    public static double RECOVER_BAND = 0.03;
    // While feeding: 0 = full power any time the wheel is below the (offset) target.
    public static double BURST_RECOVER_BAND = 0.0;
    // Extra ticks/s added to the target while feeding, so ball 1 is slightly hot and
    // later balls land nearer the aim. Raise with GP1 B / lower with GP2 B in TeleOp V2.
    public static double BURST_OFFSET = 40.0;

    // ---------------- BATTERY VOLTAGE ----------------
    public static double VOLTAGE_READ_SEC = 0.04;  // how often the hubs are asked for voltage
    public static double VOLTAGE_TAU_SEC = 0.10;   // smoothing time on the voltage reading
    public static double MIN_VOLTAGE = 9.0;        // never compensate below this
    // Warn when just holding the target needs more than this much power.
    public static double MAX_HOLD_POWER = 0.95;

    // ---------------- READY CHECK ----------------
    // Single speed readings scatter by a few percent, so the ready check and the shot log use
    // the speed averaged over about this long. The motor control still uses the raw reading.
    public static double SPEED_SMOOTH_SEC = 0.05;
    // Ready = smoothed speed within READY_TOL of target for READY_SEC in a row.
    public static double READY_TOL = 0.015;
    public static double READY_SEC = 0.06;

    // ---------------- FEED (TeleOp V2) ----------------
    // Transfer volts while shooting. Lower = more gap between balls for recovery.
    // 11 V was too fast; start slower and raise with dpad U if the burst is too slow.
    public static double FEED_VOLTS = 7.5;
    // RB held this long without the wheel getting ready -> feed anyway. 0 = never wait.
    public static double FEED_GATE_TIMEOUT_SEC = 1.0;

    // ---------------- SHOT TRACKING ----------------
    // A ball counts as shot when speed falls this fraction of target within SHOT_WINDOW_SEC.
    public static double SHOT_DROP = 0.03;
    public static double SHOT_WINDOW_SEC = 0.15;
    // Speed must climb this fraction of target off the bottom before the next ball is counted.
    public static double SHOT_REARM = 0.012;
    // Keep watching for shots this long after the transfer stops.
    public static double SHOT_WATCH_AFTER_FEED_SEC = 0.3;
    // A pause in feeding longer than this starts a new burst on the telemetry.
    public static double BURST_GAP_SEC = 1.0;
}
