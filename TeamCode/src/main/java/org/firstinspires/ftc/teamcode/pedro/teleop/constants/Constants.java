package org.firstinspires.ftc.teamcode.pedro.teleop.constants;

import com.acmerobotics.dashboard.config.Config;

/**
 * Shared tunables for teleop subsystems.
 * Edit here (or via FTC Dashboard) — do not re-declare these values in OpModes.
 */
@Config
public class Constants {

    // ---- Drive ----
    public static double DRIVE_POWER = 1.0;
    public static double STRAFE_MULTIPLIER = 1.1;
    public static double TURN_MULTIPLIER = 0.7;

    // ---- Intake ----
    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_TRIGGER_DEADZONE = 0.1;

    // ---- Transfer ----
    public static double TRANSFER_SHOOT_POWER = 1.0;
    public static double TRANSFER_REVERSE_POWER = 1.0;
    /** Max wait (s) for shooter at-speed before feeding anyway. */
    public static double FEED_GATE_TIMEOUT_SEC = 1.0;

    // ---- Flower flap ----
    public static double FLOWER_FLAP_UP = 0.0;
    public static double FLOWER_FLAP_DOWN = 0.45;
    public static double FLOWER_FLAP_TOLERANCE = 0.05;

    // ---- Shooter (bang-bang) ----
    /** Below target - hyst → ON_POWER; above target + hyst → coast. */
    public static double SHOOTER_TARGET_VELOCITY = 1825;
    public static double SHOOTER_HYSTERESIS = 5;
    public static double SHOOTER_ON_POWER = 1.0;
    /** Extra ticks/s on target while feeding. */
    public static double SHOOTER_BURST_OFFSET = 40.0;

    public static double SHOOTER_VOLTAGE_READ_SEC = 0.04;
    public static double SHOOTER_VOLTAGE_TAU_SEC = 0.10;

    public static double SHOOTER_SPEED_SMOOTH_SEC = 0.05;
    public static double SHOOTER_READY_TOL = 0.015;
    public static double SHOOTER_READY_SEC = 0.06;

    public static double SHOOTER_SHOT_DROP = 0.03;
    public static double SHOOTER_SHOT_WINDOW_SEC = 0.15;
    public static double SHOOTER_SHOT_REARM = 0.012;
    public static double SHOOTER_SHOT_WATCH_AFTER_FEED_SEC = 0.3;
    public static double SHOOTER_BURST_GAP_SEC = 1.0;

    // ---- Turret (dual positional servos, same command; no wrap) ----
    public static double TURRET_MIN_ANGLE = -180.0;
    public static double TURRET_MAX_ANGLE = 180.0;
    public static double TURRET_POS_AT_MIN_ANGLE = 1.0;
    public static double TURRET_POS_AT_MAX_ANGLE = 0.0;
    public static double TURRET_DEGREE_TO_SERVO = 1.0 / 360.0;
    public static double TURRET_OFFSET = 0.0;
    public static double TURRET_POSITION_OFFSET = 0.0;
    public static double TURRET_CENTER = 0.5;
    public static double TURRET_MIN = 0.2;
    public static double TURRET_MAX = 0.8;
    public static double TURRET_STEP = 0.005;
    public static double TURRET_MAX_STEP_POS = 0.02;
    public static double TURRET_OFFSET_X = 0.0;
    public static double TURRET_OFFSET_Y = 0.0;

    // ---- Auto aim / start pose (field inches) ----
    /** Active goal / start — updated by alliance + near/far select. */
    public static double GOAL_X = 58.0;
    public static double GOAL_Y = 72.0;
    public static double START_X = 60.0;
    public static double START_Y = 8.0;
    public static double START_HEADING_DEG = 90.0;

    public static double RED_GOAL_X = 58.0;
    public static double RED_GOAL_Y = 72.0;
    public static double BLUE_GOAL_X = 84.0;
    public static double BLUE_GOAL_Y = 72.0;

    /** Red near wall: (60, 8) @ 90°. Far wall: (60, 136) @ 270°. */
    public static double RED_START_NEAR_X = 60.0;
    public static double RED_START_NEAR_Y = 8.0;
    public static double RED_START_NEAR_HEADING_DEG = 90.0;
    public static double RED_START_FAR_X = 60.0;
    public static double RED_START_FAR_Y = 136.0;
    public static double RED_START_FAR_HEADING_DEG = 270.0;

    /** Blue = red + 24 in X (same Y / heading). */
    public static double BLUE_START_OFFSET_X = 24.0;

    // Active copies used by pose reset (filled by applyAlliance).
    public static double RED_START_X = RED_START_NEAR_X;
    public static double RED_START_Y = RED_START_NEAR_Y;
    public static double BLUE_START_X = RED_START_NEAR_X + BLUE_START_OFFSET_X;
    public static double BLUE_START_Y = RED_START_NEAR_Y;
}
