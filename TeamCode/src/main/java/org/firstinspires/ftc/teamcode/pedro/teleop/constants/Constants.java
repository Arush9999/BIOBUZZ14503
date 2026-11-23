package org.firstinspires.ftc.teamcode.pedro.teleop.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {

    // ---------------- DRIVE ----------------
    public static double DRIVE_POWER = 1.0;
    public static double STRAFE_MULTIPLIER = 1.1;
    public static double TURN_MULTIPLIER = 0.7;

    // ---------------- INTAKE ----------------
    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_TRIGGER_DEADZONE = 0.1;

    // ---------------- TRANSFER ----------------
    // Forward/shoot power (auto + teleop). Reverse uses TRANSFER_REVERSE_POWER.
    public static double TRANSFER_SHOOT_POWER = 1.0;
    public static double TRANSFER_REVERSE_POWER = 1.0;
    // Legacy alias for reverse / intake-hold reverse.
    public static double TRANSFER_POWER = TRANSFER_REVERSE_POWER;

    // ---------------- FLOWER FLAP ----------------
    // 0 = up, 0.45 = down/deployed
    public static double FLOWER_FLAP_UP = 0.0;
    public static double FLOWER_FLAP_DOWN = 0.45;
    public static double FLOWER_FLAP_TOLERANCE = 0.05;

    // ---------------- SHOOTER ----------------
    // Software PIDF: P/I/F on raw velocity; LPF only on D (filtered dV/dt).
    // F is REV-style: power ≈ (P*e + I*i + D*d + F*target) / PID_SCALE.
    public static double SHOOTER_P = 0.0;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_D = 0.0;
    public static double SHOOTER_F = 14.5;
    public static double SHOOTER_TARGET_VELOCITY = 1825;
    public static double SHOOTER_LOW = 1200;
    public static double SHOOTER_MEDIUM = 1500;
    public static double SHOOTER_HIGH = 1825;
    // REV motor internal scale (same units as setVelocityPIDFCoefficients).
    public static double SHOOTER_PID_SCALE = 32767.0;
    // LPF on D only: filteredD += alpha * (rawD - filteredD). Lower = smoother D.
    public static double SHOOTER_D_FILTER = 0.18;
    // Integral anti-windup clamp (ticks/s * s).
    public static double SHOOTER_I_MAX = 500.0;
    // Voltage comp: scale F (and P) by NOMINAL / battery. Tune F at ~NOMINAL volts.
    public static double SHOOTER_NOMINAL_VOLTAGE = 12.5;
    public static double SHOOTER_MIN_VOLTAGE = 9.0;
    public static double SHOOTER_VOLTAGE_FILTER = 0.25;

    // ---------------- TURRET ----------------
    // Dual positional (non-continuous) servos, same direction, same command.
    // Motion along the number line only (never wraps through the rear cut):
    //   low → high = CCW, high → low = CW. Soft travel limited to MIN..MAX.
    public static double TURRET_MIN_ANGLE = -180.0;
    public static double TURRET_MAX_ANGLE = 180.0;
    // Full mathematical map (pos falls as angle rises); command clipped to MIN..MAX.
    public static double TURRET_POS_AT_MIN_ANGLE = 1.0;
    public static double TURRET_POS_AT_MAX_ANGLE = 0.0;
    // Degrees → servo units. Retune if POS_AT_* or MIN/MAX change.
    public static double TURRET_DEGREE_TO_SERVO = 1.0 / 360.0;
    // Manual / mount trim (deg) applied before negation; position trim after.
    public static double TURRET_OFFSET = 0.0;
    public static double TURRET_POSITION_OFFSET = 0.0;
    // Servo center / soft command limits (never command outside MIN..MAX).
    public static double TURRET_CENTER = 0.5;
    public static double TURRET_MIN = 0.2;
    public static double TURRET_MAX = 0.8;
    public static double TURRET_STEP = 0.005;
    // Max servo-position change per aim loop (number-line slew, no wrap).
    public static double TURRET_MAX_STEP_POS = 0.02;
    // Turret pivot from Pinpoint: +X robot forward, +Y robot left.
    public static double TURRET_OFFSET_X = 0.0;
    public static double TURRET_OFFSET_Y = 0.0;

    // ---------------- AUTO AIM TARGET (field inches) ----------------
    // Active goal (switched with alliance). Defaults to red.
    public static double GOAL_X = 58.0;
    public static double GOAL_Y = 72.0;
    public static double RED_GOAL_X = 58.0;
    public static double RED_GOAL_Y = 72.0;
    public static double BLUE_GOAL_X = 84.0;
    public static double BLUE_GOAL_Y = 72.0;

    // Expected teleop start pose. Reset sets odometry here.
    public static double START_X = 12.0;
    public static double START_Y = 120.0;
    public static double START_HEADING_DEG = 0.0;
    public static double RED_START_X = 12.0;
    public static double RED_START_Y = 120.0;
    public static double BLUE_START_X = 12.0;
    public static double BLUE_START_Y = 120.0;
}
