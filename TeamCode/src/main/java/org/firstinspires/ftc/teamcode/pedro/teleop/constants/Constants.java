package org.firstinspires.ftc.teamcode.pedro.teleop.constants;

public class Constants {

    // ---------------- DRIVE ----------------
    public static double DRIVE_POWER = 1.0;
    public static double STRAFE_MULTIPLIER = 1.1;
    public static double TURN_MULTIPLIER = 0.5;

    // ---------------- INTAKE ----------------
    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_TRIGGER_DEADZONE = 0.1;

    // ---------------- TRANSFER ----------------
    public static double TRANSFER_POWER = 1.0;

    // ---------------- FLOWER FLAP ----------------
    // 0 = up, 0.45 = down/deployed
    public static double FLOWER_FLAP_UP = 0.0;
    public static double FLOWER_FLAP_DOWN = 0.45;
    public static double FLOWER_FLAP_TOLERANCE = 0.05;

    // ---------------- SHOOTER ----------------
    public static double SHOOTER_P = 0.8000;
    public static double SHOOTER_I = 0.0050;
    public static double SHOOTER_D = 0.0000;
    public static double SHOOTER_F = 14.5000;
    public static double SHOOTER_TARGET_VELOCITY = 1200;
    public static double SHOOTER_LOW = 1200;
    public static double SHOOTER_MEDIUM = 1500;
    public static double SHOOTER_HIGH = 1800;

    // ---------------- TURRET ----------------
    // 0.5 = forward, 0 and 1 meet at the rear (full +/-180 deg).
    public static double TURRET_CENTER = 0.5;
    public static double TURRET_MIN = 0.0;
    public static double TURRET_MAX = 1.0;
    public static double TURRET_STEP = 0.01;
    // Left/right aim sense.
    public static double TURRET_DIRECTION = -1.0;
    // Optional mount offset in degrees (positive = turret zero is CW of robot forward).
    public static double TURRET_MOUNT_OFFSET_DEG = 0.0;
    // Turret pivot from Pinpoint: +X robot forward, +Y robot left.
    public static double TURRET_OFFSET_X = 0.0;
    public static double TURRET_OFFSET_Y = 0.0;

    // ---------------- AUTO AIM TARGET (field inches) ----------------
    public static double GOAL_X = 72.0;
    public static double GOAL_Y = 48.0;

    // Expected teleop start pose. GP2 A resets odometry here.
    public static double START_X = 60.0;
    public static double START_Y = 8.0;
    public static double START_HEADING_DEG = 95.0;
}
