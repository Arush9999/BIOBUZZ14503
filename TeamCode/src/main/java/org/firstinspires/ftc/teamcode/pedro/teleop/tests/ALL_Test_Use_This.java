package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Combined test: shooter PIDF, robot-centric drive, intake, and turretL.
 * Gamepad 1 runs the PIDF tester plus drive and intake.
 * Gamepad 2 runs the turretL servo test.
 */
@TeleOp(name = "Code Test", group = "Testing")
public class ALL_Test_Use_This extends LinearOpMode {

    // ============================================================
    // HARDWARE
    // ============================================================

    private DcMotorEx shooterLeft;
    private DcMotorEx shooterRight;
    private DcMotor intake;
    private DcMotor LF, LB, RF, RB;
    private Servo turretL;

    // ---------------- DRIVE (master) ----------------
    public static double DRIVE_POWER = 1.0;
    public static double STRAFE_MULTIPLIER = 1.1;
    public static double TURN_MULTIPLIER = 0.5;

    // ---------------- INTAKE (master) ----------------
    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_TRIGGER_DEADZONE = 0.1;

    // ---------------- TURRET L (master) ----------------
    public static double TURRET_CENTER = 0.5;
    public static double TURRET_MIN = 0.0;
    public static double TURRET_MAX = 1.0;
    public static double TURRET_STEP = 0.02;

    private double turretPosition = TURRET_CENTER;
    private boolean turretAWasPressed = false;
    private boolean turretBWasPressed = false;

    // ============================================================
    // STARTING PIDF VALUES (kanishka)
    // ============================================================

    public static double P = 0.8000;
    public static double I = 0.0050;
    public static double D = 0.0000;
    public static double F = 14.5000;

    public static double TARGET_VELOCITY = 1200;

    public static double P_INCREMENT = 0.01;
    public static double I_INCREMENT = 0.0005;
    public static double D_INCREMENT = 0.1;
    public static double F_INCREMENT = 0.5;

    // ============================================================
    // BUTTON STATE
    // ============================================================

    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastX = false;
    private boolean lastY = false;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    private boolean shooterRunning = false;
    private boolean lastStart = false;
    private boolean lastLeftStickButton = false;

    @Override
    public void runOpMode() {

        shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        LF = hardwareMap.get(DcMotor.class, "LF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        RF.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.REVERSE);
        LF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.FORWARD);
        for (DcMotor motor : new DcMotor[]{LF, LB, RF, RB}) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        turretL = hardwareMap.get(Servo.class, "turretL");
        turretL.setPosition(turretPosition);

        shooterLeft.setDirection(DcMotorEx.Direction.FORWARD);
        shooterRight.setDirection(DcMotorEx.Direction.REVERSE);
        shooterLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        applyPIDF();

        telemetry.addLine("CODE TEST");
        telemetry.addLine("Gamepad 1: drive, intake, PIDF");
        telemetry.addLine("A / B     = Change P");
        telemetry.addLine("X / Y     = Change F");
        telemetry.addLine("D-PAD     = Target and step size");
        telemetry.addLine("Bumpers   = Change I");
        telemetry.addLine("START or L3 = Shooter ON/OFF");
        telemetry.addLine("RT / LT   = Intake");
        telemetry.addLine("Gamepad 2 turretL: A/B step, X=0, Y=1");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        lastStart = gamepad1.start;
        lastLeftStickButton = gamepad1.left_stick_button;

        try {
            while (opModeIsActive()) {
                adjustPIDF();
                driveRobot();
                runIntake();
                runTurretL();

                if ((gamepad1.start && !lastStart)
                        || (gamepad1.left_stick_button && !lastLeftStickButton)) {
                    shooterRunning = !shooterRunning;
                }
                lastStart = gamepad1.start;
                lastLeftStickButton = gamepad1.left_stick_button;

                if (shooterRunning) {
                    shooterLeft.setVelocity(TARGET_VELOCITY);
                    shooterRight.setVelocity(TARGET_VELOCITY);
                } else {
                    shooterLeft.setVelocity(0);
                    shooterRight.setVelocity(0);
                }

                showTelemetry();
            }
        } finally {
            shooterLeft.setVelocity(0);
            shooterRight.setVelocity(0);
            intake.setPower(0);
            LF.setPower(0);
            LB.setPower(0);
            RF.setPower(0);
            RB.setPower(0);
        }
    }

    // Master robot-centric drive: strafe 1.1, turn 0.5, full drive power.
    private void driveRobot() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * STRAFE_MULTIPLIER;
        double rx = gamepad1.right_stick_x * TURN_MULTIPLIER;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        LF.setPower(DRIVE_POWER * (y + x + rx) / denominator);
        LB.setPower(DRIVE_POWER * (y - x + rx) / denominator);
        RF.setPower(DRIVE_POWER * (y - x - rx) / denominator);
        RB.setPower(DRIVE_POWER * (y + x - rx) / denominator);
    }

    // Master intake: right trigger forward, left trigger reverse.
    private void runIntake() {
        double forwardTrigger = gamepad1.right_trigger;
        double reverseTrigger = gamepad1.left_trigger;

        if (forwardTrigger > INTAKE_TRIGGER_DEADZONE) {
            intake.setPower(INTAKE_POWER * forwardTrigger);
        } else if (reverseTrigger > INTAKE_TRIGGER_DEADZONE) {
            intake.setPower(-INTAKE_POWER * reverseTrigger);
        } else {
            intake.setPower(0);
        }
    }

    // Master turretL test. Gamepad 2, because gamepad 1 A/B/X/Y tune PIDF.
    private void runTurretL() {
        if (gamepad2.x) {
            turretPosition = TURRET_MIN;
        } else if (gamepad2.y) {
            turretPosition = TURRET_MAX;
        } else if (gamepad2.a && !turretAWasPressed) {
            turretPosition = Range.clip(turretPosition + TURRET_STEP, TURRET_MIN, TURRET_MAX);
        } else if (gamepad2.b && !turretBWasPressed) {
            turretPosition = Range.clip(turretPosition - TURRET_STEP, TURRET_MIN, TURRET_MAX);
        }

        turretAWasPressed = gamepad2.a;
        turretBWasPressed = gamepad2.b;
        turretL.setPosition(turretPosition);
    }

    private void applyPIDF() {
        shooterLeft.setVelocityPIDFCoefficients(P, I, D, F);
        shooterRight.setVelocityPIDFCoefficients(P, I, D, F);
    }

    private void adjustPIDF() {
        if (gamepad1.a && !lastA) {
            P += P_INCREMENT;
            applyPIDF();
        }

        if (gamepad1.b && !lastB) {
            P -= P_INCREMENT;
            if (P < 0) {
                P = 0;
            }
            applyPIDF();
        }

        if (gamepad1.x && !lastX) {
            F += F_INCREMENT;
            applyPIDF();
        }

        if (gamepad1.y && !lastY) {
            F -= F_INCREMENT;
            if (F < 0) {
                F = 0;
            }
            applyPIDF();
        }

        if (gamepad1.left_bumper && !lastLeftBumper) {
            I += I_INCREMENT;
            applyPIDF();
        }

        if (gamepad1.right_bumper && !lastRightBumper) {
            I -= I_INCREMENT;
            if (I < 0) {
                I = 0;
            }
            applyPIDF();
        }

        if (gamepad1.dpad_up && !lastDpadUp) {
            TARGET_VELOCITY += 12.5;
        }

        if (gamepad1.dpad_down && !lastDpadDown) {
            TARGET_VELOCITY -= 12.5;
            if (TARGET_VELOCITY < 0) {
                TARGET_VELOCITY = 0;
            }
        }

        if (gamepad1.dpad_right && !lastDpadRight) {
            P_INCREMENT *= 2;
            I_INCREMENT *= 2;
            D_INCREMENT *= 2;
            F_INCREMENT *= 2;
        }

        if (gamepad1.dpad_left && !lastDpadLeft) {
            P_INCREMENT /= 2;
            I_INCREMENT /= 2;
            D_INCREMENT /= 2;
            F_INCREMENT /= 2;
        }

        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastX = gamepad1.x;
        lastY = gamepad1.y;

        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;

        lastLeftBumper = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;
    }

    private void showTelemetry() {
        double leftVelocity = shooterLeft.getVelocity();
        double rightVelocity = shooterRight.getVelocity();

        telemetry.addLine("===== CODE TEST =====");
        telemetry.addData("Shooter", shooterRunning ? "RUNNING" : "OFF");
        telemetry.addData("Target Velocity", "%.0f", TARGET_VELOCITY);
        telemetry.addData("Left Velocity", "%.0f", leftVelocity);
        telemetry.addData("Right Velocity", "%.0f", rightVelocity);
        telemetry.addData("Left Error", "%.0f", TARGET_VELOCITY - leftVelocity);
        telemetry.addData("Right Error", "%.0f", TARGET_VELOCITY - rightVelocity);
        telemetry.addData("Intake power", "%.2f", intake.getPower());

        telemetry.addLine("");
        telemetry.addLine("PIDF VALUES");
        telemetry.addData("P", "%.7f", P);
        telemetry.addData("I", "%.7f", I);
        telemetry.addData("D", "%.7f", D);
        telemetry.addData("F", "%.7f", F);

        telemetry.addLine("");
        telemetry.addLine("INCREMENTS");
        telemetry.addData("P Step", "%.7f", P_INCREMENT);
        telemetry.addData("I Step", "%.7f", I_INCREMENT);
        telemetry.addData("D Step", "%.7f", D_INCREMENT);
        telemetry.addData("F Step", "%.7f", F_INCREMENT);

        telemetry.addLine("");
        telemetry.addLine("TURRET L");
        telemetry.addData("turretL", "%.4f", turretPosition);
        telemetry.addData("step", "%.4f", TURRET_STEP);
        telemetry.addLine("Gamepad 2: A/B step, X = 0, Y = 1");

        telemetry.addLine("");
        telemetry.addLine("GAMEPAD 1");
        telemetry.addLine("Left stick = Drive / strafe");
        telemetry.addLine("Right stick X = Rotate");
        telemetry.addLine("A/B = P +/-");
        telemetry.addLine("X/Y = F +/-");
        telemetry.addLine("LB/RB = I +/-");
        telemetry.addLine("DPad Up/Down = Target");
        telemetry.addLine("DPad Left/Right = Step Size");
        telemetry.addLine("START or L3 = Shooters ON/OFF");
        telemetry.addLine("RT = Intake forward");
        telemetry.addLine("LT = Intake reverse");
        telemetry.update();
    }
}
