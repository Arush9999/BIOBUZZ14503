package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "BIOBUZZ Shooter PIDF Tester", group = "Testing")
public class BIOBUZZ_ShooterPIDFTester extends LinearOpMode {

    // ============================================================
    // HARDWARE
    // ============================================================

    private DcMotorEx shooter;

    // ============================================================
    // STARTING PIDF VALUES
    // ============================================================

    public static double P = 0.0000;
    public static double I = 0.0000;
    public static double D = 0.0000;
    public static double F = 0.0000;

    // ============================================================
    // TEST SETTINGS
    // ============================================================

    public static double TARGET_VELOCITY = 1800;

    // How much each button press changes the value
    public static double P_INCREMENT = 0.0001;
    public static double I_INCREMENT = 0.00001;
    public static double D_INCREMENT = 0.0001;
    public static double F_INCREMENT = 0.0001;

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

    @Override
    public void runOpMode() {

        // ========================================================
        // HARDWARE
        // ========================================================

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setMode(
                com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER
        );

        shooter.setZeroPowerBehavior(
                com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.FLOAT
        );

        // Apply starting PIDF
        applyPIDF();

        telemetry.addLine("BIOBUZZ SHOOTER PIDF TESTER");
        telemetry.addLine("---------------------------");
        telemetry.addLine("");
        telemetry.addLine("A / B     = Change P");
        telemetry.addLine("X / Y     = Change F");
        telemetry.addLine("D-PAD     = Change target velocity");
        telemetry.addLine("Bumpers   = Change D");
        telemetry.addLine("START     = Shooter ON/OFF");
        telemetry.addLine("");
        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ====================================================
            // PIDF ADJUSTMENT
            // ====================================================

            adjustPIDF();

            // ====================================================
            // SHOOTER
            // ====================================================

            if (shooterRunning) {

                shooter.setVelocity(TARGET_VELOCITY);

            } else {

                shooter.setVelocity(0);
            }

            // ====================================================
            // TELEMETRY
            // ====================================================

            showTelemetry();
        }
    }

    // ============================================================
    // APPLY PIDF
    // ============================================================

    private void applyPIDF() {

        shooter.setVelocityPIDFCoefficients(
                P,
                I,
                D,
                F
        );
    }

    // ============================================================
    // PIDF CONTROLS
    // ============================================================

    private void adjustPIDF() {

        // --------------------------------------------------------
        // P
        // --------------------------------------------------------

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

        // --------------------------------------------------------
        // F
        // --------------------------------------------------------

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

        // --------------------------------------------------------
        // D
        // --------------------------------------------------------

        if (gamepad1.left_bumper && !lastLeftBumper) {

            D += D_INCREMENT;
            applyPIDF();
        }

        if (gamepad1.right_bumper && !lastRightBumper) {

            D -= D_INCREMENT;

            if (D < 0) {
                D = 0;
            }

            applyPIDF();
        }

        // --------------------------------------------------------
        // TARGET VELOCITY
        // --------------------------------------------------------

        if (gamepad1.dpad_up && !lastDpadUp) {

            TARGET_VELOCITY += 50;
        }

        if (gamepad1.dpad_down && !lastDpadDown) {

            TARGET_VELOCITY -= 50;

            if (TARGET_VELOCITY < 0) {
                TARGET_VELOCITY = 0;
            }
        }

        // --------------------------------------------------------
        // INCREMENT SIZE
        // --------------------------------------------------------

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

        // ========================================================
        // SAVE BUTTON STATES
        // ========================================================

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

    // ============================================================
    // TELEMETRY
    // ============================================================

    private void showTelemetry() {

        double actualVelocity = shooter.getVelocity();

        double error = TARGET_VELOCITY - actualVelocity;

        telemetry.addLine("===== PIDF TESTER =====");

        telemetry.addLine("");

        telemetry.addData(
                "Shooter",
                shooterRunning ? "RUNNING" : "OFF"
        );

        telemetry.addData(
                "Target Velocity",
                "%.0f",
                TARGET_VELOCITY
        );

        telemetry.addData(
                "Actual Velocity",
                "%.0f",
                actualVelocity
        );

        telemetry.addData(
                "Error",
                "%.0f",
                error
        );

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

        telemetry.addLine("CONTROLS");
        telemetry.addLine("A/B = P +/-");
        telemetry.addLine("X/Y = F +/-");
        telemetry.addLine("LB/RB = D +/-");
        telemetry.addLine("DPad Up/Down = Target");
        telemetry.addLine("DPad Left/Right = Step Size");

        telemetry.update();
    }
}