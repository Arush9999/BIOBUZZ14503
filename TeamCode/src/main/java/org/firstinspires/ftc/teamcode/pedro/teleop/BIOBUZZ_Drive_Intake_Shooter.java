package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BIOBUZZ Drive Intake Shooter", group = "BIOBUZZ")
public class BIOBUZZ_Drive_Intake_Shooter extends LinearOpMode {

    /* Tuned shooter PIDF coefficients, applied to both motors. */
    private static final double P = 0.6416;
    private static final double I = 0.00256;
    private static final double D = 0.0;
    private static final double F = 12.5;

    /* Edit these three presets after testing shot distances. */
    private static final double SHORT_VELOCITY = 1200;
    private static final double MEDIUM_VELOCITY = 1600;
    private static final double LONG_VELOCITY = 1800;

    private static final double INTAKE_POWER = 0.75;
    private static final double TRIGGER_DEADBAND = 0.15;
    private static final double DRIVE_SPEED_LIMIT = 1.0;
    private static final double TURN_SPEED_MULTIPLIER = 0.5;
    private static final double DRIVE_DEADBAND = 0.05;

    @Override
    public void runOpMode() {
        /* Hardware names match the existing drivetrain and PIDF tester. */
        DcMotor LF = hardwareMap.get(DcMotor.class, "LF");
        DcMotor LB = hardwareMap.get(DcMotor.class, "LB");
        DcMotor RF = hardwareMap.get(DcMotor.class, "RF");
        DcMotor RB = hardwareMap.get(DcMotor.class, "RB");
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotorEx shooterLeft = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        DcMotorEx shooterRight = hardwareMap.get(DcMotorEx.class, "shooterRight");

        /* Preserve directions from Robot_Centric_Drive.java. */
        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] driveMotors = {LF, LB, RF, RB};
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0);
        }

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setPower(0);

        /* Preserve shooter directions from the PIDF tester. */
        shooterLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterRight.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotorEx motor : new DcMotorEx[]{shooterLeft, shooterRight}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            motor.setVelocityPIDFCoefficients(P, I, D, F);
            motor.setVelocity(0);
        }

        boolean shooterEnabled = false;
        boolean previousA = false;
        double targetVelocity = MEDIUM_VELOCITY;
        String shootingMode = "MEDIUM";

        telemetry.addLine("GAMEPAD 1: left stick drive/strafe, right X turn");
        telemetry.addLine("GAMEPAD 1: RT intake forward, LT intake reverse");
        telemetry.addLine("GAMEPAD 1: A shooter toggle");
        telemetry.addLine("GAMEPAD 1: D-pad DOWN short, LEFT medium, UP long");
        telemetry.addData("Default target", "%.0f ticks/sec", targetVelocity);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        previousA = gamepad1.a;

        try {
            while (opModeIsActive()) {
                /* Full-speed translation; turning input is limited to 50%. */
                double y = deadband(-gamepad1.left_stick_y);
                double x = deadband(gamepad1.left_stick_x) * 1.1;
                double rx = deadband(gamepad1.right_stick_x) * TURN_SPEED_MULTIPLIER;
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

                LF.setPower(DRIVE_SPEED_LIMIT * (y + x + rx) / denominator);
                LB.setPower(DRIVE_SPEED_LIMIT * (y - x + rx) / denominator);
                RF.setPower(DRIVE_SPEED_LIMIT * (y - x - rx) / denominator);
                RB.setPower(DRIVE_SPEED_LIMIT * (y + x - rx) / denominator);

                /* Intake works independently of shooter; reverse has priority. */
                double intakePower = 0;
                if (gamepad1.left_trigger > TRIGGER_DEADBAND) {
                    intakePower = -INTAKE_POWER;
                } else if (gamepad1.right_trigger > TRIGGER_DEADBAND) {
                    intakePower = INTAKE_POWER;
                }
                intake.setPower(intakePower);

                /* Three shooting presets; medium is the default. */
                if (gamepad1.dpad_down) {
                    targetVelocity = SHORT_VELOCITY;
                    shootingMode = "SHORT";
                } else if (gamepad1.dpad_left) {
                    targetVelocity = MEDIUM_VELOCITY;
                    shootingMode = "MEDIUM";
                } else if (gamepad1.dpad_up) {
                    targetVelocity = LONG_VELOCITY;
                    shootingMode = "LONG";
                }

                /* A toggles both shooter motors once per press. */
                if (gamepad1.a && !previousA) {
                    shooterEnabled = !shooterEnabled;
                }
                previousA = gamepad1.a;

                double commandedVelocity = shooterEnabled ? targetVelocity : 0;
                shooterLeft.setVelocity(commandedVelocity);
                shooterRight.setVelocity(commandedVelocity);

                telemetry.addData("Shooter", shooterEnabled ? "ON" : "OFF");
                telemetry.addData("Mode", shootingMode);
                telemetry.addData("Target (ticks/s)", "%.0f", targetVelocity);
                telemetry.addData("Left actual", "%.0f", shooterLeft.getVelocity());
                telemetry.addData("Right actual", "%.0f", shooterRight.getVelocity());
                telemetry.addData("Left error", "%.0f", commandedVelocity - shooterLeft.getVelocity());
                telemetry.addData("Right error", "%.0f", commandedVelocity - shooterRight.getVelocity());
                telemetry.addData("Intake power", "%.2f", intakePower);
                telemetry.addData("PIDF", "P %.4f | I %.5f | D %.0f | F %.1f", P, I, D, F);
                telemetry.update();
            }
        } finally {
            /* Stop every motor when the OpMode ends. */
            for (DcMotor motor : driveMotors) motor.setPower(0);
            intake.setPower(0);
            shooterLeft.setVelocity(0);
            shooterRight.setVelocity(0);
            shooterLeft.setPower(0);
            shooterRight.setPower(0);
        }
    }

    private static double deadband(double value) {
        return Math.abs(value) < DRIVE_DEADBAND ? 0 : value;
    }
}
