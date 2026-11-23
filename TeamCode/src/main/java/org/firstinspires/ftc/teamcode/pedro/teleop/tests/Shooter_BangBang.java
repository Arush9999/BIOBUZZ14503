package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Intake;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Transfer;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;

/**
 * Bang-bang (on/off) flywheel speed loop with hysteresis.
 *
 * Each loop:
 *   velocity &lt; target - HYSTERESIS  -> full ON power
 *   velocity &gt; target + HYSTERESIS  -> coast (0)
 *   inside the band                 -> hold last state (stops chatter)
 *
 * Controls:
 *   START / L3 = toggle shooter
 *   Dpad U/D   = target ± TARGET_STEP
 *   Dpad L/R   = hysteresis ± HYSTERESIS_STEP
 *   RT / LT    = intake in / out
 *   RB         = transfer forward (feed)
 *   Turret held at 0.5 (forward)
 */
@Config
@TeleOp(name = "Shooter_BangBang", group = "Testing")
public class Shooter_BangBang extends OpMode {

    public static double TARGET_VELOCITY = 1800; // ticks/s
    public static double HYSTERESIS = 40;        // ticks/s each side of target
    public static double ON_POWER = 1.0;         // power when below the band
    public static double TARGET_STEP = 25;
    public static double HYSTERESIS_STEP = 5;
    public static double TURRET_HOLD = 0.5;

    private DcMotorEx shooter;
    private Intake intake;
    private Transfer transfer;
    private Turret turret;
    private boolean running = false;
    private boolean lastToggle;
    private boolean lastUp, lastDown, lastLeft, lastRight;
    private boolean bangOn = false; // latched state inside the hysteresis band
    private double lastPower = 0.0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setPower(0);

        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        turret = new Turret(hardwareMap);
        turret.setPosition(TURRET_HOLD);

        telemetry.addLine("SHOOTER BANG-BANG");
        telemetry.addLine("START / L3 = toggle shooter");
        telemetry.addLine("Dpad U/D = target | L/R = hysteresis");
        telemetry.addLine("RT/LT = intake | RB = transfer feed");
        telemetry.addLine("Turret held at 0.5");
        telemetry.update();
    }

    @Override
    public void start() {
        running = true;
        bangOn = true;
    }

    @Override
    public void loop() {
        handleButtons();

        double rt = gamepad1.right_trigger;
        double lt = gamepad1.left_trigger;
        boolean rb = gamepad1.right_bumper;

        intake.run(rt, lt);
        transfer.run(rt, lt, rb);
        turret.setPosition(TURRET_HOLD);

        double velocity = shooter.getVelocity();
        double target = Math.max(0, TARGET_VELOCITY);
        double hyst = Math.max(0, HYSTERESIS);
        double error = target - velocity;

        if (!running || target <= 0) {
            bangOn = false;
            lastPower = 0.0;
            shooter.setPower(0.0);
        } else {
            // Classic bang-bang with hysteresis deadband.
            if (velocity < target - hyst) {
                bangOn = true;
            } else if (velocity > target + hyst) {
                bangOn = false;
            }
            // else: keep bangOn (inside band)

            lastPower = bangOn ? Range.clip(ON_POWER, 0.0, 1.0) : 0.0;
            shooter.setPower(lastPower);
        }

        telemetry.addData("Running", running ? "ON" : "OFF");
        telemetry.addData("Bang state", bangOn ? "FULL" : "COAST");
        telemetry.addData("Target", "%.0f", target);
        telemetry.addData("Velocity", "%.0f", velocity);
        telemetry.addData("Error", "%.0f", error);
        telemetry.addData("Hysteresis ±", "%.0f", hyst);
        telemetry.addData("Power", "%.2f", lastPower);
        telemetry.addData("Intake", "%.2f", intake.getIntake().getPower());
        telemetry.addData("Transfer", "%.2f", transfer.getTransfer().getPower());
        telemetry.addData("Turret", "%.2f", turret.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        running = false;
        bangOn = false;
        lastPower = 0.0;
        if (shooter != null) {
            shooter.setPower(0.0);
        }
        if (intake != null) {
            intake.stop();
        }
        if (transfer != null) {
            transfer.stop();
        }
    }

    private void handleButtons() {
        boolean toggle = gamepad1.start || gamepad1.left_stick_button;
        if (toggle && !lastToggle) {
            running = !running;
            if (running) {
                bangOn = true; // spin up immediately when enabled
            }
        }
        lastToggle = toggle;

        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;

        if (up && !lastUp) {
            TARGET_VELOCITY += TARGET_STEP;
        }
        if (down && !lastDown) {
            TARGET_VELOCITY = Math.max(0, TARGET_VELOCITY - TARGET_STEP);
        }
        if (right && !lastRight) {
            HYSTERESIS += HYSTERESIS_STEP;
        }
        if (left && !lastLeft) {
            HYSTERESIS = Math.max(0, HYSTERESIS - HYSTERESIS_STEP);
        }

        lastUp = up;
        lastDown = down;
        lastLeft = left;
        lastRight = right;
    }
}
