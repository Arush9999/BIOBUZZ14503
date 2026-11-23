package org.firstinspires.ftc.teamcode.pedro.teleop;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_GOAL_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_GOAL_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_START_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_START_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_TRIGGER_DEADZONE;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_GOAL_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_GOAL_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_HEADING_DEG;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_REVERSE_POWER;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TURRET_STEP;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.constants.FlywheelConstants;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.FlowerFlap;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Flywheel;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Intake;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Transfer;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;

/**
 * Same driving and controls as BIOBUZZ TeleOp V1, but with the new {@link Flywheel}:
 * - battery-compensated speed control (same wheel speed from 12 V to 14 V)
 * - RB only starts feeding once the wheel is at speed (hold A to feed anyway)
 * - transfer feeds at a fixed voltage instead of a fixed power
 * - telemetry shows the wheel speed at each shot, so no graph is needed
 *
 * Dpad is different from V1: U/D = feed volts, L/R = target speed.
 */
@TeleOp(name = "BIOBUZZ TeleOp V2 (Flywheel)", group = "BIOBUZZ")
public class BIOBUZZ_TeleOp_V2 extends OpMode {

    private static final double FEED_VOLTS_STEP = 0.5;
    private static final double TARGET_STEP = 25.0;
    private static final double BURST_OFFSET_STEP = 10.0;

    private Follower follower;
    private Drivetrain drive;
    private Intake intake;
    private Transfer transfer;
    private FlowerFlap flowerFlap;
    private Flywheel flywheel;
    private Turret turret;

    private boolean lastShooterToggle = false;
    private boolean lastFlowerFlap = false;
    private boolean lastResetPose = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastAllianceToggle = false;
    private boolean lastAutoAimToggle = false;
    private boolean lastModeToggle = false;
    private boolean lastBurstOffsetUp = false;
    private boolean lastBurstOffsetDown = false;
    private boolean redAlliance = true;
    private boolean autoAim = false;
    /** GP2 Mode: start forward; Mode toggles to auto-aim, Mode again back to forward. */
    private boolean forwardOnly = true;

    // Feed state
    private boolean feeding = false;
    private boolean waitingForSpeed = false;
    private long feedRequestNs = 0L;
    private double transferPower = 0.0;

    @Override
    public void init() {
        Scheduler.reset();

        // One bulk read per hub per loop keeps the loop time short and steady.
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.create(hardwareMap);
        applyAlliance();
        resetStartPose();

        drive = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        flowerFlap = new FlowerFlap(hardwareMap);
        flywheel = new Flywheel(hardwareMap);
        flywheel.setTargetVelocity(FlywheelConstants.TARGET_VELOCITY);
        turret = new Turret(hardwareMap);
        // Init: auto-aim off, turret fixed at 0.5.
        turret.clearTrim();
        turret.setPosition(0.5);

        telemetry.addLine("BIOBUZZ TELEOP V2 (new flywheel)");
        telemetry.addLine("Sticks = drive");
        telemetry.addLine("RT/LT = intake in/out");
        telemetry.addLine("RB = shoot (waits for wheel speed) | A = feed anyway");
        telemetry.addLine("LB = flower flap | START = shooter toggle (ON at play)");
        telemetry.addLine("Dpad U/D = feed volts | L/R = target speed");
        telemetry.addLine("GP1 B = +burst offset | GP2 B = -burst offset");
        telemetry.addLine("GP2 Dpad L/R = turret trim | Mode = auto-aim / forward");
        telemetry.addLine("X = red/blue | Y = auto aim | BACK = reset pose");
        telemetry.update();
    }

    @Override
    public void start() {
        flywheel.setRunning(true);
        schedule(parallel(
                drive.teleOpCommand(
                        () -> -gamepad1.left_stick_y,
                        () -> gamepad1.left_stick_x,
                        () -> gamepad1.right_stick_x
                ),
                intake.teleOpCommand(
                        () -> gamepad1.right_trigger,
                        () -> gamepad1.left_trigger
                ),
                infinite(this::updateFeed),
                infinite(this::updateTurretAim),
                infinite(this::handleButtons),
                infinite(this::showTelemetry)
        ));
    }

    @Override
    public void loop() {
        follower.update();
        flywheel.update(); // speed control first, so the feed logic sees this loop's speed
        Scheduler.execute();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        intake.stop();
        transfer.stop();
        flywheel.stop();
        drive.stop();
    }

    /**
     * RB starts a burst only when the wheel is at speed. Once started it keeps feeding while RB
     * is held (the transfer wheel cannot hold balls back), at a battery-compensated voltage.
     */
    private void updateFeed() {
        boolean intakeActive = gamepad1.right_trigger > INTAKE_TRIGGER_DEADZONE
                || gamepad1.left_trigger > INTAKE_TRIGGER_DEADZONE;
        boolean wantFeed = gamepad1.right_bumper;
        long now = System.nanoTime();

        if (!wantFeed) {
            feeding = false;
            waitingForSpeed = false;
        } else if (!feeding) {
            if (!waitingForSpeed) {
                waitingForSpeed = true;
                feedRequestNs = now;
            }
            double waited = (now - feedRequestNs) / 1.0e9;
            boolean gateOpen = !flywheel.isRunning()          // shooter off: RB works like V1
                    || flywheel.atSpeed()
                    || gamepad1.a                             // driver override
                    || waited >= FlywheelConstants.FEED_GATE_TIMEOUT_SEC;
            if (gateOpen) {
                feeding = true;
                waitingForSpeed = false;
            }
        }

        flywheel.setFeeding(feeding && flywheel.isRunning());

        if (feeding) {
            transferPower = Range.clip(
                    FlywheelConstants.FEED_VOLTS
                            / Math.max(flywheel.getBatteryVoltage(), FlywheelConstants.MIN_VOLTAGE),
                    0.0, 1.0);
        } else if (intakeActive) {
            transferPower = -TRANSFER_REVERSE_POWER;
        } else {
            transferPower = 0.0;
        }
        transfer.setPower(transferPower);
    }

    private void handleButtons() {
        boolean shooterToggle = gamepad1.start;
        if (shooterToggle && !lastShooterToggle) {
            flywheel.toggle();
        }
        lastShooterToggle = shooterToggle;

        boolean flowerToggle = gamepad1.left_bumper;
        if (flowerToggle && !lastFlowerFlap) {
            flowerFlap.toggle();
        }
        lastFlowerFlap = flowerToggle;

        if (gamepad1.x && !lastAllianceToggle) {
            redAlliance = !redAlliance;
            applyAlliance();
            resetStartPose();
            turret.clearTrim();
            if (forwardOnly) {
                turret.holdForward();
            } else if (autoAim) {
                turret.update(follower);
            }
        }
        lastAllianceToggle = gamepad1.x;

        if (gamepad1.y && !lastAutoAimToggle) {
            autoAim = !autoAim;
        }
        lastAutoAimToggle = gamepad1.y;

        // Dpad U/D = feed volts (slower feed = more time for the wheel to recover).
        if (gamepad1.dpad_up && !lastDpadUp) {
            FlywheelConstants.FEED_VOLTS = Range.clip(FlywheelConstants.FEED_VOLTS + FEED_VOLTS_STEP, 3.0, 13.0);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            FlywheelConstants.FEED_VOLTS = Range.clip(FlywheelConstants.FEED_VOLTS - FEED_VOLTS_STEP, 3.0, 13.0);
        }
        // Dpad L/R = target speed (for building the distance table).
        if (gamepad1.dpad_right && !lastDpadRight) {
            FlywheelConstants.TARGET_VELOCITY += TARGET_STEP;
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            FlywheelConstants.TARGET_VELOCITY = Math.max(0, FlywheelConstants.TARGET_VELOCITY - TARGET_STEP);
        }
        // Burst offset: raise aim during a feed so later balls land closer to the real target.
        if (gamepad1.b && !lastBurstOffsetUp) {
            FlywheelConstants.BURST_OFFSET =
                    Range.clip(FlywheelConstants.BURST_OFFSET + BURST_OFFSET_STEP, 0.0, 150.0);
        }
        if (gamepad2.b && !lastBurstOffsetDown) {
            FlywheelConstants.BURST_OFFSET =
                    Range.clip(FlywheelConstants.BURST_OFFSET - BURST_OFFSET_STEP, 0.0, 150.0);
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastBurstOffsetUp = gamepad1.b;
        lastBurstOffsetDown = gamepad2.b;
        // Also picks up live edits from FTC Dashboard.
        flywheel.setTargetVelocity(FlywheelConstants.TARGET_VELOCITY);

        boolean resetPose = gamepad1.back;
        if (resetPose && !lastResetPose) {
            resetStartPose();
            turret.clearTrim();
            if (forwardOnly) {
                turret.holdForward();
            } else if (autoAim) {
                turret.update(follower);
            }
        }
        lastResetPose = resetPose;

        // GP2 Mode (guide): forward (default) <-> auto-aim
        boolean modePressed = gamepad2.guide;
        if (modePressed && !lastModeToggle) {
            forwardOnly = !forwardOnly;
            turret.clearTrim();
            if (forwardOnly) {
                autoAim = false;
                turret.holdForward(); // 0.5 (trim cleared above)
            } else {
                autoAim = true;
                turret.update(follower);
            }
        }
        lastModeToggle = modePressed;
    }

    /** GP2 Dpad L/R = trim in BOTH forward-fixed (0.5+trim) and auto-aim. */
    private void updateTurretAim() {
        if (gamepad2.dpad_right) {
            turret.nudgeTrim(TURRET_STEP);
        }
        if (gamepad2.dpad_left) {
            turret.nudgeTrim(-TURRET_STEP);
        }

        if (forwardOnly) {
            // Always re-apply 0.5 + trim so dpad moves the servo in fixed mode.
            turret.holdForward();
        } else if (autoAim) {
            turret.update(follower);
        }
    }

    private void applyAlliance() {
        if (redAlliance) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_X = RED_GOAL_X;
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_Y = RED_GOAL_Y;
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_X = RED_START_X;
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_Y = RED_START_Y;
        } else {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_X = BLUE_GOAL_X;
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_Y = BLUE_GOAL_Y;
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_X = BLUE_START_X;
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_Y = BLUE_START_Y;
        }
    }

    private void resetStartPose() {
        follower.setPose(PoseFactory.degrees().of(START_X, START_Y, START_HEADING_DEG));
    }

    private String shooterStatus() {
        if (!flywheel.isRunning()) {
            return "OFF";
        }
        if (!flywheel.canHoldTarget()) {
            return "CAN'T HOLD SPEED - battery low or target too high";
        }
        if (feeding) {
            return "FEEDING";
        }
        if (waitingForSpeed) {
            return "WAITING FOR SPEED";
        }
        return flywheel.atSpeed() ? "READY" : "SPINNING UP";
    }

    private void showTelemetry() {
        Pose pose = follower.pose();

        telemetry.addLine("===== BIOBUZZ TELEOP V2 =====");
        telemetry.addData("Shooter", shooterStatus());
        telemetry.addData("Speed / target", "%.0f / %.0f", flywheel.getVelocity(), flywheel.getTargetVelocity());
        telemetry.addData("Shot speeds", flywheel.shotSpeedsText());
        telemetry.addData("Shot spread", "%.1f %% (first to last %.1f %%)",
                flywheel.getShotSpreadPercent(), flywheel.getShotDropPercent());
        telemetry.addData("Recovery s", flywheel.shotRecoveryText());
        telemetry.addData("Battery", "%.2f V", flywheel.getBatteryVoltage());
        telemetry.addData("Shooter power", "%.2f (%s, hold needs %.2f)",
                flywheel.getLastPower(), flywheel.getMode(), flywheel.holdPowerNeeded());
        telemetry.addData("Feed volts", "%.1f (transfer %.2f)", FlywheelConstants.FEED_VOLTS, transferPower);
        telemetry.addData("Burst offset", "%.0f ticks/s", FlywheelConstants.BURST_OFFSET);
        telemetry.addData("Loop", "%.0f ms", flywheel.getLoopMs());
        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Turret mode", forwardOnly ? "FORWARD" : (autoAim ? "AUTO AIM" : "OFF"));
        telemetry.addData("Goal", "%.0f / %.0f", GOAL_X, GOAL_Y);
        telemetry.addData("Flower flap", "%.2f", flowerFlap.getPosition());
        telemetry.addData("Turret pos / trim / deg", "%.3f / %.3f / %.1f",
                turret.getPosition(), turret.getTrimPos(), turret.getAimDegrees());
        telemetry.addData("Pose X/Y/H", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addLine("GP1: sticks drive | RT/LT intake | RB shoot | A feed anyway | LB flap");
        telemetry.addLine("GP1 START shooter | X alliance | Y aim | BACK reset");
        telemetry.addLine("GP1 Dpad U/D feed volts | L/R target | B +burst offset");
        telemetry.addLine("GP2 B -burst offset | Dpad L/R trim | Mode = forward/autoaim");
        telemetry.update();
    }
}
