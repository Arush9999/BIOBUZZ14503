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
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.FEED_GATE_TIMEOUT_SEC;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_REVERSE_POWER;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TURRET_STEP;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.FlowerFlap;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Shooter;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Intake;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Transfer;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;

/**
 * BIOBUZZ competition teleop with bang-bang {@link Shooter}.
 * Bang-bang / feed numbers are hardcoded here (not driver-tunable).
 * Retune with Shooter_BangBang tester, then update the constants below.
 *
 * PS5 layout (Driver Station: Options + Cross to link).
 */
@TeleOp(name = "BIOBUZZ TeleOp", group = "BIOBUZZ")
public class BIOBUZZ_TeleOp extends OpMode {

    // ---- Bang-bang / feed (hardcoded; update after tester tuning) ----
    private static final double TARGET_VELOCITY = 1825; // ticks/s
    private static final double HYSTERESIS = 5;         // ticks/s each side
    private static final double ON_POWER = 1.0;
    private static final double BURST_OFFSET = 40.0;    // extra ticks/s while feeding
    private static final double FEED_POWER = 1.0;

    private Follower follower;
    private Drivetrain drive;
    private Intake intake;
    private Transfer transfer;
    private FlowerFlap flowerFlap;
    private Shooter shooter;
    private Turret turret;

    private boolean lastShooterToggle = false;
    private boolean lastFlowerFlap = false;
    private boolean lastResetPose = false;
    private boolean lastAllianceToggle = false;
    private boolean lastAutoAimToggle = false;
    private boolean lastModeToggle = false;
    private boolean redAlliance = true;
    private boolean autoAim = false;
    /** GP2 PS: start forward; toggles to auto-aim, again back to forward. */
    private boolean forwardOnly = true;

    // Feed state
    private boolean feeding = false;
    private boolean waitingForSpeed = false;
    private long feedRequestNs = 0L;
    private double transferPower = 0.0;

    @Override
    public void init() {
        Scheduler.reset();

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
        applyBangBangDefaults();
        shooter = new Shooter(hardwareMap);
        shooter.setTargetVelocity(TARGET_VELOCITY);
        turret = new Turret(hardwareMap);
        turret.clearTrim();
        turret.setPosition(0.5);

        telemetry.addLine("BIOBUZZ TELEOP (PS5)");
        telemetry.addLine("Sticks = drive");
        telemetry.addLine("R2/L2 = intake in/out");
        telemetry.addLine("R1 = shoot (waits for wheel speed) | Cross = feed anyway");
        telemetry.addLine("L1 = flower flap | Options = shooter toggle (ON at play)");
        telemetry.addLine("GP2 Dpad L/R = turret trim | PS = auto-aim / forward");
        telemetry.addLine("Square = red/blue | Triangle = auto aim | Share = reset pose");
        telemetry.addData("Bang-bang", "tgt %.0f  hyst ±%.0f  on %.2f  burst %.0f  feed %.2f",
                TARGET_VELOCITY, HYSTERESIS, ON_POWER, BURST_OFFSET, FEED_POWER);
        telemetry.update();
    }

    /** Push hardcoded values into Constants so {@link Shooter} uses them. */
    private void applyBangBangDefaults() {
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_TARGET_VELOCITY =
                TARGET_VELOCITY;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_HYSTERESIS =
                HYSTERESIS;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_ON_POWER =
                ON_POWER;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_BURST_OFFSET =
                BURST_OFFSET;
    }

    @Override
    public void start() {
        shooter.setRunning(true);
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
        shooter.update();
        Scheduler.execute();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        intake.stop();
        transfer.stop();
        shooter.stop();
        drive.stop();
    }

    /**
     * R1 starts a burst only when the wheel is at speed. Once started it keeps feeding while R1
     * is held, at full transfer power.
     */
    private void updateFeed() {
        boolean intakeActive = gamepad1.right_trigger > INTAKE_TRIGGER_DEADZONE
                || gamepad1.left_trigger > INTAKE_TRIGGER_DEADZONE;
        boolean wantFeed = gamepad1.right_bumper; // R1
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
            boolean gateOpen = !shooter.isRunning()
                    || shooter.atSpeed()
                    || gamepad1.cross
                    || waited >= FEED_GATE_TIMEOUT_SEC;
            if (gateOpen) {
                feeding = true;
                waitingForSpeed = false;
            }
        }

        shooter.setFeeding(feeding && shooter.isRunning());

        if (feeding) {
            transferPower = FEED_POWER;
        } else if (intakeActive) {
            transferPower = -TRANSFER_REVERSE_POWER;
        } else {
            transferPower = 0.0;
        }
        transfer.setPower(transferPower);
    }

    private void handleButtons() {
        boolean shooterToggle = gamepad1.options;
        if (shooterToggle && !lastShooterToggle) {
            shooter.toggle();
        }
        lastShooterToggle = shooterToggle;

        boolean flowerToggle = gamepad1.left_bumper; // L1
        if (flowerToggle && !lastFlowerFlap) {
            flowerFlap.toggle();
        }
        lastFlowerFlap = flowerToggle;

        if (gamepad1.square && !lastAllianceToggle) {
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
        lastAllianceToggle = gamepad1.square;

        if (gamepad1.triangle && !lastAutoAimToggle) {
            autoAim = !autoAim;
        }
        lastAutoAimToggle = gamepad1.triangle;

        boolean resetPose = gamepad1.share;
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

        boolean modePressed = gamepad2.ps;
        if (modePressed && !lastModeToggle) {
            forwardOnly = !forwardOnly;
            turret.clearTrim();
            if (forwardOnly) {
                autoAim = false;
                turret.holdForward();
            } else {
                autoAim = true;
                turret.update(follower);
            }
        }
        lastModeToggle = modePressed;
    }

    private void updateTurretAim() {
        if (gamepad2.dpad_right) {
            turret.nudgeTrim(TURRET_STEP);
        }
        if (gamepad2.dpad_left) {
            turret.nudgeTrim(-TURRET_STEP);
        }

        if (forwardOnly) {
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
        if (!shooter.isRunning()) {
            return "OFF";
        }
        if (feeding) {
            return "FEEDING";
        }
        if (waitingForSpeed) {
            return "WAITING FOR SPEED";
        }
        return shooter.atSpeed() ? "READY" : "SPINNING UP";
    }

    private void showTelemetry() {
        Pose pose = follower.pose();

        telemetry.addLine("===== BIOBUZZ TELEOP =====");
        telemetry.addData("Shooter", shooterStatus());
        telemetry.addData("Speed / target", "%.0f / %.0f", shooter.getVelocity(), shooter.getTargetVelocity());
        telemetry.addData("Shot speeds", shooter.shotSpeedsText());
        telemetry.addData("Shot spread", "%.1f %% (first to last %.1f %%)",
                shooter.getShotSpreadPercent(), shooter.getShotDropPercent());
        telemetry.addData("Recovery s", shooter.shotRecoveryText());
        telemetry.addData("Battery", "%.2f V", shooter.getBatteryVoltage());
        telemetry.addData("Shooter power", "%.2f (%s)", shooter.getLastPower(), shooter.getMode());
        telemetry.addData("Bang-bang", "tgt %.0f  hyst ±%.0f  burst %.0f  feed %.2f",
                TARGET_VELOCITY, HYSTERESIS, BURST_OFFSET, FEED_POWER);
        telemetry.addData("Transfer", "%.2f", transferPower);
        telemetry.addData("Loop", "%.0f ms", shooter.getLoopMs());
        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Turret mode", forwardOnly ? "FORWARD" : (autoAim ? "AUTO AIM" : "OFF"));
        telemetry.addData("Goal", "%.0f / %.0f", GOAL_X, GOAL_Y);
        telemetry.addData("Flower flap", "%.2f", flowerFlap.getPosition());
        telemetry.addData("Turret pos / trim / deg", "%.3f / %.3f / %.1f",
                turret.getPosition(), turret.getTrimPos(), turret.getAimDegrees());
        telemetry.addData("Pose X/Y/H", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addLine("GP1: sticks drive | R2/L2 intake | R1 shoot | Cross feed anyway | L1 flap");
        telemetry.addLine("GP1 Options shooter | Square alliance | Triangle aim | Share reset");
        telemetry.addLine("GP2 Dpad L/R trim | PS = forward/autoaim");
        telemetry.update();
    }
}
