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
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_GOAL_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_GOAL_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_BURST_OFFSET;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_HYSTERESIS;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_ON_POWER;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_TARGET_VELOCITY;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_HEADING_DEG;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_REVERSE_POWER;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_SHOOT_POWER;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TURRET_CENTER;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TURRET_MAX;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TURRET_MIN;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TURRET_STEP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.FlowerFlap;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Intake;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.LimelightVision;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Shooter;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Transfer;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.VisionTurretAim;

/**
 * Shared competition teleop. Alliance is fixed by subclasses
 * ({@link BIOBUZZ_TeleOp_Red} / {@link BIOBUZZ_TeleOp_Blue}).
 *
 * Y toggles Limelight AUTO AIM. A/X/B set turret presets; Dpad nudges.
 * Hive lost → odometry fallback while auto-aim is on.
 */
public abstract class BIOBUZZ_TeleOp extends OpMode {

    /** Fixed at OpMode select — not toggled in-match. */
    protected abstract boolean isRedAlliance();

    private Follower follower;
    private Drivetrain drive;
    private Intake intake;
    private Transfer transfer;
    private FlowerFlap flowerFlap;
    private Shooter shooter;
    private Turret turret;
    private LimelightVision vision;
    private VisionTurretAim hiveAim;

    private boolean lastShooterToggle = false;
    private boolean lastFlowerFlap = false;
    private boolean lastResetPose = false;
    private boolean lastAutoAimToggle = false;
    private boolean lastTurretA = false;
    private boolean lastTurretX = false;
    private boolean lastTurretB = false;
    private boolean redAlliance;
    /** true = Limelight AUTO AIM (default), false = manual (A/X/B/Dpad). */
    private boolean autoAim = true;

    private boolean feeding = false;
    private double transferPower = 0.0;

    @Override
    public void init() {
        redAlliance = isRedAlliance();

        Scheduler.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);

        vision = new LimelightVision(hardwareMap, redAlliance);
        hiveAim = new VisionTurretAim(vision, turret);
        hiveAim.setFollower(follower);
        applyAim();

        telemetry.addLine("BIOBUZZ TELEOP " + (redAlliance ? "RED" : "BLUE") + " (PS5)");
        telemetry.addLine("Shooter ON at play | Turret starts in AUTO AIM (hive)");
        addControls();
        telemetry.addData("Bang-bang", "tgt %.0f  hyst ±%.0f  on %.2f  burst %.0f  feed %.2f",
                SHOOTER_TARGET_VELOCITY, SHOOTER_HYSTERESIS, SHOOTER_ON_POWER,
                SHOOTER_BURST_OFFSET, TRANSFER_SHOOT_POWER);
        telemetry.update();
    }

    private void applyAim() {
        hiveAim.setActive(autoAim);
    }

    @Override
    public void start() {
        shooter.setRunning(true);
        vision.start();
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
                hiveAim.senseCommand(),
                hiveAim.adjustCommand(),
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
        vision.stop();
        intake.stop();
        transfer.stop();
        shooter.stop();
        drive.stop();
    }

    /** R1 feeds transfer immediately while held; otherwise reverses with intake. */
    private void updateFeed() {
        boolean intakeActive = intake.isActive(gamepad1.right_trigger, gamepad1.left_trigger);
        feeding = gamepad1.right_bumper;

        shooter.setFeeding(feeding && shooter.isRunning());

        if (feeding) {
            transferPower = TRANSFER_SHOOT_POWER;
        } else if (intakeActive) {
            transferPower = -TRANSFER_REVERSE_POWER;
        } else {
            transferPower = 0.0;
        }
        transfer.setPower(transferPower);
    }

    private void handleButtons() {
        // Start / Options = shooter toggle (ON at play)
        boolean shooterToggle = gamepad1.start || gamepad1.options;
        if (shooterToggle && !lastShooterToggle) {
            shooter.toggle();
        }
        lastShooterToggle = shooterToggle;

        boolean flowerToggle = gamepad1.left_bumper;
        if (flowerToggle && !lastFlowerFlap) {
            flowerFlap.toggle();
        }
        lastFlowerFlap = flowerToggle;

        // Y = auto aim toggle
        if (gamepad1.y && !lastAutoAimToggle) {
            autoAim = !autoAim;
            turret.clearTrim();
            applyAim();
        }
        lastAutoAimToggle = gamepad1.y;

        // A / X / B = turret presets (manual; turns auto aim off)
        if (gamepad1.a && !lastTurretA) {
            setManualTurret(TURRET_CENTER);
        }
        if (gamepad1.x && !lastTurretX) {
            setManualTurret(TURRET_MIN);
        }
        if (gamepad1.b && !lastTurretB) {
            setManualTurret(TURRET_MAX);
        }
        lastTurretA = gamepad1.a;
        lastTurretX = gamepad1.x;
        lastTurretB = gamepad1.b;

        boolean resetPose = gamepad1.back || gamepad1.share;
        if (resetPose && !lastResetPose) {
            resetStartPose();
            turret.clearTrim();
        }
        lastResetPose = resetPose;
    }

    private void setManualTurret(double position) {
        autoAim = false;
        applyAim();
        turret.clearTrim();
        turret.setPosition(position);
    }

    private void updateTurretAim() {
        // Dpad L/R = nudge turret position (manual; turns auto aim off)
        if (gamepad1.dpad_right || gamepad1.dpad_left) {
            if (autoAim) {
                autoAim = false;
                applyAim();
                turret.clearTrim();
            }
            if (gamepad1.dpad_right) {
                turret.nudge(TURRET_STEP);
            }
            if (gamepad1.dpad_left) {
                turret.nudge(-TURRET_STEP);
            }
        }
    }

    private void applyAlliance() {
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_X =
                redAlliance ? RED_GOAL_X : BLUE_GOAL_X;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_Y =
                redAlliance ? RED_GOAL_Y : BLUE_GOAL_Y;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_X =
                redAlliance ? RED_START_X : BLUE_START_X;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_Y =
                redAlliance ? RED_START_Y : BLUE_START_Y;
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
        return shooter.atSpeed() ? "READY" : "SPINNING UP";
    }

    private void showTelemetry() {
        Pose pose = follower.pose();

        telemetry.addLine("===== BIOBUZZ TELEOP " + (redAlliance ? "RED" : "BLUE") + " =====");
        telemetry.addData("Shooter", shooterStatus());
        telemetry.addData("Speed / target", "%.0f / %.0f",
                shooter.getVelocity(), shooter.getTargetVelocity());
        telemetry.addData("Shot speeds", shooter.shotSpeedsText());
        telemetry.addData("Shot spread", "%.1f %% (first to last %.1f %%)",
                shooter.getShotSpreadPercent(), shooter.getShotDropPercent());
        telemetry.addData("Recovery s", shooter.shotRecoveryText());
        telemetry.addData("Battery", "%.2f V", shooter.getBatteryVoltage());
        telemetry.addData("Shooter power", "%.2f (%s)", shooter.getLastPower(), shooter.getMode());
        telemetry.addData("Bang-bang", "tgt %.0f  hyst ±%.0f  burst %.0f  feed %.2f",
                SHOOTER_TARGET_VELOCITY, SHOOTER_HYSTERESIS, SHOOTER_BURST_OFFSET,
                TRANSFER_SHOOT_POWER);
        telemetry.addData("Transfer", "%.2f", transferPower);
        telemetry.addData("Loop", "%.0f ms", shooter.getLoopMs());
        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Turret mode", autoAim ? "AUTO AIM (hive)" : "MANUAL");
        telemetry.addData("Goal", "%.0f / %.0f", GOAL_X, GOAL_Y);
        telemetry.addData("Flower flap", "%.2f", flowerFlap.getPosition());
        telemetry.addData("Turret pos / deg", "%.3f / %.1f",
                turret.getPosition(), turret.getAimDegrees());
        telemetry.addData("Pose X/Y/H", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Cell tags 0 / 1", "%d / %d",
                vision.getVisibleTagCount(0), vision.getVisibleTagCount(1));
        hiveAim.addTelemetry(telemetry);
        addControls();
        telemetry.update();
    }

    private void addControls() {
        telemetry.addLine("----- CONTROLS -----");
        telemetry.addLine("Sticks = drive | RT/LT = intake | RB = transfer | LB = flap");
        telemetry.addLine("Start = shooter | Y = auto aim");
        telemetry.addLine("A = 0.5 | X = 0.2 | B = 0.8 | Dpad L/R = nudge");
    }
}
