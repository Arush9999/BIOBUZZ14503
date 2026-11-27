package org.firstinspires.ftc.teamcode.pedro.teleop;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_GOAL_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_GOAL_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_START_OFFSET_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_GOAL_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_GOAL_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_FAR_HEADING_DEG;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_FAR_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_FAR_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_NEAR_HEADING_DEG;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_NEAR_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_NEAR_Y;
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
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.TurretAutoAim;

/**
 * Shared competition teleop. Alliance is fixed by subclasses
 * ({@link BIOBUZZ_TeleOp_Red} / {@link BIOBUZZ_TeleOp_Blue}).
 *
 * Y toggles hybrid {@link TurretAutoAim} (odometry + Kalman bias) ↔ FORWARD (0.5).
 * A/X/B set turret presets; Dpad nudges.
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
    private TurretAutoAim turretAim;

    private boolean lastShooterToggle = false;
    private boolean lastFlowerFlap = false;
    private boolean lastResetPose = false;
    private boolean lastAutoAimToggle = false;
    private boolean lastTurretA = false;
    private boolean lastTurretX = false;
    private boolean lastTurretB = false;
    private boolean redAlliance;
    /** true = near wall (y=8 @ 90°), false = far wall (y=136 @ 270°). */
    private boolean startNear = true;
    /** true = Limelight AUTO AIM (default), false = FORWARD or manual preset. */
    private boolean autoAim = true;
    /** True after A/X/B or Dpad — don't force holdForward over a chosen pose. */
    private boolean manualTurret = false;

    private boolean feeding = false;
    private double transferPower = 0.0;

    @Override
    public void init() {
        redAlliance = isRedAlliance();

        Scheduler.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(100);

        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.create(hardwareMap);
        applyStartPoseConstants();
        resetStartPose();

        drive = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        flowerFlap = new FlowerFlap(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        turret.clearTrim();
        // Point at goal from the selected start pose before hybrid takes over.
        turret.setPosition(turret.odometryTargetPosition(follower));

        vision = new LimelightVision(hardwareMap, redAlliance);
        turretAim = new TurretAutoAim(vision, turret);
        turretAim.setFollower(follower); // hybrid: odometry + Kalman camera bias
        applyAim();

        telemetry.addLine("BIOBUZZ TELEOP " + (redAlliance ? "RED" : "BLUE") + " (PS5)");
        telemetry.addLine("Init: Dpad Up = near (y=8@90) | Down = far (y=136@270)");
        telemetry.addLine("Shooter ON at play | Turret starts in AUTO AIM (hive)");
        addControls();
        telemetry.addData("Bang-bang", "tgt %.0f  hyst ±%.0f  on %.2f  burst %.0f  feed %.2f",
                SHOOTER_TARGET_VELOCITY, SHOOTER_HYSTERESIS, SHOOTER_ON_POWER,
                SHOOTER_BURST_OFFSET, TRANSFER_SHOOT_POWER);
        showInitTelemetry();
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_up && !startNear) {
            startNear = true;
            applyStartPoseConstants();
            resetStartPose();
        }
        if (gamepad1.dpad_down && startNear) {
            startNear = false;
            applyStartPoseConstants();
            resetStartPose();
        }
        showInitTelemetry();
    }

    private void showInitTelemetry() {
        telemetry.addData("Start side", startNear ? "NEAR" : "FAR");
        telemetry.addData("Start pose", "%.0f, %.0f @ %.0f°",
                START_X, START_Y, START_HEADING_DEG);
        telemetry.addData("Turret @ start", "%.3f",
                turret != null ? turret.getPosition() : Double.NaN);
        telemetry.update();
    }

    private void applyAim() {
        turretAim.setActive(autoAim);
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
                turretAim.senseCommand(),
                turretAim.adjustCommand(),
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

        // Y = auto aim ↔ forward (0.5), same as HiveTeleOp V2
        if (gamepad1.y && !lastAutoAimToggle) {
            autoAim = !autoAim;
            manualTurret = false;
            turret.clearTrim();
            if (!autoAim) {
                turret.holdForward();
            }
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
        manualTurret = true;
        applyAim();
        turret.clearTrim();
        turret.setPosition(position);
    }

    private void updateTurretAim() {
        // Dpad L/R = nudge (leaves AUTO AIM for manual control)
        if (gamepad1.dpad_right || gamepad1.dpad_left) {
            if (autoAim) {
                autoAim = false;
                applyAim();
                turret.clearTrim();
            }
            manualTurret = true;
            if (gamepad1.dpad_right) {
                turret.nudge(TURRET_STEP);
            }
            if (gamepad1.dpad_left) {
                turret.nudge(-TURRET_STEP);
            }
            return;
        }

        // FORWARD (Y off, no preset): hold 0.5. AUTO AIM: TurretAutoAim owns the turret.
        if (!autoAim && !manualTurret) {
            turret.holdForward();
        }
    }

    private void applyStartPoseConstants() {
        double sx = startNear ? RED_START_NEAR_X : RED_START_FAR_X;
        double sy = startNear ? RED_START_NEAR_Y : RED_START_FAR_Y;
        double sh = startNear ? RED_START_NEAR_HEADING_DEG : RED_START_FAR_HEADING_DEG;
        if (!redAlliance) {
            sx += BLUE_START_OFFSET_X;
        }

        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_X =
                redAlliance ? RED_GOAL_X : BLUE_GOAL_X;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_Y =
                redAlliance ? RED_GOAL_Y : BLUE_GOAL_Y;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_X = sx;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_Y = sy;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_HEADING_DEG = sh;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_X =
                startNear ? RED_START_NEAR_X : RED_START_FAR_X;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_Y =
                startNear ? RED_START_NEAR_Y : RED_START_FAR_Y;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_START_X =
                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_X
                        + BLUE_START_OFFSET_X;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_START_Y =
                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_Y;
    }

    private void resetStartPose() {
        follower.setPose(PoseFactory.degrees().of(START_X, START_Y, START_HEADING_DEG));
        if (turret != null) {
            turret.clearTrim();
            turret.setPosition(turret.odometryTargetPosition(follower));
            if (turretAim != null) {
                turretAim.reset();
            }
        }
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

        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Start", "%s  %.0f, %.0f @ %.0f°",
                startNear ? "NEAR" : "FAR", START_X, START_Y, START_HEADING_DEG);
        telemetry.addData("Shooter", shooterStatus());
        telemetry.addData("Speed / tgt", "%.0f / %.0f",
                shooter.getVelocity(), shooter.getTargetVelocity());
        telemetry.addData("Power", "%.2f (%s)", shooter.getLastPower(), shooter.getMode());
        telemetry.addData("Transfer", "%.2f", transferPower);
        telemetry.addData("Turret", "%s  pos %.3f",
                autoAim ? "AUTO" : "MANUAL", turret.getPosition());
        telemetry.addData("Pose", "%.1f, %.1f @ %.0f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Tags 0/1", "%d / %d",
                vision.getVisibleTagCount(0), vision.getVisibleTagCount(1));
        turretAim.addTelemetry(telemetry);
        telemetry.update();
    }

    private void addControls() {
        telemetry.addLine("----- CONTROLS -----");
        telemetry.addLine("Sticks = drive | RT/LT = intake | RB = transfer | LB = flap");
        telemetry.addLine("Start = shooter | Y = auto aim");
        telemetry.addLine("A = 0.5 | X = 0.2 | B = 0.8 | Dpad L/R = nudge");
    }
}
