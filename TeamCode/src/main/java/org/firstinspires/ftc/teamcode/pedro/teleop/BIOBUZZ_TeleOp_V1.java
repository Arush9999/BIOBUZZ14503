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
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_D;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_F;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_I;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_P;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_HEADING_DEG;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TURRET_STEP;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.FlowerFlap;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Intake;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Shooter;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Transfer;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;

@TeleOp(name = "BIOBUZZ TeleOp V1", group = "BIOBUZZ")
public class BIOBUZZ_TeleOp_V1 extends OpMode {

    private static final double P_INCREMENT = 0.05;
    private static final double F_INCREMENT = 0.5;

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
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastAllianceToggle = false;
    private boolean lastAutoAimToggle = false;
    private boolean lastModeToggle = false;
    private boolean redAlliance = true;
    private boolean autoAim = false;
    /** GP2 Mode: start forward; Mode toggles to auto-aim, Mode again back to forward. */
    private boolean forwardOnly = true;

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        applyAlliance();
        resetStartPose();

        drive = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        flowerFlap = new FlowerFlap(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        // Init: auto-aim off, turret fixed at 0.5.
        turret.clearTrim();
        turret.setPosition(0.5);

        telemetry.addLine("BIOBUZZ TELEOP V1 — one driver");
        telemetry.addLine("Sticks = drive");
        telemetry.addLine("RT/LT = intake in/out");
        telemetry.addLine("RB = transfer forward | LB = flower flap");
        telemetry.addLine("START = shooter toggle (ON at play)");
        telemetry.addLine("GP2 Dpad L/R = turret trim");
        telemetry.addLine("GP2 Mode = auto-aim on / forward (starts OFF)");
        telemetry.addLine("X = red/blue | Y = auto aim | BACK = reset pose");
        telemetry.addLine("Dpad U/D = P | L/R = F");
        telemetry.update();
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
                transfer.teleOpCommand(
                        () -> gamepad1.right_trigger,
                        () -> gamepad1.left_trigger,
                        () -> gamepad1.right_bumper
                ),
                shooter.updateCommand(),
                infinite(this::updateTurretAim),
                infinite(this::handleButtons),
                infinite(this::showTelemetry)
        ));
    }

    @Override
    public void loop() {
        follower.update();
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

    private void handleButtons() {
        boolean shooterToggle = gamepad1.start;
        if (shooterToggle && !lastShooterToggle) {
            shooter.toggle();
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

        if (gamepad1.dpad_up && !lastDpadUp) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_P += P_INCREMENT;
            shooter.applyPIDF();
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_P =
                    Math.max(0, SHOOTER_P - P_INCREMENT);
            shooter.applyPIDF();
        }
        if (gamepad1.dpad_right && !lastDpadRight) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_F += F_INCREMENT;
            shooter.applyPIDF();
        }
        if (gamepad1.dpad_left && !lastDpadLeft) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_F =
                    Math.max(0, SHOOTER_F - F_INCREMENT);
            shooter.applyPIDF();
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;

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

        // GP2 Mode (guide): forward (default) ↔ auto-aim
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

    private void showTelemetry() {
        Pose pose = follower.pose();

        telemetry.addLine("===== BIOBUZZ TELEOP V1 =====");
        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Turret mode", forwardOnly ? "FORWARD" : (autoAim ? "AUTO AIM" : "OFF"));
        telemetry.addData("Goal", "%.0f / %.0f", GOAL_X, GOAL_Y);
        telemetry.addData("Start", "%.0f / %.0f / %.0f", START_X, START_Y, START_HEADING_DEG);
        telemetry.addData("Intake", "%.2f", intake.getIntake().getPower());
        telemetry.addData("Transfer", "%.2f", transfer.getTransfer().getPower());
        telemetry.addData("Flower flap", "%.2f", flowerFlap.getPosition());
        telemetry.addData("Shooter", shooter.isRunning() ? "ON" : "OFF");
        telemetry.addData("Target vel", "%.0f", shooter.getTargetVelocity());
        telemetry.addData("shooter vel", "%.0f", shooter.getShooter().getVelocity());
        telemetry.addData("P/I/D/F", "%.4f / %.4f / %.4f / %.4f", SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        telemetry.addData("Turret pos", "%.3f", turret.getPosition());
        telemetry.addData("Turret trim", "%.3f", turret.getTrimPos());
        telemetry.addData("Turret deg", "%.1f", turret.getAimDegrees());
        telemetry.addData("Pose X/Y/H", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addLine("GP1: sticks drive | RT/LT intake | RB transfer | LB flap");
        telemetry.addLine("GP1 START shooter | X alliance | Y aim | BACK reset");
        telemetry.addLine("GP2 Dpad L/R = trim | Mode = forward/autoaim");
        telemetry.update();
    }
}
