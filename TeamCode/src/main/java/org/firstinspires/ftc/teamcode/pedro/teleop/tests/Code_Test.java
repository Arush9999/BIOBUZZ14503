package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_D;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_F;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_I;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_P;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_TARGET_VELOCITY;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_HEADING_DEG;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_Y;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.FlowerFlap;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Intake;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Shooter;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Transfer;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;

/**
 * PIDF / subsystem tuner built on the same Ivy methods as TeleOp V1.
 */
@TeleOp(name = "Code_Test", group = "Testing")
public class Code_Test extends OpMode {

    public static double P_INCREMENT = 0.01;
    public static double I_INCREMENT = 0.0005;
    public static double D_INCREMENT = 0.1;
    public static double F_INCREMENT = 0.5;
    public static double INTAKE_POWER_STEP = 0.05;
    public static double TRANSFER_POWER_STEP = 0.05;

    private Follower follower;
    private Drivetrain drive;
    private Intake intake;
    private Transfer transfer;
    private FlowerFlap flowerFlap;
    private Shooter shooter;
    private Turret turret;

    private boolean lastA, lastB, lastX, lastY;
    private boolean lastDpadUp, lastDpadDown, lastDpadLeft, lastDpadRight;
    private boolean lastLeftBumper, lastRightBumper;
    private boolean lastIntakePowerUp, lastIntakePowerDown;
    private boolean lastResetPose;
    private boolean lastShooterToggle;
    private boolean lastFlowerFlap;

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        resetStartPose();

        drive = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        flowerFlap = new FlowerFlap(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);

        telemetry.addLine("CODE TEST");
        telemetry.addLine("RT/LT = Intake | RB = Transfer forward");
        telemetry.addLine("LB = Flower flap toggle (0 / 1)");
        telemetry.addLine("Turret auto-aim always on");
        telemetry.addLine("GP1 A/B = P  X/Y = F  GP2 Dpad L/R = I");
        telemetry.addLine("START/L3 = Shooter | GP2 A = reset start pose");
        telemetry.update();
    }

    @Override
    public void start() {
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
                turret.autoAimCommand(follower),
                infinite(this::adjustPIDF),
                infinite(this::adjustIntakePower),
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
        boolean shooterToggle = gamepad1.start || gamepad1.left_stick_button;
        if (shooterToggle && !lastShooterToggle) {
            shooter.toggle();
        }
        lastShooterToggle = shooterToggle;

        boolean resetPose = gamepad2.a;
        if (resetPose && !lastResetPose) {
            resetStartPose();
        }
        lastResetPose = resetPose;

        boolean flowerToggle = gamepad1.left_bumper;
        if (flowerToggle && !lastFlowerFlap) {
            flowerFlap.toggle();
        }
        lastFlowerFlap = flowerToggle;
    }

    private void resetStartPose() {
        follower.setPose(PoseFactory.degrees().of(START_X, START_Y, START_HEADING_DEG));
    }

    private void adjustIntakePower() {
        if (gamepad2.dpad_up && !lastIntakePowerUp) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_POWER = Range.clip(
                    org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_POWER + INTAKE_POWER_STEP, 0, 1);
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_POWER = Range.clip(
                    org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_POWER + TRANSFER_POWER_STEP, 0, 1);
        }
        if (gamepad2.dpad_down && !lastIntakePowerDown) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_POWER = Range.clip(
                    org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_POWER - INTAKE_POWER_STEP, 0, 1);
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_POWER = Range.clip(
                    org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_POWER - TRANSFER_POWER_STEP, 0, 1);
        }
        lastIntakePowerUp = gamepad2.dpad_up;
        lastIntakePowerDown = gamepad2.dpad_down;
    }

    private void adjustPIDF() {
        if (gamepad1.a && !lastA) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_P += P_INCREMENT;
            shooter.applyPIDF();
        }
        if (gamepad1.b && !lastB) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_P =
                    Math.max(0, SHOOTER_P - P_INCREMENT);
            shooter.applyPIDF();
        }
        if (gamepad1.x && !lastX) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_F += F_INCREMENT;
            shooter.applyPIDF();
        }
        if (gamepad1.y && !lastY) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_F =
                    Math.max(0, SHOOTER_F - F_INCREMENT);
            shooter.applyPIDF();
        }
        // I tuning on GP2 dpad L/R so GP1 bumpers stay free for transfer/flap.
        if (gamepad2.dpad_right && !lastRightBumper) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_I += I_INCREMENT;
            shooter.applyPIDF();
        }
        if (gamepad2.dpad_left && !lastLeftBumper) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_I =
                    Math.max(0, SHOOTER_I - I_INCREMENT);
            shooter.applyPIDF();
        }
        if (gamepad1.dpad_up && !lastDpadUp) {
            shooter.setTargetVelocity(SHOOTER_TARGET_VELOCITY + 12.5);
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            shooter.setTargetVelocity(Math.max(0, SHOOTER_TARGET_VELOCITY - 12.5));
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
        lastLeftBumper = gamepad2.dpad_left;
        lastRightBumper = gamepad2.dpad_right;
    }

    private void showTelemetry() {
        Pose pose = follower.pose();
        telemetry.addLine("===== CODE TEST =====");
        telemetry.addData("Shooter", shooter.isRunning() ? "ON" : "OFF");
        telemetry.addData("Target", "%.0f", shooter.getTargetVelocity());
        telemetry.addData("shooter", "%.0f", shooter.getShooter().getVelocity());
        telemetry.addData("P/I/D/F", "%.4f / %.4f / %.4f / %.4f", SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        telemetry.addData("Intake power set", "%.2f",
                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_POWER);
        telemetry.addData("Transfer power set", "%.2f",
                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_POWER);
        telemetry.addData("intake", "%.2f", intake.getIntake().getPower());
        telemetry.addData("transfer", "%.2f", transfer.getTransfer().getPower());
        telemetry.addData("flower flap", "%.2f", flowerFlap.getPosition());
        telemetry.addData("Turret pos/deg", "%.3f / %.1f",
                turret.getPosition(), turret.getAimDegrees());
        telemetry.addData("Aim limited", turret.isAimLimited());
        telemetry.addData("Pose", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addLine("RT/LT intake | RB transfer fwd | LB flower flap");
        telemetry.addLine("Turret auto-aim always on");
        telemetry.addLine("GP2 A = reset start pose");
        telemetry.addLine("GP2 Dpad up/down = intake/transfer power");
        telemetry.update();
    }
}
