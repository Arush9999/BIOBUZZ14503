package org.firstinspires.ftc.teamcode.pedro.teleop;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_HIGH;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_LOW;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_MEDIUM;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_HEADING_DEG;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_Y;

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

    private Follower follower;
    private Drivetrain drive;
    private Intake intake;
    private Transfer transfer;
    private FlowerFlap flowerFlap;
    private Shooter shooter;
    private Turret turret;

    private boolean lastResetPose = false;
    private boolean lastShooterToggle = false;
    private boolean lastFlowerFlap = false;

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

        telemetry.addLine("BIOBUZZ TELEOP V1");
        telemetry.addLine("RT/LT = Intake");
        telemetry.addLine("Transfer reverses with intake");
        telemetry.addLine("RB = Transfer forward");
        telemetry.addLine("LB = Flower flap toggle (0 / 1)");
        telemetry.addLine("START/L3 = Shooter toggle");
        telemetry.addLine("Turret auto-aim always on");
        telemetry.addLine("GP2 A = reset start pose");
        telemetry.addLine("Ready!");
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

        if (gamepad2.dpad_up) {
            shooter.setTargetVelocity(SHOOTER_HIGH);
        } else if (gamepad2.dpad_left) {
            shooter.setTargetVelocity(SHOOTER_MEDIUM);
        } else if (gamepad2.dpad_down) {
            shooter.setTargetVelocity(SHOOTER_LOW);
        }

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

    private void showTelemetry() {
        Pose pose = follower.pose();

        telemetry.addLine("===== BIOBUZZ TELEOP V1 =====");
        telemetry.addData("Intake", "%.2f", intake.getIntake().getPower());
        telemetry.addData("Transfer", "%.2f", transfer.getTransfer().getPower());
        telemetry.addData("Flower flap", "%.2f", flowerFlap.getPosition());
        telemetry.addData("Shooter", shooter.isRunning() ? "ON" : "OFF");
        telemetry.addData("Target vel", "%.0f", shooter.getTargetVelocity());
        telemetry.addData("shooter vel", "%.0f", shooter.getShooter().getVelocity());
        telemetry.addData("Turret pos", "%.3f", turret.getPosition());
        telemetry.addData("Turret aim deg", "%.1f", turret.getAimDegrees());
        telemetry.addData("Aim limited", turret.isAimLimited());
        telemetry.addData("Pose X/Y", "%.1f / %.1f", pose.x(), pose.y());
        telemetry.addData("Heading deg", "%.1f", Math.toDegrees(pose.heading()));
        telemetry.addLine("RT/LT intake | RB transfer fwd | LB flower flap");
        telemetry.addLine("START/L3 shooter | GP2 A = reset start pose");
        telemetry.addLine("Turret auto-aim always on");
        telemetry.update();
    }
}
