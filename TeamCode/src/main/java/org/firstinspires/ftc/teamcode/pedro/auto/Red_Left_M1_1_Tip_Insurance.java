package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_TARGET_VELOCITY;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_REVERSE_POWER;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_SHOOT_POWER;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.FlowerFlap;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Intake;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Shooter;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Transfer;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;

@Autonomous(name = "Red_Left_M1_1_Tip_Insurance", group = "Auto")
public class Red_Left_M1_1_Tip_Insurance extends OpMode {

    private static final double PATH_SPEED = 1;
    private static final double SLOW_SPEED = 0.25;
    private static final long PRELOAD_SPINUP_MS = 5000;
    private static final long PRELOAD_SHOT_MS = 1500;
    private static final long SHOT_MS = 1500;
    private static final long PRE_SHOT_WAIT_MS = 1000;
    private static final long INTAKE_WAIT_MS = 500;
    private static final double TURRET_PRELOAD_POS = 0.225;
    private static final double TURRET_PATH4_POS = 0.225;

    private final Paths_Red_Left_M1_1_Tip_Insurance paths =
            new Paths_Red_Left_M1_1_Tip_Insurance();
    private Follower follower;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private FlowerFlap flowerFlap;
    private double intakePower = 0;
    private double transferPower = 0;

    @Override
    public void init() {
        Scheduler.reset();
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_TARGET_VELOCITY = 1825;

        follower = Constants.create(hardwareMap);
        follower.setPose(paths.startPose());
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.setTargetVelocity(SHOOTER_TARGET_VELOCITY);
        turret = new Turret(hardwareMap);
        turret.setPosition(TURRET_PRELOAD_POS);
        flowerFlap = new FlowerFlap(hardwareMap);

        telemetry.addLine("Red_Left_M1_1_Tip_Insurance");
        telemetry.addData("Target vel", "%.0f", SHOOTER_TARGET_VELOCITY);
        telemetry.update();
    }

    @Override
    public void start() {
        schedule(sequential(
                shooter.spinUpCommand(SHOOTER_TARGET_VELOCITY),
                follow(follower, paths.path1()),
                preloadShot(),

                follow(follower, paths.path2().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
                follow(follower, paths.path3().with(Constants.foresightConfig.maxPathSpeed.at(SLOW_SPEED))),
                waitMs(INTAKE_WAIT_MS),

                follow(follower, paths.path4().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
                shotAfterPath(TURRET_PATH4_POS),

                follow(follower, paths.path5().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED)))
        ));
    }

    @Override
    public void loop() {
        shooter.setRunning(true);
        shooter.update();
        follower.update();
        Scheduler.execute();
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        telemetry.addData("Pose", follower.pose());
        telemetry.addData("Following", follower.following());
        telemetry.addData("Intake", intakePower);
        telemetry.addData("Transfer", transferPower);
        telemetry.addData("Shooter", shooter.isRunning() ? "ON" : "OFF");
        telemetry.addData("targetVelocity", shooter.getTargetVelocity());
        telemetry.addData("actualVelocity", shooter.getShooter().getVelocity());
        telemetry.addData("Bang", shooter.isBangOn() ? "FULL" : "COAST");
        telemetry.addData("Power", "%.2f", shooter.getLastPower());
        telemetry.addData("Flower flap", flowerFlap.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.stop();
        transfer.stop();
        shooter.stop();
    }

    private Command preloadShot() {
        return sequential(
                instant(() -> {
                    intakePower = 0;
                    transferPower = 0;
                    turret.setPosition(TURRET_PRELOAD_POS);
                    shooter.setTargetVelocity(SHOOTER_TARGET_VELOCITY);
                    shooter.setRunning(true);
                    shooter.update();
                }),
                waitMs(PRELOAD_SPINUP_MS),
                instant(() -> intakePower = INTAKE_POWER),
                instant(this::transferForward),
                waitMs(PRELOAD_SHOT_MS),
                instant(this::transferReverse)
        );
    }

    private Command shotAfterPath(double turretPos) {
        return sequential(
                waitMs(PRE_SHOT_WAIT_MS),
                instant(() -> turret.setPosition(turretPos)),
                transferOnWait(SHOT_MS)
        );
    }

    private Command transferOnWait(long ms) {
        return sequential(
                instant(this::transferForward),
                waitMs(ms),
                instant(this::transferReverse)
        );
    }

    private void transferForward() {
        transferPower = TRANSFER_SHOOT_POWER;
    }

    private void transferReverse() {
        transferPower = -TRANSFER_REVERSE_POWER;
    }
}
