package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_TARGET_VELOCITY;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_POWER;

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

@Autonomous(name = "Red_Left_2_Tip_Plus_4", group = "Auto")
public class Red_Left_2_Tip_Plus_4 extends OpMode {

    private static final double PATH_SPEED = 1;
    private static final double SLOW_SPEED = 0.25;
    private static final double PICKUP_SPEED = 0.5;
    private static final long PRELOAD_SPINUP_MS = 3000;
    private static final long PRELOAD_SHOT_MS = 5000;
    private static final double TURRET_RIGHT_DEG = 90;

    private final Paths_Red_Left_2_Tip_Plus_4 paths = new Paths_Red_Left_2_Tip_Plus_4();
    private Follower follower;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private FlowerFlap flowerFlap;
    private double transferPower = -TRANSFER_POWER;

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(paths.startPose());
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        flowerFlap = new FlowerFlap(hardwareMap);
    }

    @Override
    public void start() {
        schedule(sequential(
                intake.onCommand(),
                instant(this::transferReverse),
                shooter.spinUpCommand(SHOOTER_TARGET_VELOCITY),
                preloadShot(),

                follow(follower, paths.path1()),
                follow(follower, paths.path2().with(Constants.foresightConfig.maxPathSpeed.at(SLOW_SPEED))),
                waitMs(1000),

                follow(follower, paths.path3().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
                follow(follower, paths.path4().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
                transferOnWait(4000),

                flowerFlap.downCommand(), // deploy before path 5, stay down through path 8
                follow(follower, paths.path5().with(Constants.foresightConfig.maxPathSpeed.at(PICKUP_SPEED))),
                transferOnWait(4000),

                follow(follower, paths.path6().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
                follow(follower, paths.path7().with(Constants.foresightConfig.maxPathSpeed.at(PICKUP_SPEED))),
                waitMs(2000),

                follow(follower, paths.path8().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
                transferOnWait(4000),
                flowerFlap.upCommand(), // raise after path 8 (old path 7)

                follow(follower, paths.path9().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED)))
        ));
    }

    @Override
    public void loop() {
        intake.setPower(INTAKE_POWER);
        transfer.setPower(transferPower);
        shooter.setRunning(true);
        shooter.update();
        follower.update();
        Scheduler.execute();

        telemetry.addData("Pose", follower.pose());
        telemetry.addData("Following", follower.following());
        telemetry.addData("Intake", INTAKE_POWER);
        telemetry.addData("Transfer", transferPower);
        telemetry.addData("Shooter", shooter.isRunning() ? "ON" : "OFF");
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
                    turret.setPosition(Turret.angleToPosition(TURRET_RIGHT_DEG));
                    shooter.setTargetVelocity(SHOOTER_TARGET_VELOCITY);
                    shooter.setRunning(true);
                    shooter.update();
                }),
                waitMs(PRELOAD_SPINUP_MS),
                instant(this::transferForward),
                waitMs(PRELOAD_SHOT_MS - PRELOAD_SPINUP_MS),
                instant(this::transferReverse)
        );
    }

    /** Forward during the wait, then reverse again after. */
    private Command transferOnWait(long ms) {
        return sequential(
                instant(this::transferForward),
                waitMs(ms),
                instant(this::transferReverse)
        );
    }

    private void transferForward() {
        transferPower = TRANSFER_POWER;
    }

    private void transferReverse() {
        transferPower = -TRANSFER_POWER;
    }
}
