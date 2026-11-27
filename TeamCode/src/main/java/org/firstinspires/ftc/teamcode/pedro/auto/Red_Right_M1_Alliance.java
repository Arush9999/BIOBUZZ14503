package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.FLOWER_FLAP_DOWN;
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
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.LimelightVision;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Shooter;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Transfer;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;

@Autonomous(name = "Red_Right_M1_Alliance", group = "Auto")
public class Red_Right_M1_Alliance extends OpMode {
    private static final double PATH_SPEED = 1;
    private static final double SLOW_SPEED = 0.25;
    private static final long PRELOAD_SPINUP_MS = 5000;
    private static final long PRELOAD_SHOT_MS = 1500;
    private static final long SHOT_MS = 1500;
    private static final long PRE_SHOT_WAIT_MS = 1000;
    private static final long INTAKE_WAIT_MS = 500;
    private static final double TURRET_PRELOAD_POS = 0.225;
    private static final double TURRET_PATH4_POS = 0.225;
    private final Paths_Red_Alliance paths = new Paths_Red_Alliance();
    private Follower follower;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private FlowerFlap flowerFlap;
    private double intakePower = 0;
    private double transferPower = 0;

    private LimelightVision vision;
    private boolean hiveTipped = false;

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
        vision = new LimelightVision(hardwareMap, true);
        vision.start();

        telemetry.addLine("Red M1 Alliance Autonomous");
        telemetry.addData("Target velocity", "%.0f", SHOOTER_TARGET_VELOCITY);
        telemetry.update();
    }

    @Override
    public void start() {
        // drive to the shooting spot then look at the hive
        // hive standing: preload shot here
        // go to the other side and shoot
        // does nothing for now
        // collect at the flower
        // TODO: tune how long to collect
        // second shot
        schedule(sequential(
                shooter.spinUpCommand(SHOOTER_TARGET_VELOCITY),

                // drive to the shooting spot then look at the hive
                follow(follower, paths.shootpose1()),
                checkHiveTip(750),

                waitMs(300),
                conditional(() -> !hiveTipped,
                        // hive standing: preload shot here
                        preloadShot(),
                        // go to the other side and shoot
                        instant(() -> {})         // does nothing for now
                ),

                // collect at the flower
                waitMs(1500),
                instant(() ->
                        flowerFlap.setPosition(FLOWER_FLAP_DOWN)
                ), intake.onCommand(),
                follow(follower, paths.flowerpose()),
                waitMs(300),                     // TODO: tune how long to collect
                follow(follower, paths.flowerpick()),
                waitMs(2000),

                // second shot
                follow(follower, paths.shootpose2()),
                instant(this::transferForward),
                waitMs(PRE_SHOT_WAIT_MS),
                instant(() -> {
                    transferPower = 0;
                    intakePower = 0;
                }),

                follow(follower, paths.parkPose())
        ));
    }

    private void transferForward() {
        transferPower = TRANSFER_SHOOT_POWER;
    }

    @Override
    public void loop() {
        shooter.setRunning(true);
        shooter.update();
        follower.update();
        vision.update();
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
        telemetry.addData("Hive tipped?", hiveTipped);
        telemetry.update();
    }

    public void stop(){
        intake.stop();
        transfer.stop();
        shooter.stop();
        vision.stop();
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
                instant(() -> transferPower = -TRANSFER_REVERSE_POWER)
        );
    }

    public Command checkHiveTip(long timeoutMs) {
        return sequential(
                instant(() -> hiveTipped = true),
                race(
                        waitUntil(() -> vision.getVisibleTagCount(0) > 0), //if tag is seen, stop early
                        waitMs(timeoutMs)                                  // if it never seen, it stops at timeout
                ),
                instant(() -> hiveTipped = vision.getVisibleTagCount(0) == 0)
        );
    }

}
