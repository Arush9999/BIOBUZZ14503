package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_D;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_F;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_I;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_P;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_TARGET_VELOCITY;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_REVERSE_POWER;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_SHOOT_POWER;
import static org.firstinspires.ftc.teamcode.pedro.auto.Paths_Red_Left_Scrimmage_2_Tip.*;


import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.FlowerFlap;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Intake;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.LimelightVision;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Shooter;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Transfer;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;

import java.time.MonthDay;

//uses LimelightVision in methods
@Autonomous(name = "Red_Left_Scrimmage_2_Tip_HiveCheck", group = "Auto")
public class Red_Left_Scrimmage_2_Tip_HiveCheck extends OpMode {

    private static final double PATH_SPEED = 1;
    private static final double SLOW_SPEED = 0.25;
    private static final double PICKUP_SPEED = 0.5;
    private static final long PRELOAD_SPINUP_MS = 6000;
    private static final long PRELOAD_SHOT_MS = 7500;
    private static final long FLAP_DEPLOY_MS = 250;
    private static final double TURRET_PRELOAD_POS = 0.21;
    private static final double TURRET_PATH4_POS = 0.24;
    private static final double TURRET_PATH8_POS = 0.24;

    /** How long to look for the first side's tags before deciding the hive tipped. */
    private static final long tagcheck = 750;
    /** How long to wait for the other side's tags before shooting there anyway. */
    private static final long othertagwait = 750;
    /** LimelightVision cells: 0 = tags 30-33 (first side), 1 = tags 34-37 (other side). */
    private static final int FIRST_SIDE_CELL = 0;
    private static final int OTHER_SIDE_CELL = 1;

    // Same spots as Paths_Red_Left_Scrimmage_2_Tip (point8 = shooting spot, point9 = park).
    private final PoseFactory poseFactory = PoseFactory.degrees();
    // TODO: real shooting spot on the OTHER side of the hive (x, y, heading)
    private final Pose otherSideSpot = poseFactory.of(100, 130, 180);

    private final Paths_Red_Left_Scrimmage_2_Tip paths = new Paths_Red_Left_Scrimmage_2_Tip();
    private Follower follower;
    private Intake intake;
    private Transfer transfer;
    private Shooter shooter;
    private Turret turret;
    private FlowerFlap flowerFlap;
    private LimelightVision vision;
    private double intakePower = 0;
    private double transferPower = 0;

    private boolean hiveTipped = false;
    private boolean sawHiveTag = false;

    @Override
    public void init() {
        Scheduler.reset();

        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_P = 0.0;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_I = 0.0;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_D = 0.0;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_F = 14.5;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_D_FILTER = 0.18;
        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_TARGET_VELOCITY = 1825;

        follower = Constants.create(hardwareMap);
        follower.setPose(paths.startPose());
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.setTargetVelocity(SHOOTER_TARGET_VELOCITY);
        shooter.applyPIDF();
        turret = new Turret(hardwareMap);
        turret.setPosition(TURRET_PRELOAD_POS);
        flowerFlap = new FlowerFlap(hardwareMap);

        vision = new LimelightVision(hardwareMap, true); // true = red alliance
        vision.start();

        telemetry.addLine("Red_Left_Scrimmage_2_Tip_HiveCheck");
        telemetry.addData("Shooter PIDF", "%.1f / %.1f / %.1f / %.1f",
                SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        telemetry.addData("Target vel", "%.0f", SHOOTER_TARGET_VELOCITY);
        telemetry.update();
    }

    @Override
    public void start() {
        schedule(sequential(
                shooter.spinUpCommand(SHOOTER_TARGET_VELOCITY),
                preloadShot(),

                follow(follower, paths.path1()),
                follow(follower, paths.path2().with(Constants.foresightConfig.maxPathSpeed.at(SLOW_SPEED))),
                waitMs(500),

                follow(follower, paths.path3().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
                follow(follower, paths.path4().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
                instant(() -> turret.setPosition(TURRET_PATH4_POS)),
                transferOnWait(1500), // shot after path 4

                flowerFlap.downCommand(),
                waitMs(FLAP_DEPLOY_MS),
                follow(follower, paths.path5().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
                follow(follower, paths.path6().with(Constants.foresightConfig.maxPathSpeed.at(PICKUP_SPEED))),
                waitMs(1000),
                follow(follower, paths.path7().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
                follow(follower, paths.path8().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
                instant(() -> turret.setPosition(TURRET_PATH8_POS)),

                //method to check if hiveTipped
                checkHiveTipped(),

                conditional(() -> !hiveTipped,
                        // hive standing: shoot here like normal, then park
                        sequential(
                                transferOnWait(1500), // after path 8
                                flowerFlap.upCommand(),
                                follow(follower, paths.path9().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED)))
                        ),
                        // hive tipped so shoot from the other side then park
                        sequential(
                                follow(follower, paths.pathToOtherSide().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
                                instant(() -> {
                                    hiveTipped = false; // the other side hasn't tipped
                                }),
                                // wait until we can see the other side's tags before shooting
                                race(
                                        waitUntil(() -> vision.getVisibleTagCount(OTHER_SIDE_CELL) > 0),
                                        waitMs(othertagwait)
                                ),
                                transferOnWait(1500),
                                flowerFlap.upCommand(),
                                follow(follower, paths.pathOtherSideToPark().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED)))
                        )
                )
        ));
    }



    @Override
    public void loop() {
        vision.update(); // once per loop, before anything reads it
        shooter.setRunning(true);
        shooter.update();
        follower.update();
        Scheduler.execute();
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        telemetry.addData("Pose", follower.pose());
        telemetry.addData("Following", follower.following());
        telemetry.addData("Hive tipped", hiveTipped);
        telemetry.addData("First side tags", vision.getVisibleTagCount(FIRST_SIDE_CELL));
        telemetry.addData("Other side tags", vision.getVisibleTagCount(OTHER_SIDE_CELL));
        telemetry.addData("Intake", intakePower);
        telemetry.addData("Transfer", transferPower);
        telemetry.addData("Shooter", shooter.isRunning() ? "ON" : "OFF");
        telemetry.addData("targetVelocity", shooter.getTargetVelocity());
        telemetry.addData("actualVelocity", shooter.getShooter().getVelocity());
        telemetry.addData("P/I/D/F", "%.1f / %.1f / %.1f / %.1f",
                SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        telemetry.addData("compensatedF", "%.2f", shooter.getCompensatedF());
        telemetry.addData("batteryV", "%.2f", shooter.getBatteryVoltage());
        telemetry.addData("Flower flap", flowerFlap.getPosition());
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.stop();
        transfer.stop();
        shooter.stop();
        vision.stop();
    }


    /** Looks for the first side's tags for up to 750 ms (tagcheck). None seen means hive has tipped */
    private Command checkHiveTipped() {
        return sequential(
                instant(() -> sawHiveTag = false), //instantly sets the hivetag as false incase it was true from earlier
                race(
                        waitUntil(() -> {
                            if (vision.getVisibleTagCount(FIRST_SIDE_CELL) > 0) sawHiveTag = true; //checks how many tags of id 30-33 are seen in 750 ms if it sees one it skips it
                            return sawHiveTag; // stops early as soon as a tag shows up
                        }),
                        waitMs(tagcheck) //wait 750 ms
                ),
                instant(() -> hiveTipped = !sawHiveTag) //if no hive tag seen hive has tipped
        );
    }
    /*
    LimelightVision cells: 0 = tags 30-33 (first side), 1 = tags 34-37 (other side).

     */



    // shooting

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
                waitMs(PRELOAD_SHOT_MS - PRELOAD_SPINUP_MS),
                instant(this::transferReverse)
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
