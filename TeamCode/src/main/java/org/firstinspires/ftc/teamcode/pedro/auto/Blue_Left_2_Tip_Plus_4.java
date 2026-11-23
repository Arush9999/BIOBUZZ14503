//package org.firstinspires.ftc.teamcode.pedro.auto;
//
//import static com.pedropathing.ivy.Scheduler.schedule;
//import static com.pedropathing.ivy.commands.Commands.instant;
//import static com.pedropathing.ivy.commands.Commands.waitMs;
//import static com.pedropathing.ivy.groups.Groups.sequential;
//import static com.pedropathing.ivy.pedro.PedroCommands.follow;
//import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_POWER;
//import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_D;
//import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_F;
//import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_I;
//import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_P;
//import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_TARGET_VELOCITY;
//import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_REVERSE_POWER;
//import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_SHOOT_POWER;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.ivy.Command;
//import com.pedropathing.ivy.Scheduler;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
//import org.firstinspires.ftc.teamcode.pedro.teleop.methods.FlowerFlap;
//import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Intake;
//import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Shooter;
//import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Transfer;
//import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;
//
//@Autonomous(name = "Blue_Left_2_Tip_Plus_4", group = "Auto")
//public class Blue_Left_2_Tip_Plus_4 extends OpMode {
//
//    private static final double PATH_SPEED = 1;
//    private static final double SLOW_SPEED = 0.25;
//    private static final double PICKUP_SPEED = 0.5;
//    private static final long PRELOAD_SPINUP_MS = 5000;
//    private static final long PRELOAD_SHOT_MS = 8000;
//    private static final long FLAP_DEPLOY_MS = 500;
//    private static final double TURRET_PRELOAD_POS = 0.21;
//    private static final double TURRET_PATH10_POS = 0.21;
//
//    private final Paths_Blue_Left_2_Tip_Plus_4 paths = new Paths_Blue_Left_2_Tip_Plus_4();
//    private Follower follower;
//    private Intake intake;
//    private Transfer transfer;
//    private Shooter shooter;
//    private Turret turret;
//    private FlowerFlap flowerFlap;
//    private double intakePower = 0;
//    private double transferPower = 0;
//
//    @Override
//    public void init() {
//        Scheduler.reset();
//
//        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_P = 0.0;
//        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_I = 0.0;
//        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_D = 0.0;
//        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_F = 13.75;
//        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_D_FILTER = 0.18;
//        org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_TARGET_VELOCITY = 1800;
//
//        follower = Constants.create(hardwareMap);
//        follower.setPose(paths.startPose());
//        intake = new Intake(hardwareMap);
//        transfer = new Transfer(hardwareMap);
//        shooter = new Shooter(hardwareMap);
//        shooter.setTargetVelocity(SHOOTER_TARGET_VELOCITY);
//        shooter.applyPIDF();
//        turret = new Turret(hardwareMap);
//        turret.setPosition(TURRET_PRELOAD_POS);
//        flowerFlap = new FlowerFlap(hardwareMap);
//
//        telemetry.addLine("Blue_Left_2_Tip_Plus_4");
//        telemetry.addData("Shooter PIDF", "%.1f / %.1f / %.1f / %.1f",
//                SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
//        telemetry.addData("Target vel", "%.0f", SHOOTER_TARGET_VELOCITY);
//        telemetry.update();
//    }
//
//    @Override
//    public void start() {
//        schedule(sequential(
//                shooter.spinUpCommand(SHOOTER_TARGET_VELOCITY),
//                preloadShot(),
//
//                follow(follower, paths.path1()),
//                follow(follower, paths.path2().with(Constants.foresightConfig.maxPathSpeed.at(SLOW_SPEED))),
//                waitMs(1000),
//
//                follow(follower, paths.path3().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
//                follow(follower, paths.path4().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
//                transferOnWait(4000), // shot after path 4
//
//                flowerFlap.downCommand(),
//                waitMs(FLAP_DEPLOY_MS),
//                follow(follower, paths.path5().with(Constants.foresightConfig.maxPathSpeed.at(PICKUP_SPEED))),
//                transferOnWait(4000), // shot after path 5
//
//                follow(follower, paths.path6().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
//                follow(follower, paths.path7().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
//                follow(follower, paths.path8().with(Constants.foresightConfig.maxPathSpeed.at(PICKUP_SPEED))),
//                waitMs(2000),
//                follow(follower, paths.path9().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
//                waitMs(2000),
//
//                flowerFlap.upCommand(),
//                follow(follower, paths.path10().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED))),
//                instant(() -> turret.setPosition(TURRET_PATH10_POS)),
//                transferOnWait(4000), // shot after path 10
//
//                follow(follower, paths.path11().with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED)))
//        ));
//    }
//
//    @Override
//    public void loop() {
//        shooter.setRunning(true);
//        shooter.update();
//        follower.update();
//        Scheduler.execute();
//        intake.setPower(intakePower);
//        transfer.setPower(transferPower);
//
//        telemetry.addData("Pose", follower.pose());
//        telemetry.addData("Following", follower.following());
//        telemetry.addData("Intake", intakePower);
//        telemetry.addData("Transfer", transferPower);
//        telemetry.addData("Shooter", shooter.isRunning() ? "ON" : "OFF");
//        telemetry.addData("targetVelocity", shooter.getTargetVelocity());
//        telemetry.addData("actualVelocity", shooter.getShooter().getVelocity());
//        telemetry.addData("P/I/D/F", "%.1f / %.1f / %.1f / %.1f",
//                SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
//        telemetry.addData("compensatedF", "%.2f", shooter.getCompensatedF());
//        telemetry.addData("batteryV", "%.2f", shooter.getBatteryVoltage());
//        telemetry.addData("Flower flap", flowerFlap.getPosition());
//        telemetry.update();
//    }
//
//    @Override
//    public void stop() {
//        intake.stop();
//        transfer.stop();
//        shooter.stop();
//    }
//
//    private Command preloadShot() {
//        return sequential(
//                instant(() -> {
//                    intakePower = 0;
//                    transferPower = 0;
//                    turret.setPosition(TURRET_PRELOAD_POS);
//                    shooter.setTargetVelocity(SHOOTER_TARGET_VELOCITY);
//                    shooter.setRunning(true);
//                    shooter.update();
//                }),
//                waitMs(PRELOAD_SPINUP_MS),
//                instant(() -> intakePower = INTAKE_POWER),
//                instant(this::transferForward),
//                waitMs(PRELOAD_SHOT_MS - PRELOAD_SPINUP_MS),
//                instant(this::transferReverse)
//        );
//    }
//
//    private Command transferOnWait(long ms) {
//        return sequential(
//                instant(this::transferForward),
//                waitMs(ms),
//                instant(this::transferReverse)
//        );
//    }
//
//    private void transferForward() {
//        transferPower = TRANSFER_SHOOT_POWER;
//    }
//
//    private void transferReverse() {
//        transferPower = -TRANSFER_REVERSE_POWER;
//    }
//}
