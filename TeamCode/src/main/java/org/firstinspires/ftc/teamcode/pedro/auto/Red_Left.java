package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_POWER;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Intake;

@Autonomous(name = "Red Left Shoot", group = "Auto")
public class Red_Left extends OpMode {

    private static final double PATH_SPEED = 1;
    private static final double STOP_DELAY_MS = 500;

    private final Paths_Red_Left paths = new Paths_Red_Left();
    private Follower follower;
    private Intake intake;

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(paths.startPose());
        intake = new Intake(hardwareMap);
    }

    @Override
    public void start() {
        intake.setPower(INTAKE_POWER);

        schedule(sequential(
                stopAt(paths.path1()),
                stopAt(paths.path2()),
                stopAt(paths.path3()),
                stopAt(paths.path4()),
                stopAt(paths.path5())
        ));
    }

    private Command stopAt(Path path) {
        return follow(follower, path.with(Constants.foresightConfig.maxPathSpeed.at(PATH_SPEED)))
                .then(waitMs(STOP_DELAY_MS));
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("Pose", follower.pose());
        telemetry.addData("Following", follower.following());
        telemetry.addData("Intake", INTAKE_POWER);
        telemetry.update();
    }

    @Override
    public void stop() {
        intake.stop();
    }
}
