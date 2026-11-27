package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.CircleObstacle;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Drivetrain;

import java.util.ArrayList;
import java.util.List;


@Config
@TeleOp(name = "APF Obstacle Test", group = "Tests")
public class APF_Obstacle_Test extends OpMode {

    // tuning
    public static double START_X = 53.5, START_Y = 6.5, START_HEADING = 180;

    public static double LX = 47, LY = 69.53, LR = 6;
    public static double RX = 95.35, RY = 71.08, RR = 6;

    public static double INFLUENCE = 12;

    public static double K_REP = 50;//push strength
    public static double MAX_PUSH = 0.8; //prevents robot flinging
    public static double ROBOT_RADIUS = 9;

    public static double SLOW_SPEED = 0.4;

    public Constants constants = new Constants();

    private final PoseFactory pf = PoseFactory.degrees();
    private Follower follower;
    private Drivetrain drive;
    private final List<CircleObstacle> obstacles = new ArrayList<>();

    private boolean avoidOn = true;
    private boolean lastY = false;

    // telemetry
    private double nearestGap;
    private double pushX, pushY;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(pf.of(START_X, START_Y, START_HEADING));
        drive = new Drivetrain(hardwareMap);

        telemetry.addLine("APF Obstacle Test");
        telemetry.addData("Start", "(%.0f, %.0f) facing %.0f°", START_X, START_Y, START_HEADING);
        telemetry.addData("Left Hive", "(%.0f, %.0f) r=%.0f", LX, LY, LR);
        telemetry.addData("Right Hive", "(%.0f, %.0f) r=%.0f", RX, RY, RR);
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        Pose p = follower.pose();

        // rebuild each loop
        //Dashboard changes can apply live
        obstacles.clear();
        obstacles.add(new CircleObstacle(LX, LY, LR, INFLUENCE, K_REP));
        obstacles.add(new CircleObstacle(RX, RY, RR, INFLUENCE, K_REP));

        if (gamepad1.y && !lastY) avoidOn = !avoidOn;
        lastY = gamepad1.y;
        if (gamepad1.a) follower.setPose(pf.of(START_X, START_Y, START_HEADING));

        double speed = gamepad1.left_bumper ? SLOW_SPEED : 1.0;
        double forward = -gamepad1.left_stick_y * speed;
        double right = gamepad1.left_stick_x * speed;
        double turn = gamepad1.right_stick_x * speed;

        double h = p.heading();
        double fx = forward * Math.cos(h) + right * Math.sin(h);
        double fy = forward * Math.sin(h) - right * Math.cos(h);

        // push from obstacles
        Pose2D robotPose = new Pose2D(DistanceUnit.INCH, p.x(), p.y(), AngleUnit.RADIANS, h);
        Vector2D push = drive.calculateRepulsion(robotPose, obstacles);
        pushX = push.x();
        pushY = push.y();

        // cap the total push so it can't fling the robot
        double m = Math.hypot(pushX, pushY);
        if (m > MAX_PUSH) {
            pushX *= MAX_PUSH / m;
            pushY *= MAX_PUSH / m;
        }

        if (avoidOn) {
            fx += pushX;
            fy += pushY;
        }

        double outForward = fx * Math.cos(h) + fy * Math.sin(h);
        double outRight = fx * Math.sin(h) - fy * Math.cos(h);

        drive.drive(outForward, outRight, turn);

        nearestGap = nearestGap(p);
        telemetry.addData("Avoidance (Y)", avoidOn ? "ON" : "OFF");
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f°", p.x(), p.y(), Math.toDegrees(h));
        telemetry.addData("Gap to obstacle", "%.1f in (push under %.0f)", nearestGap, INFLUENCE);
        telemetry.addData("Push", "x=%.2f y=%.2f", pushX, pushY);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
    }

    // Robot edge to the closest obstacle's edge (in), for telemetry.
    private double nearestGap(Pose p) {
        double best = Double.MAX_VALUE;
        for (CircleObstacle o : obstacles) {
            double gap = Math.hypot(p.x() - o.x, p.y() - o.y) - o.radius - ROBOT_RADIUS;
            best = Math.min(best, gap);
        }
        return best;
    }
}
