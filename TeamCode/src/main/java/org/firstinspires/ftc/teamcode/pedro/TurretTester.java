package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.pedro.Constants;
@TeleOp(name = "turret tester")
public class TurretTester extends OpMode {
    private Follower follower;
    private Servo tL;
    private Servo tR;

    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(72, 72, 0);
    private final double GOAL_X = 60;    // goal center x
    private final double GOAL_Y = 60;   // goal center y

    private final double SERVO_RANGE_DEG = 300;  // full servo travel
    private final double CENTER_POS = 0;
    private final double MIN_ANGLE = -90, MAX_ANGLE = 90; // allowed travel from center
    private final double DIRECTION = 1;
    private final double TURRET_OFFSET_X = 0;
    private final double TURRET_OFFSET_Y = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        tL = hardwareMap.get(Servo.class, "turretL");
        tR = hardwareMap.get(Servo.class, "turretR");
        setTurretAngle(0);
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        follower.update();

        Pose pose = follower.pose();
        double heading = pose.heading();

        // turret pivot position on the field
        double tx = pose.x() + TURRET_OFFSET_X * Math.cos(heading) - TURRET_OFFSET_Y * Math.sin(heading);
        double ty = pose.y() + TURRET_OFFSET_X * Math.sin(heading) + TURRET_OFFSET_Y * Math.cos(heading);

        // field angle to goal relative to the robot
        double fieldAngle = Math.atan2(GOAL_Y - ty, GOAL_X - tx);
        double turretDeg = Math.toDegrees(wrap(fieldAngle - heading));

        setTurretAngle(turretDeg);

        double dist = Math.hypot(GOAL_X - tx, GOAL_Y - ty);
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f°", pose.x(), pose.y(), Math.toDegrees(heading));
        telemetry.addData("Turret target (deg)", "%.1f", turretDeg);
        telemetry.addData("Dist to goal (in)", "%.1f", dist);
        telemetry.addData("Left Turret pos", "%.3f", tL.getPosition());
        telemetry.addData("Right Turret pos", "%.3f", tR.getPosition());
        telemetry.update();
    }

    private void setTurretAngle(double deg) {
        deg = Range.clip(DIRECTION * deg, MIN_ANGLE, MAX_ANGLE);
        tL.setPosition(Range.clip(CENTER_POS + deg / SERVO_RANGE_DEG, 0, 1));
        tR.setPosition(Range.clip(CENTER_POS + deg / SERVO_RANGE_DEG, 0, 1));
    }


    private double wrap(double a) {
        while (a > Math.PI)  a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }
}