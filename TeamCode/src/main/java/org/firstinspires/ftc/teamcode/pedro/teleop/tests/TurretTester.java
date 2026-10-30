package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
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
    private DcMotorEx LF, RF, LB, RB;

    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(24, 24, 0);
    private final double GOAL_X = 72;    // goal center x
    private final double GOAL_Y = 84;   // goal center y


    //turret servo constants

    private final double SERVO_RANGE_DEG = 300;  // full servo travel
    private final double CENTER_POS = 0.5;
    private final double MIN_ANGLE = -90, MAX_ANGLE = 90; // allowed travel from center
    private final double DIRECTION = 1;
    private final double TURRET_OFFSET_X = 0;
    private final double TURRET_OFFSET_Y = 0;
    //drive constants
    public static double DRIVE_SPEED = 1.0;
    public static double SLOW_SPEED = 0.40;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        LF = hardwareMap.get(DcMotorEx.class, "LF");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RB = hardwareMap.get(DcMotorEx.class, "RB");


        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);

        tL = hardwareMap.get(Servo.class, "turretL");
        tR = hardwareMap.get(Servo.class, "turretR");
        tR.setDirection(Servo.Direction.REVERSE);
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
        driveRobot();

        double dist = Math.hypot(GOAL_X - tx, GOAL_Y - ty);
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f°", pose.x(), pose.y(), Math.toDegrees(heading));
        telemetry.addData("Turret target (deg)", "%.1f", turretDeg);
        telemetry.addData("Dist to goal (in)", "%.1f", dist);
        telemetry.addData("Left Turret pos", "%.3f", tL.getPosition());
        telemetry.addData("Right Turret pos", "%.3f", tR.getPosition());
        telemetry.update();
    }

    private void driveRobot() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double speed = DRIVE_SPEED;

        // Left bumper = slow mode
        if (gamepad1.left_bumper) {
            speed = SLOW_SPEED;
        }

        // Mecanum calculations

        double frontLeft = y + x + turn;
        double frontRight = y - x - turn;

        double backLeft = y - x + turn;
        double backRight = y + x - turn;

        // Normalize powers

        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(frontLeft),
                        Math.max(
                                Math.abs(frontRight),
                                Math.max(
                                        Math.abs(backLeft),
                                        Math.abs(backRight)
                                )
                        )
                )
        );

        frontLeft /= max;
        frontRight /= max;
        backLeft /= max;
        backRight /= max;

        // Apply power

        LF.setPower(frontLeft * speed);
        RF.setPower(frontRight * speed);
        LB.setPower(backLeft * speed);
        RB.setPower(backRight * speed);
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

