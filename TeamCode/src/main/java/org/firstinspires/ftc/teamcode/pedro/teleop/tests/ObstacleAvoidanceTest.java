package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;


@TeleOp(name = "apf test")
public class ObstacleAvoidanceTest extends LinearOpMode {
    private DcMotor LB,RB,RF,LF;
    private Follower follower;
    public static double DRIVE_SPEED = 1.0;
    public static double SLOW_SPEED = 0.40;

    // assist constants
    static double ROBOT_R = 9;           // half the robot width
    static double ASSIST_D0 = 10;        // start slowing the approach when the gap is under this
    static double RAIL_CLEARANCE = 2;    // robot stops this far from a rail and gets pushed out if closer
    static double K_PUSH = 0.3;          // max push-away power

    // localization
    private final PoseFactory pf = PoseFactory.degrees();
    private final Pose startPose = pf.of(8, 72, 0);

    public static boolean RAILS_ALONG_Y = true;
    private static final double[][] RAILS_X = {
            {52, 95.4, 92, 97.6},
            {52, 46.4, 92, 48.6},
    };
    private static final double[][] RAILS_Y = {
            {95.8, 51.0, 96.8, 90.7},  // right rail
            {46.6, 51.0, 47.6, 90.7},  // left rail
    };


    private static final double[][] FLOWERS = {
            {43.7, 136.5, 50.7, 141.5},  // top
            {136.5, 90.8, 141.5, 97.8},  // right
            {0, 43.7, 5, 50.7},          // left
            {91, 0, 98, 5},              // bottom
    };
    static double FLOWER_D0 = 18;
    static double FLOWER_MIN_KEEP = 0.3;

    private boolean assistOn = true;
    private boolean lastY = false;
    // for telemetry
    private double nearestGap;
    private double nearestFlowerGap;
    private double blocked;

    @Override
    public void runOpMode() throws InterruptedException {

        LF = hardwareMap.get(DcMotorEx.class, "LF");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RB = hardwareMap.get(DcMotorEx.class, "RB");


        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);

        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        waitForStart();

        while(opModeIsActive()){

            follower.update();
            Pose p = follower.pose();

            if (gamepad1.y && !lastY) assistOn = !assistOn;
            lastY = gamepad1.y;

            driveRobot(p);

            telemetry.addData("Assist", assistOn ? "ON" : "OFF");
            telemetry.addData("Rails", RAILS_ALONG_Y ? "along y" : "along x");
            telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f°", p.x(), p.y(), Math.toDegrees(p.heading()));
            telemetry.addData("Nearest rail gap", "%.1f in (assist under %.0f)", nearestGap, ASSIST_D0);
            telemetry.addData("Nearest flower gap", "%.1f in (slow under %.0f)", nearestFlowerGap, FLOWER_D0);
            telemetry.addData("Input blocked", "%.0f%%", blocked * 100);
            telemetry.update();
        }
    }
    // apf assist at the hive rails (block) and the flowers (slow down for the intake)
    private void driveRobot(Pose p) {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double speed = DRIVE_SPEED;

        // Left bumper = slow mode
        if (gamepad1.left_bumper) {
            speed = SLOW_SPEED;
        }

        double h = p.heading();
        // robot frame (forward, strafe right) to field frame
        double fx = y * Math.cos(h) + x * Math.sin(h);
        double fy = y * Math.sin(h) - x * Math.cos(h);
        double inMag = Math.hypot(fx, fy);
        double[] v = {fx, fy};

        nearestGap = Double.MAX_VALUE;
        for (double[] r : RAILS_ALONG_Y ? RAILS_Y : RAILS_X) {
            // rails block fully at contact and push out
            nearestGap = Math.min(nearestGap, assist(p, r, v, ASSIST_D0, RAIL_CLEARANCE, 0, true));
        }
        nearestFlowerGap = Double.MAX_VALUE;
        for (double[] r : FLOWERS) {
            // flowers: only slow down, no push, so the intake can touch them
            nearestFlowerGap = Math.min(nearestFlowerGap, assist(p, r, v, FLOWER_D0, 0, FLOWER_MIN_KEEP, false));
        }
        fx = v[0];
        fy = v[1];
        blocked = inMag > 0.05 ? clamp(1 - Math.hypot(fx, fy) / inMag, 0, 1) : 0;

        // back to robot frame (same rotation, it's its own inverse)
        double forward = fx * Math.cos(h) + fy * Math.sin(h);
        double right   = fx * Math.sin(h) - fy * Math.cos(h);

        setMecanum(forward, right, turn, speed);
    }

    // Fades out the part of the field-frame input v that points into rectangle r.
    // minKeep = fraction of the approach kept at contact (0 = full stop), push = shove out when touching.
    // Returns the gap from the robot's edge to r.
    // clear = gap where the approach is fully faded out (and pushing starts)
    private double assist(Pose p, double[] r, double[] v, double d0, double clear, double minKeep, boolean push) {
        // nearest point on the rectangle
        double nx = clamp(p.x(), r[0], r[2]);
        double ny = clamp(p.y(), r[1], r[3]);
        double dx = p.x() - nx, dy = p.y() - ny;
        double center = Math.hypot(dx, dy);
        if (center < 1e-6) return 0; // inside the rectangle, pose is bad
        double gap = center - ROBOT_R;
        if (!assistOn || gap >= d0) return gap;

        // unit vector pointing away from the rectangle
        double ux = dx / center, uy = dy / center;

        // remove the part of the input heading into it
        double into = v[0] * ux + v[1] * uy;
        if (into < 0) {
            double keep = minKeep + (1 - minKeep) * clamp((gap - clear) / (d0 - clear), 0, 1);
            v[0] -= (1 - keep) * into * ux;
            v[1] -= (1 - keep) * into * uy;
        }

        // gentle push out
        if (push && gap < clear) {
            double k = K_PUSH * (clear - gap) / clear;
            v[0] += k * ux;
            v[1] += k * uy;
        }
        return gap;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // y forward x strafe turn clockwise
    private void setMecanum(double y, double x, double turn, double speed) {

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
}
