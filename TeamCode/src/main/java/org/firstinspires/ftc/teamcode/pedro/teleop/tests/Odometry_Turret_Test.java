//package org.firstinspires.ftc.teamcode.pedro.teleop.tests;
//
//import com.pedropathing.revhub.localizers.PinpointLocalizer;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
//
///**
// * Pinpoint → dual turret servos only. No Turret class, no drive.
// * Both servos get the same position (same direction).
// * Robot CW X deg → turret CCW X deg. Starts at 0.5.
// */
//@TeleOp(name = "Odometry_Turret_Test", group = "BIOBUZZ")
//public class Odometry_Turret_Test extends LinearOpMode {
//
//    private static final double CENTER = 0.5;
//
//    private double zeroHeadingDeg;
//    private boolean lastBack;
//
//    @Override
//    public void runOpMode() {
//        PinpointLocalizer pinpoint = new PinpointLocalizer(hardwareMap, Constants.localizerConfig);
//        Servo turretL = hardwareMap.get(Servo.class, "turretL");
//        Servo turretR = hardwareMap.get(Servo.class, "turretR");
//
//        pinpoint.update();
//        zeroHeadingDeg = Math.toDegrees(pinpoint.state().pose().heading());
//        setTurret(turretL, turretR, CENTER);
//
//        telemetry.addLine("PINPOINT → TURRETL/R");
//        telemetry.addLine("Both servos same direction");
//        telemetry.addLine("No drive | Back = re-zero");
//        telemetry.update();
//
//        waitForStart();
//        if (isStopRequested()) {
//            return;
//        }
//
//        pinpoint.update();
//        zeroHeadingDeg = Math.toDegrees(pinpoint.state().pose().heading());
//        setTurret(turretL, turretR, CENTER);
//
//        while (opModeIsActive()) {
//            pinpoint.update();
//            double headingDeg = Math.toDegrees(pinpoint.state().pose().heading());
//
//            if (gamepad1.back && !lastBack) {
//                zeroHeadingDeg = headingDeg;
//                setTurret(turretL, turretR, CENTER);
//            }
//            lastBack = gamepad1.back;
//
//            // World-lock: opposite of robot heading change.
//            double deltaDeg = wrapDegrees(headingDeg - zeroHeadingDeg);
//            double turretDeg = -deltaDeg;
//            double pos = Range.clip(
//                    CENTER + turretDeg / 360.0,
//                    org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TURRET_MIN,
//                    org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TURRET_MAX);
//            setTurret(turretL, turretR, pos);
//
//            telemetry.addLine("===== PINPOINT → SERVOS =====");
//            telemetry.addData("Heading", "%.2f", headingDeg);
//            telemetry.addData("Zero", "%.2f", zeroHeadingDeg);
//            telemetry.addData("Robot delta", "%.2f", deltaDeg);
//            telemetry.addData("Turret deg", "%.2f", turretDeg);
//            telemetry.addData("turretL", "%.4f", turretL.getPosition());
//            telemetry.addData("turretR", "%.4f", turretR.getPosition());
//            telemetry.addLine("Back = re-zero (0.5)");
//            telemetry.update();
//        }
//
//        setTurret(turretL, turretR, CENTER);
//    }
//
//    /** Same command to both servos — same physical direction. */
//    private static void setTurret(Servo turretL, Servo turretR, double pos) {
//        pos = Range.clip(
//                pos,
//                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TURRET_MIN,
//                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TURRET_MAX);
//        turretL.setPosition(pos);
//        turretR.setPosition(pos);
//    }
//
//    private static double wrapDegrees(double deg) {
//        while (deg > 180.0) deg -= 360.0;
//        while (deg < -180.0) deg += 360.0;
//        return deg;
//    }
//}
