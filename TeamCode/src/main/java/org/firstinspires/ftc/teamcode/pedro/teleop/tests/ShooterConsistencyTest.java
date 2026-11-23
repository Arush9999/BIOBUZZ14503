package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Shooter PIDF Test", group = "tests")
public class ShooterConsistencyTest extends OpMode {

    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.00044;

    public static double TARGET_VELOCITY = 1200;
    public static double TARGET_STEP = 50;
    public static double NOMINAL_VOLTAGE = 12.0;
    public static double I_ZONE = 100;
    public static double READY_TOLERANCE = 25;

    private DcMotorEx shooter;
    private DcMotor intake;
    private VoltageSensor battery;

    private boolean shooterOn = false;
    private double integral = 0, lastError = 0, power = 0, velocity = 0;
    private final ElapsedTime dtTimer = new ElapsedTime();

    private int selected = 0;
    private final double[] steps = {0.0005, 0.00001, 0.00001, 0.00001};
    private static final String[] NAMES = {"kP", "kI", "kD", "kF"};

    private boolean lastA, lastB, lastX, lastY, lastUp, lastDown, lastLeft, lastRight;

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);

        battery = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void loop() {
        handleButtons();
        runShooter();
        intake.setPower(gamepad1.right_trigger > 0.1 ? gamepad1.right_trigger : 0);
        showTelemetry();
    }

    private void handleButtons() {
        if (gamepad1.a && !lastA) shooterOn = !shooterOn;
        if (gamepad1.dpad_up && !lastUp) TARGET_VELOCITY += TARGET_STEP;
        if (gamepad1.dpad_down && !lastDown) TARGET_VELOCITY = Math.max(0, TARGET_VELOCITY - TARGET_STEP);
        if (gamepad1.b && !lastB) selected = (selected + 1) % 4;
        if (gamepad1.dpad_right && !lastRight) changeGain(steps[selected]);
        if (gamepad1.dpad_left && !lastLeft) changeGain(-steps[selected]);
        if (gamepad1.y && !lastY) steps[selected] *= 10;
        if (gamepad1.x && !lastX) steps[selected] /= 10;

        lastA = gamepad1.a; lastB = gamepad1.b; lastX = gamepad1.x; lastY = gamepad1.y;
        lastUp = gamepad1.dpad_up; lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left; lastRight = gamepad1.dpad_right;
    }

    private void changeGain(double delta) {
        switch (selected) {
            case 0: kP = Math.max(0, kP + delta); break;
            case 1: kI = Math.max(0, kI + delta); integral = 0; break;
            case 2: kD = Math.max(0, kD + delta); break;
            case 3: kF = Math.max(0, kF + delta); break;
        }
    }

    private void runShooter() {
        double dt = dtTimer.seconds();
        dtTimer.reset();

        velocity = shooter.getVelocity();

        if (!shooterOn) {
            shooter.setPower(0);
            integral = 0;
            lastError = 0;
            power = 0;
            return;
        }

        double error = TARGET_VELOCITY - velocity;
        if (Math.abs(error) < I_ZONE) integral += error * dt; else integral = 0;
        double derivative = dt > 0 ? (error - lastError) / dt : 0;
        lastError = error;

        double ff = kF * TARGET_VELOCITY * (NOMINAL_VOLTAGE / battery.getVoltage());
        power = Range.clip(ff + kP * error + kI * integral + kD * derivative, 0, 1);
        shooter.setPower(power);
    }

    private void showTelemetry() {
        double error = TARGET_VELOCITY - velocity;
        boolean ready = shooterOn && Math.abs(error) < READY_TOLERANCE;

        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("A: shooter on/off");
        telemetry.addLine("Dpad up/down: target +/- " + (int) TARGET_STEP);
        telemetry.addLine("B: select increment (P > I > D > F)");
        telemetry.addData("currently incrementing", NAMES[selected]);
        telemetry.addLine("Dpad right/left: selected gain up/down");
        telemetry.addLine("Y: step x10   X: step /10");
        telemetry.addLine("Right trigger: intake");

        telemetry.addLine();
        telemetry.addLine(ready ? "AT SPEED" : shooterOn ? "spinning up" : "SHOOTER OFF (A)");
        telemetry.addData("Target", "%.0f", TARGET_VELOCITY);
        telemetry.addData("Velocity", "%.0f", velocity);
        telemetry.addData("Error", "%.0f", error);
        telemetry.addData("Power", "%.3f", power);
        telemetry.addData("Battery", "%.2f V", battery.getVoltage());
        telemetry.update();

        double[] g = {kP, kI, kD, kF};
        for (int i = 0; i < 4; i++) {
            telemetry.addData((i == selected ? "> " : "  ") + NAMES[i], "%.6f  (step %.6f)", g[i], steps[i]);
        }
    }
}
