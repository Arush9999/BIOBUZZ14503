package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_D;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_F;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_HIGH;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_I;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_P;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_TARGET_VELOCITY;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_GOAL_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_GOAL_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_START_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.BLUE_START_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_GOAL_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_GOAL_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.RED_START_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_HEADING_DEG;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_X;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_Y;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TURRET_CENTER;
import static org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TURRET_STEP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Drivetrain;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.FlowerFlap;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Intake;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Shooter;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Transfer;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;

@Config
@TeleOp(name = "Code_Test", group = "Testing")
public class Code_Test extends OpMode {

    private enum TuneParam {
        P, I, D, F, D_ALPHA, TARGET_VEL
    }

    public static double P_INCREMENT = 0.01;
    public static double I_INCREMENT = 0.0005;
    public static double D_INCREMENT = 0.1;
    public static double F_INCREMENT = 0.5;
    public static double D_FILTER_INCREMENT = 0.02;
    public static double TARGET_VEL_INCREMENT = 12.5;
    public static double INTAKE_POWER_STEP = 0.05;
    public static double TRANSFER_POWER_STEP = 0.05;

    private Follower follower;
    private Drivetrain drive;
    private Intake intake;
    private Transfer transfer;
    private FlowerFlap flowerFlap;
    private Shooter shooter;
    private Turret turret;

    private TuneParam tuneParam = TuneParam.F;
    private boolean lastInc, lastDec, lastCycle, lastStepUp, lastStepDown;
    private boolean lastIntakePowerUp, lastIntakePowerDown;
    private boolean lastResetPose;
    private boolean lastShooterToggle;
    private boolean lastFlowerFlap;
    private boolean lastAllianceToggle;
    private boolean lastAutoAimToggle;
    private boolean redAlliance = true;
    private boolean autoAim = true;

    @Override
    public void init() {
        Scheduler.reset();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        follower = Constants.create(hardwareMap);
        applyAlliance();
        resetStartPose();

        drive = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        flowerFlap = new FlowerFlap(hardwareMap);
        shooter = new Shooter(hardwareMap);
        shooter.setTargetVelocity(SHOOTER_HIGH);
        turret = new Turret(hardwareMap);
        turret.setPosition(TURRET_CENTER);

        telemetry.addLine("CODE TEST");
        telemetry.addLine("--- GP1 (robot) ---");
        telemetry.addLine("Sticks = drive");
        telemetry.addLine("RT = intake in | LT = intake out");
        telemetry.addLine("RB = transfer forward");
        telemetry.addLine("LB = flower flap toggle");
        telemetry.addLine("START / L3 = shooter toggle");
        telemetry.addLine("A = reset start pose");
        telemetry.addLine("B = toggle red/blue alliance");
        telemetry.addLine("BACK = toggle odo auto aim");
        telemetry.addLine("X/Y = manual turret (overrides aim while held)");
        telemetry.addLine("Dpad U/D = intake/transfer power");
        telemetry.addLine("--- GP2 (PIDF tuner) ---");
        telemetry.addLine("X = cycle param (P→I→D→F→α→VEL)");
        telemetry.addLine("A = increment selected | B = decrement");
        telemetry.addLine("RB = bigger step | LB = smaller step");
        telemetry.update();
    }

    @Override
    public void start() {
        schedule(parallel(
                drive.teleOpCommand(
                        () -> -gamepad1.left_stick_y,
                        () -> gamepad1.left_stick_x,
                        () -> gamepad1.right_stick_x
                ),
                intake.teleOpCommand(
                        () -> gamepad1.right_trigger,
                        () -> gamepad1.left_trigger
                ),
                transfer.teleOpCommand(
                        () -> gamepad1.right_trigger,
                        () -> gamepad1.left_trigger,
                        () -> gamepad1.right_bumper
                ),
                shooter.updateCommand(),
                infinite(this::updateTurretAim),
                infinite(this::adjustPIDF),
                infinite(this::adjustIntakePower),
                infinite(this::handleButtons),
                infinite(this::showTelemetry)
        ));
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();
    }

    @Override
    public void stop() {
        Scheduler.reset();
        intake.stop();
        transfer.stop();
        shooter.stop();
        drive.stop();
    }

    private void handleButtons() {
        boolean shooterToggle = gamepad1.start || gamepad1.left_stick_button;
        if (shooterToggle && !lastShooterToggle) {
            shooter.toggle();
        }
        lastShooterToggle = shooterToggle;

        boolean resetPose = gamepad1.a;
        if (resetPose && !lastResetPose) {
            resetStartPose();
        }
        lastResetPose = resetPose;

        boolean allianceToggle = gamepad1.b;
        if (allianceToggle && !lastAllianceToggle) {
            redAlliance = !redAlliance;
            applyAlliance();
            resetStartPose();
        }
        lastAllianceToggle = allianceToggle;

        boolean autoAimToggle = gamepad1.back;
        if (autoAimToggle && !lastAutoAimToggle) {
            autoAim = !autoAim;
        }
        lastAutoAimToggle = autoAimToggle;

        boolean flowerToggle = gamepad1.left_bumper;
        if (flowerToggle && !lastFlowerFlap) {
            flowerFlap.toggle();
        }
        lastFlowerFlap = flowerToggle;
    }

    /** Odo auto-aim both servos the same way; X/Y manual nudge overrides while held. */
    private void updateTurretAim() {
        if (gamepad1.x) {
            turret.nudge(TURRET_STEP);
            return;
        }
        if (gamepad1.y) {
            turret.nudge(-TURRET_STEP);
            return;
        }
        if (autoAim) {
            turret.update(follower);
        }
    }

    private void applyAlliance() {
        if (redAlliance) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_X = RED_GOAL_X;
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_Y = RED_GOAL_Y;
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_X = RED_START_X;
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_Y = RED_START_Y;
        } else {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_X = BLUE_GOAL_X;
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.GOAL_Y = BLUE_GOAL_Y;
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_X = BLUE_START_X;
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.START_Y = BLUE_START_Y;
        }
    }

    private void resetStartPose() {
        follower.setPose(PoseFactory.degrees().of(START_X, START_Y, START_HEADING_DEG));
    }

    private void adjustIntakePower() {
        if (gamepad1.dpad_up && !lastIntakePowerUp) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_POWER = Range.clip(
                    org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_POWER + INTAKE_POWER_STEP, 0, 1);
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_POWER = Range.clip(
                    org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_POWER + TRANSFER_POWER_STEP, 0, 1);
        }
        if (gamepad1.dpad_down && !lastIntakePowerDown) {
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_POWER = Range.clip(
                    org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_POWER - INTAKE_POWER_STEP, 0, 1);
            org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_POWER = Range.clip(
                    org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_POWER - TRANSFER_POWER_STEP, 0, 1);
        }
        lastIntakePowerUp = gamepad1.dpad_up;
        lastIntakePowerDown = gamepad1.dpad_down;
    }

    private void adjustPIDF() {
        // X = cycle selected param: P → I → D → F → α → VEL → …
        if (gamepad2.x && !lastCycle) {
            TuneParam[] order = TuneParam.values();
            tuneParam = order[(tuneParam.ordinal() + 1) % order.length];
        }
        lastCycle = gamepad2.x;

        // A / B = inc / dec the selected value
        if (gamepad2.a && !lastInc) {
            nudgeSelected(+1.0);
        }
        if (gamepad2.b && !lastDec) {
            nudgeSelected(-1.0);
        }
        lastInc = gamepad2.a;
        lastDec = gamepad2.b;

        // RB / LB = bigger / smaller step for the selected param only
        if (gamepad2.right_bumper && !lastStepUp) {
            scaleSelectedStep(2.0);
        }
        if (gamepad2.left_bumper && !lastStepDown) {
            scaleSelectedStep(0.5);
        }
        lastStepUp = gamepad2.right_bumper;
        lastStepDown = gamepad2.left_bumper;
    }

    private void nudgeSelected(double sign) {
        switch (tuneParam) {
            case P:
                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_P =
                        Math.max(0, SHOOTER_P + sign * P_INCREMENT);
                break;
            case I:
                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_I =
                        Math.max(0, SHOOTER_I + sign * I_INCREMENT);
                break;
            case D:
                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_D =
                        Math.max(0, SHOOTER_D + sign * D_INCREMENT);
                break;
            case F:
                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_F =
                        Math.max(0, SHOOTER_F + sign * F_INCREMENT);
                break;
            case D_ALPHA:
                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_D_FILTER =
                        Range.clip(
                                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_D_FILTER
                                        + sign * D_FILTER_INCREMENT,
                                0.01,
                                1.0);
                break;
            case TARGET_VEL:
                shooter.setTargetVelocity(Math.max(0, SHOOTER_TARGET_VELOCITY + sign * TARGET_VEL_INCREMENT));
                break;
        }
        shooter.applyPIDF();
    }

    private void scaleSelectedStep(double factor) {
        switch (tuneParam) {
            case P:
                P_INCREMENT = Math.max(1e-6, P_INCREMENT * factor);
                break;
            case I:
                I_INCREMENT = Math.max(1e-8, I_INCREMENT * factor);
                break;
            case D:
                D_INCREMENT = Math.max(1e-6, D_INCREMENT * factor);
                break;
            case F:
                F_INCREMENT = Math.max(1e-6, F_INCREMENT * factor);
                break;
            case D_ALPHA:
                D_FILTER_INCREMENT = Math.max(1e-4, D_FILTER_INCREMENT * factor);
                break;
            case TARGET_VEL:
                TARGET_VEL_INCREMENT = Math.max(0.5, TARGET_VEL_INCREMENT * factor);
                break;
        }
    }

    private String tuneParamName() {
        switch (tuneParam) {
            case P: return "P";
            case I: return "I";
            case D: return "D";
            case F: return "F";
            case D_ALPHA: return "Dα";
            case TARGET_VEL: return "VEL";
            default: return "?";
        }
    }

    private double tuneParamValue() {
        switch (tuneParam) {
            case P: return SHOOTER_P;
            case I: return SHOOTER_I;
            case D: return SHOOTER_D;
            case F: return SHOOTER_F;
            case D_ALPHA:
                return org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_D_FILTER;
            case TARGET_VEL: return SHOOTER_TARGET_VELOCITY;
            default: return 0;
        }
    }

    private double tuneParamStep() {
        switch (tuneParam) {
            case P: return P_INCREMENT;
            case I: return I_INCREMENT;
            case D: return D_INCREMENT;
            case F: return F_INCREMENT;
            case D_ALPHA: return D_FILTER_INCREMENT;
            case TARGET_VEL: return TARGET_VEL_INCREMENT;
            default: return 0;
        }
    }

    private void showTelemetry() {
        // Pick up live @Config edits from FTC Dashboard.
        shooter.applyPIDF();
        shooter.setTargetVelocity(SHOOTER_TARGET_VELOCITY);

        double targetVelocity = shooter.getTargetVelocity();
        double actualVelocity = shooter.getRawVelocity(); // encoder ticks/s
        double velocityError = targetVelocity - actualVelocity;
        double filteredD = shooter.getFilteredDerivative();

        // Explicit packet so Dashboard Graph can plot these keys.
        // Keep "actualVelocity" name — Dashboard graphs stick to the key you first select.
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("targetVelocity", targetVelocity);
        packet.put("actualVelocity", actualVelocity);
        packet.put("rawVelocity", actualVelocity);
        packet.put("velocityError", velocityError);
        packet.put("filteredDerivative", filteredD);
        packet.put("shooterPower", shooter.getLastPower());
        packet.put("batteryVoltage", shooter.getBatteryVoltage());
        packet.put("compensatedF", shooter.getCompensatedF());
        packet.put("compensatedP", shooter.getCompensatedP());
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        Pose pose = follower.pose();
        telemetry.addLine("===== CODE TEST =====");
        telemetry.addData("Shooter", shooter.isRunning() ? "ON" : "OFF");
        telemetry.addData("targetVelocity", targetVelocity);
        telemetry.addData("actualVelocity", actualVelocity);
        telemetry.addData("velocityError", velocityError);
        telemetry.addData("filteredD", filteredD);
        telemetry.addData("shooterPower", "%.3f", shooter.getLastPower());
        telemetry.addData("batteryVoltage", shooter.getBatteryVoltage());
        telemetry.addData("compensatedF", shooter.getCompensatedF());
        telemetry.addData("compensatedP", shooter.getCompensatedP());
        telemetry.addData("TUNING", "%s = %.4f  (step %.4f)",
                tuneParamName(), tuneParamValue(), tuneParamStep());
        telemetry.addData("P", SHOOTER_P);
        telemetry.addData("I", SHOOTER_I);
        telemetry.addData("D", SHOOTER_D);
        telemetry.addData("F", SHOOTER_F);
        telemetry.addData("D filter α",
                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_D_FILTER);
        telemetry.addData("Vnominal",
                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.SHOOTER_NOMINAL_VOLTAGE);
        telemetry.addData("Steps P/I/D/F/Dα/VEL", "%.4f / %.4f / %.4f / %.4f / %.3f / %.1f",
                P_INCREMENT, I_INCREMENT, D_INCREMENT, F_INCREMENT, D_FILTER_INCREMENT, TARGET_VEL_INCREMENT);
        telemetry.addData("Intake power set", "%.2f",
                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.INTAKE_POWER);
        telemetry.addData("Transfer power set", "%.2f",
                org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants.TRANSFER_POWER);
        telemetry.addData("intake", "%.2f", intake.getIntake().getPower());
        telemetry.addData("transfer", "%.2f", transfer.getTransfer().getPower());
        telemetry.addData("flower flap", "%.2f", flowerFlap.getPosition());
        telemetry.addData("Alliance", redAlliance ? "RED" : "BLUE");
        telemetry.addData("Auto aim", autoAim ? "ON" : "OFF");
        telemetry.addData("Goal", "%.0f / %.0f", GOAL_X, GOAL_Y);
        telemetry.addData("Turret pos", "%.3f", turret.getPosition());
        telemetry.addData("Turret deg", "%.1f", turret.getAimDegrees());
        telemetry.addData("Aim limited", turret.isAimLimited());
        telemetry.addData("Pose", "%.1f / %.1f / %.1f",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addLine("--- GP1 (robot) ---");
        telemetry.addLine("Sticks = drive");
        telemetry.addLine("RT = intake in | LT = intake out");
        telemetry.addLine("RB = transfer forward");
        telemetry.addLine("LB = flower flap toggle");
        telemetry.addLine("START / L3 = shooter toggle");
        telemetry.addLine("A = reset start pose | B = red/blue");
        telemetry.addLine("BACK = auto aim | X/Y = manual turret");
        telemetry.addLine("Dpad U/D = intake/transfer power");
        telemetry.addLine("--- GP2 (PIDF tuner) ---");
        telemetry.addLine("X = cycle P→I→D→F→α→VEL");
        telemetry.addLine("A = + | B = -");
        telemetry.addLine("RB = bigger step | LB = smaller step");
        telemetry.update();
    }
}
