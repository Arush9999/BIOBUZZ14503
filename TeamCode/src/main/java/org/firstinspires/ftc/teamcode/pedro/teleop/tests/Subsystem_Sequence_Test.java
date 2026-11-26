package org.firstinspires.ftc.teamcode.pedro.teleop.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.FlowerFlap;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Intake;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Shooter;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Transfer;
import org.firstinspires.ftc.teamcode.pedro.teleop.methods.Turret;

/**
 * Runs every mechanism once in order:
 * intake fwd/back → transfer fwd/back → flap up/down → turret sweep → shooter.
 */
@TeleOp(name = "Subsystem_Sequence_Test", group = "Testing")
public class Subsystem_Sequence_Test extends LinearOpMode {

    private static final long FLAP_HOLD_MS = 1000;
    private static final long TURRET_HOLD_MS = 1000;
    private static final long SHOOTER_RUN_MS = 6000;

    private Intake intake;
    private Transfer transfer;
    private FlowerFlap flowerFlap;
    private Turret turret;
    private Shooter shooter;

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        flowerFlap = new FlowerFlap(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);

        intake.stop();
        transfer.stop();
        flowerFlap.up();
        shooter.stop();

        telemetry.addLine("SUBSYSTEM SEQUENCE TEST");
        telemetry.addLine("1) Intake fwd 1s, back 1s");
        telemetry.addLine("2) Transfer fwd 1s, back 2s");
        telemetry.addLine("3) Flap up, then down");
        telemetry.addLine("4) Turret center → min → max → center");
        telemetry.addLine("5) Shooter on 6s");
        telemetry.addLine("Press START to begin.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        runIntake();
        runTransfer();
        runFlap();
        runTurret();
        runShooter();

        stopAll();
        showStep("DONE");
        while (opModeIsActive()) {
            idle();
        }
    }

    private void runIntake() {
        showStep("Intake FORWARD 1s");
        intake.setPower(Constants.INTAKE_POWER);
        sleepWhileActive(1000);
        intake.stop();

        showStep("Intake BACKWARD 1s");
        intake.setPower(-Constants.INTAKE_POWER);
        sleepWhileActive(1000);
        intake.stop();
    }

    private void runTransfer() {
        showStep("Transfer FORWARD 1s");
        transfer.setPower(Constants.TRANSFER_SHOOT_POWER);
        sleepWhileActive(1000);
        transfer.stop();

        showStep("Transfer BACKWARD 2s");
        transfer.setPower(-Constants.TRANSFER_REVERSE_POWER);
        sleepWhileActive(2000);
        transfer.stop();
    }

    private void runFlap() {
        showStep("Flap UP");
        flowerFlap.up();
        sleepWhileActive(FLAP_HOLD_MS);

        showStep("Flap DOWN");
        flowerFlap.down();
        sleepWhileActive(FLAP_HOLD_MS);
    }

    private void runTurret() {
        double[] positions = {
                Constants.TURRET_CENTER,
                Constants.TURRET_MIN,
                Constants.TURRET_MAX,
                Constants.TURRET_CENTER
        };
        for (double pos : positions) {
            showStep(String.format("Turret → %.1f", pos));
            turret.setPosition(pos);
            sleepWhileActive(TURRET_HOLD_MS);
        }
    }

    private void runShooter() {
        showStep("Shooter ON 6s");
        shooter.setRunning(true);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < SHOOTER_RUN_MS) {
            shooter.update();
            telemetry.addData("shooter vel", "%.0f / %.0f",
                    shooter.getVelocity(), shooter.getTargetVelocity());
            telemetry.addData("shooter power", "%.3f", shooter.getLastPower());
            telemetry.update();
            idle();
        }

        shooter.stop();
    }

    private void stopAll() {
        intake.stop();
        transfer.stop();
        shooter.stop();
    }

    private void showStep(String step) {
        telemetry.addLine("SUBSYSTEM SEQUENCE TEST");
        telemetry.addData("Step", step);
        telemetry.addData("intake", "%.2f", intake.getIntake().getPower());
        telemetry.addData("transfer", "%.2f", transfer.getTransfer().getPower());
        telemetry.addData("flap", "%.2f", flowerFlap.getPosition());
        telemetry.addData("turret", "%.3f", turret.getPosition());
        telemetry.addData("shooter", shooter.isRunning() ? "ON" : "OFF");
        telemetry.update();
    }

    private void sleepWhileActive(long ms) {
        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.milliseconds() < ms) {
            idle();
        }
    }
}
