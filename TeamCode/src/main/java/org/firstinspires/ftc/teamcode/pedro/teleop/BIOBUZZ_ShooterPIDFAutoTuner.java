package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "BIOBUZZ Shooter PIDF Auto Tuner")
public class BIOBUZZ_ShooterPIDFAutoTuner extends LinearOpMode {

    DcMotorEx shooter;

    // Target shooter velocity
    double TARGET_VELOCITY = 1800;

    // Starting PIDF values
    double START_P = 0.0;
    double START_I = 0.0;
    double START_D = 0.0;
    double START_F = 0.0;

    // Maximum values to test
    double MAX_P = 0.01;
    double MAX_I = 0.001;
    double MAX_D = 0.01;
    double MAX_F = 0.01;

    // Size of each increment
    double P_STEP = 0.001;
    double I_STEP = 0.0001;
    double D_STEP = 0.001;
    double F_STEP = 0.001;

    // Time settings
    double SETTLE_TIME = 1.5;
    double TEST_TIME = 1.0;

    // Best values found
    double bestP = 0;
    double bestI = 0;
    double bestD = 0;
    double bestF = 0;

    double bestScore = Double.MAX_VALUE;

    @Override
    public void runOpMode() {

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("BIOBUZZ Shooter PIDF Auto Tuner");
        telemetry.addLine("");
        telemetry.addLine("Target Velocity: " + TARGET_VELOCITY);
        telemetry.addLine("");
        telemetry.addLine("Press START to begin.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        // Start with the shooter stopped
        shooter.setVelocity(0);

        // Give the user a moment before starting
        sleep(1000);

        // --------------------------------
        // STEP 1: FIND F
        // --------------------------------

        findF();

        if (isStopRequested()) {
            stopShooter();
            return;
        }

        // --------------------------------
        // STEP 2: FIND P
        // --------------------------------

        findP();

        if (isStopRequested()) {
            stopShooter();
            return;
        }

        // --------------------------------
        // STEP 3: FIND D
        // --------------------------------

        findD();

        if (isStopRequested()) {
            stopShooter();
            return;
        }

        // --------------------------------
        // STEP 4: FIND I
        // --------------------------------

        findI();

        if (isStopRequested()) {
            stopShooter();
            return;
        }

        // --------------------------------
        // FINAL TEST
        // --------------------------------

        finalTest();

        stopShooter();

        telemetry.clear();
        telemetry.addLine("PIDF TUNING COMPLETE");
        telemetry.addLine("");
        telemetry.addData("Best P", bestP);
        telemetry.addData("Best I", bestI);
        telemetry.addData("Best D", bestD);
        telemetry.addData("Best F", bestF);
        telemetry.addData("Score", bestScore);
        telemetry.addLine("");
        telemetry.addLine("Copy these values into your TeleOp.");
        telemetry.update();

        sleep(10000);
    }

    // ==========================================
    // FIND F
    // ==========================================

    void findF() {

        double bestError = Double.MAX_VALUE;
        double testF = START_F;

        while (testF <= MAX_F && !isStopRequested()) {

            setPIDF(START_P, START_I, START_D, testF);

            shooter.setVelocity(TARGET_VELOCITY);

            sleep((long)(SETTLE_TIME * 1000));

            double error = getAverageError();

            telemetry.clear();
            telemetry.addLine("TESTING F");
            telemetry.addData("F", testF);
            telemetry.addData("Actual Velocity", shooter.getVelocity());
            telemetry.addData("Average Error", error);
            telemetry.update();

            if (error < bestError) {
                bestError = error;
                bestF = testF;
            }

            testF += F_STEP;
        }

        START_F = bestF;

        telemetry.clear();
        telemetry.addLine("F COMPLETE");
        telemetry.addData("Best F", bestF);
        telemetry.update();

        sleep(1000);
    }

    // ==========================================
    // FIND P
    // ==========================================

    void findP() {

        double bestError = Double.MAX_VALUE;
        double testP = START_P;

        while (testP <= MAX_P && !isStopRequested()) {

            setPIDF(testP, START_I, START_D, bestF);

            shooter.setVelocity(TARGET_VELOCITY);

            sleep((long)(SETTLE_TIME * 1000));

            double error = getAverageError();

            telemetry.clear();
            telemetry.addLine("TESTING P");
            telemetry.addData("P", testP);
            telemetry.addData("I", START_I);
            telemetry.addData("D", START_D);
            telemetry.addData("F", bestF);
            telemetry.addData("Actual Velocity", shooter.getVelocity());
            telemetry.addData("Average Error", error);
            telemetry.update();

            if (error < bestError) {
                bestError = error;
                bestP = testP;
            }

            testP += P_STEP;
        }

        START_P = bestP;

        telemetry.clear();
        telemetry.addLine("P COMPLETE");
        telemetry.addData("Best P", bestP);
        telemetry.update();

        sleep(1000);
    }

    // ==========================================
    // FIND D
    // ==========================================

    void findD() {

        double bestError = Double.MAX_VALUE;
        double testD = START_D;

        while (testD <= MAX_D && !isStopRequested()) {

            setPIDF(bestP, START_I, testD, bestF);

            shooter.setVelocity(TARGET_VELOCITY);

            sleep((long)(SETTLE_TIME * 1000));

            double error = getAverageError();

            telemetry.clear();
            telemetry.addLine("TESTING D");
            telemetry.addData("P", bestP);
            telemetry.addData("D", testD);
            telemetry.addData("F", bestF);
            telemetry.addData("Actual Velocity", shooter.getVelocity());
            telemetry.addData("Average Error", error);
            telemetry.update();

            if (error < bestError) {
                bestError = error;
                bestD = testD;
            }

            testD += D_STEP;
        }

        START_D = bestD;

        telemetry.clear();
        telemetry.addLine("D COMPLETE");
        telemetry.addData("Best D", bestD);
        telemetry.update();

        sleep(1000);
    }

    // ==========================================
    // FIND I
    // ==========================================

    void findI() {

        double bestError = Double.MAX_VALUE;
        double testI = START_I;

        while (testI <= MAX_I && !isStopRequested()) {

            setPIDF(bestP, testI, bestD, bestF);

            shooter.setVelocity(TARGET_VELOCITY);

            sleep((long)(SETTLE_TIME * 1000));

            double error = getAverageError();

            telemetry.clear();
            telemetry.addLine("TESTING I");
            telemetry.addData("P", bestP);
            telemetry.addData("I", testI);
            telemetry.addData("D", bestD);
            telemetry.addData("F", bestF);
            telemetry.addData("Actual Velocity", shooter.getVelocity());
            telemetry.addData("Average Error", error);
            telemetry.update();

            if (error < bestError) {
                bestError = error;
                bestI = testI;
            }

            testI += I_STEP;
        }

        START_I = bestI;

        telemetry.clear();
        telemetry.addLine("I COMPLETE");
        telemetry.addData("Best I", bestI);
        telemetry.update();

        sleep(1000);
    }

    // ==========================================
    // FINAL TEST
    // ==========================================

    void finalTest() {

        setPIDF(bestP, bestI, bestD, bestF);

        shooter.setVelocity(TARGET_VELOCITY);

        ElapsedTime timer = new ElapsedTime();

        while (timer.seconds() < TEST_TIME && !isStopRequested()) {

            double actual = shooter.getVelocity();
            double error = TARGET_VELOCITY - actual;

            telemetry.clear();
            telemetry.addLine("FINAL PIDF TEST");
            telemetry.addLine("");
            telemetry.addData("Target", TARGET_VELOCITY);
            telemetry.addData("Actual", actual);
            telemetry.addData("Error", error);
            telemetry.addLine("");
            telemetry.addData("P", bestP);
            telemetry.addData("I", bestI);
            telemetry.addData("D", bestD);
            telemetry.addData("F", bestF);
            telemetry.update();

            sleep(50);
        }
    }

    // ==========================================
    // CALCULATE AVERAGE ERROR
    // ==========================================

    double getAverageError() {

        ElapsedTime timer = new ElapsedTime();

        double totalError = 0;
        int samples = 0;

        while (timer.seconds() < TEST_TIME && !isStopRequested()) {

            double actual = shooter.getVelocity();

            double error = Math.abs(TARGET_VELOCITY - actual);

            totalError += error;
            samples++;

            sleep(50);
        }

        if (samples == 0) {
            return Double.MAX_VALUE;
        }

        return totalError / samples;
    }

    // ==========================================
    // SET PIDF
    // ==========================================

    void setPIDF(double p, double i, double d, double f) {

        shooter.setVelocityPIDFCoefficients(p, i, d, f);
    }

    // ==========================================
    // STOP SHOOTER
    // ==========================================

    void stopShooter() {

        shooter.setVelocity(0);
    }
}
