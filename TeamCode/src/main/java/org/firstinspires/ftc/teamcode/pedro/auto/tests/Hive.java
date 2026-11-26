package org.firstinspires.ftc.teamcode.pedro.auto.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.teleop.methods.LimelightVision;


@Autonomous(name = "Hive", group = "Auto")
public class Hive extends OpMode {

    private static final double TAG_CHECK_MS = 750;

    private LimelightVision vision;
    private final ElapsedTime sinceLastTag = new ElapsedTime();
    private boolean hiveTipped = false;

    @Override
    public void init() {
        vision = new LimelightVision(hardwareMap, true); // true = red alliance
        vision.start();
    }

    @Override
    public void start() {
        sinceLastTag.reset(); // fresh 750 ms when the match starts
    }

    @Override
    public void loop() {
        vision.update(); // read the camera once per loop

        // saw any of our hive tags (30-37) restart the timer
        if (vision.hasAllianceTags()) {
            sinceLastTag.reset();
        }

        // no tag for 750 ms -> tipped
        hiveTipped = sinceLastTag.milliseconds() > TAG_CHECK_MS;

        telemetry.addData("Hive tipped?", hiveTipped);
        telemetry.addData("ms since last tag", "%.0f", sinceLastTag.milliseconds());
        vision.addTelemetry(telemetry); // valid frame, tags per side, tag IDs
        telemetry.update();
    }

    @Override
    public void stop() {
        vision.stop();
    }
}