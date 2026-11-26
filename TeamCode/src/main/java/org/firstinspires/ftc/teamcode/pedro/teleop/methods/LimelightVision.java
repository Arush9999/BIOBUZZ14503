
package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LimelightVision {

    // LIMELIGHT CONFIGURATION

    private static final int PIPELINE = 0;
    private static final int POLL_RATE = 100;
    private static final long STALE_MS = 100;

    private static final double TX_TOLERANCE = 1.0;


    // APRILTAG GROUPS


    private static final int[][] RED_CELLS = {
            {30, 31, 32, 33},
            {34, 35, 36, 37}
    };

    private static final int[][] BLUE_CELLS = {
            {38, 39, 40, 41},
            {42, 43, 44, 45}
    };


    // HARDWARE AND STATE


    private final Limelight3A limelight;

    private boolean redAlliance;
    private boolean validFrame = false;
    private boolean newFrame = false;

    private double lastTimestamp = Double.NaN;

    private LLResult result;

    private final int[] visibleTags = new int[2];


    // CONSTRUCTOR


    public LimelightVision(
            HardwareMap hardwareMap,
            boolean redAlliance) {

        this.redAlliance = redAlliance;

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight");

        limelight.setPollRateHz(POLL_RATE);
        limelight.pipelineSwitch(PIPELINE);
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }


    // ALLIANCE SELECTION


    public void setAlliance(boolean redAlliance) {
        this.redAlliance = redAlliance;

        validFrame = false;
        newFrame = false;

        visibleTags[0] = 0;
        visibleTags[1] = 0;
    }

    public boolean isRedAlliance() {
        return redAlliance;
    }

    // Returns the CELL index associated
    // with an AprilTag on our alliance.

    private int getCell(int id) {

        int[][] cells = redAlliance
                ? RED_CELLS : BLUE_CELLS;

        for (int i = 0; i < cells.length; i++) {

            for (int tagID : cells[i]) {

                if (id == tagID) {
                    return i;
                }
            }
        }

        return -1;
    }


    // VISION UPDATE


    public void update() {

        result = limelight.getLatestResult();

        validFrame = false;
        newFrame = false;

        visibleTags[0] = 0;
        visibleTags[1] = 0;

        /*
         * Reject missing, invalid, and stale
         * Limelight measurements.
         */

        if (result == null
                || !result.isValid()
                || result.getStaleness() > STALE_MS) {

            return;
        }

        validFrame = true;

        /*
         * Use the frame timestamp instead of
         * comparing consecutive TX values.
         */

        double timestamp = result.getTimestamp();

        newFrame = timestamp != lastTimestamp;
        lastTimestamp = timestamp;

        // Count visible tags in each CELL.

        for (LLResultTypes.FiducialResult tag :
                result.getFiducialResults()) {

            int cell = getCell(tag.getFiducialId());

            if (cell >= 0) {
                visibleTags[cell]++;
            }
        }
    }


    // INDIVIDUAL TAG MEASUREMENTS


    private LLResultTypes.FiducialResult findTag(
            int tagID) {

        if (!validFrame || getCell(tagID) < 0) {
            return null;
        }

        for (LLResultTypes.FiducialResult tag :
                result.getFiducialResults()) {

            if (tag.getFiducialId() == tagID) {
                return tag;
            }
        }

        return null;
    }

    public double readTx(int tagID) {

        LLResultTypes.FiducialResult tag =
                findTag(tagID);

        return tag == null
                ? Double.NaN
                : tag.getTargetXDegrees();
    }

    public double readTy(int tagID) {

        LLResultTypes.FiducialResult tag =
                findTag(tagID);

        return tag == null
                ? Double.NaN
                : tag.getTargetYDegrees();
    }

    public double readTa(int tagID) {

        LLResultTypes.FiducialResult tag =
                findTag(tagID);

        return tag == null
                ? Double.NaN
                : tag.getTargetArea();
    }


    // VISION STATUS


    public boolean hasValidFrame() {
        return validFrame;
    }

    public boolean hasNewFrame() {
        return newFrame;
    }

    public boolean hasAllianceTags() {
        return validFrame
                && (visibleTags[0] > 0
                || visibleTags[1] > 0);
    }

    public int getVisibleTagCount(int cell) {

        if (cell < 0 || cell >= 2) {
            return 0;
        }

        return visibleTags[cell];
    }

    public boolean isCellFullyVisible(int cell) {
        return getVisibleTagCount(cell) == 4;
    }

    /*
     * Individual-tag visual centering diagnostic.
     * Does not command servos or indicate
     * projectile readiness.
     */

    public boolean onTarget(int tagID) {

        double tx = readTx(tagID);

        return Double.isFinite(tx)
                && Math.abs(tx) <= TX_TOLERANCE;
    }


    // TELEMETRY


    public void addTelemetry(Telemetry telemetry) {

        telemetry.addData(
                "Vision Alliance",
                redAlliance ? "RED" : "BLUE");

        telemetry.addData("Valid Frame", validFrame);
        telemetry.addData("New Frame", newFrame);

        telemetry.addData(
                "Alliance Tags Visible",
                hasAllianceTags());

        telemetry.addData(
                "CELL 1 Visible Tags",
                visibleTags[0]);

        telemetry.addData(
                "CELL 2 Visible Tags",
                visibleTags[1]);

        telemetry.addData(
                "CELL 1 Fully Visible",
                isCellFullyVisible(0));

        telemetry.addData(
                "CELL 2 Fully Visible",
                isCellFullyVisible(1));

        if (result != null) {

            telemetry.addData(
                    "Frame Age (ms)",
                    result.getStaleness());
        }

        if (!validFrame) {
            telemetry.addLine("Vision unavailable");
            return;
        }

        telemetry.addData(
                "Frame Timestamp",
                lastTimestamp);

        for (LLResultTypes.FiducialResult tag :
                result.getFiducialResults()) {

            int id = tag.getFiducialId();

            if (getCell(id) < 0) {
                continue;
            }

            telemetry.addData(
                    "Tag " + id + " TX",
                    tag.getTargetXDegrees());

            telemetry.addData(
                    "Tag " + id + " TY",
                    tag.getTargetYDegrees());

            telemetry.addData(
                    "Tag " + id + " TA",
                    tag.getTargetArea());

            telemetry.addData(
                    "Tag " + id + " Centered",
                    onTarget(id));

            telemetry.addData(
                    "Tag " + id + " Camera Pose",
                    tag.getTargetPoseCameraSpace());
        }
    }
}

