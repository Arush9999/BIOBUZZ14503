package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(
        c -> {
            c.frontLeftName.set("LF");
            c.frontRightName.set("RF");
            c.backLeftName.set("LB");
            c.backRightName.set("RB");
            c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
            c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
            c.manualBrakeMode.set(true);
        }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(
    c -> {
            c.name.set("pinpoint");
            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            c.xPodOffset.set(-4.52919156532588);
            c.yPodOffset.set(-0.7675236979807456);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
        }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.18123280305662043);
            Controller secondaryTranslationalForward = Controller.proportional(0.06696067809973559);
            Controller primaryTranslationalLateral = Controller.proportional(0.23684382677430674);
            Controller secondaryTranslationalLateral = Controller.proportional(0.08750746540949973);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.010171316095925873));
            c.brake.set(Controller.proportionalFeedforward(0.008645618681536993));

            c.headingFeedback.set(Controller.proportional(1.991165941584554));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.045795586114149134, 0.005874935922214319));

            c.linearBrakeCoefficients.set(Matrix.diag(0.0798999117556175, 0.05187145650938242));
            c.quadraticBrakeCoefficients.set(Matrix.diag(0.0018438473502983127, 0.002247970207156194));

            c.maxAchievableForwardVelocity.set(77.4631050709046);
            c.maxAchievableStrafeVelocity.set(62.70299618993852);
            c.naturalForwardDeceleration.set(25.567318935193825);
            c.naturalStrafeDeceleration.set(44.13366757406659);
        }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}