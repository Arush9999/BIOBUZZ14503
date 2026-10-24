package org.firstinspires.ftc.teamcode.pedro.misc;

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
        }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(
        c -> {
            c.name.set("pinpoint");
            c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            c.xPodOffset.set(1.3077073585330032);
            c.yPodOffset.set(2.8448702594426676);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
        }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.31662946736700043);
            Controller secondaryTranslationalForward = Controller.proportional(0.11698612769691942);
            Controller primaryTranslationalLateral = Controller.proportional(0.29991141393645654);
            Controller secondaryTranslationalLateral = Controller.proportional(0.11080925366894842);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.0118694994021589));
            c.brake.set(Controller.proportionalFeedforward(0.010089074491835066));

            c.headingFeedback.set(Controller.proportional(6.593762366032545));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05605535490349702, 0.004139524303951793));

            c.linearBrakeCoefficients.set(Matrix.diag(0.1250887843505262, 0.05442939197127121));
            c.quadraticBrakeCoefficients.set(Matrix.diag(6.701486539246143E-4, 0.0018759904727426186));

            c.maxAchievableForwardVelocity.set(69.98182522655269);
            c.maxAchievableStrafeVelocity.set(60.34948926315254);
            c.naturalForwardDeceleration.set(43.486436649825436);
            c.naturalStrafeDeceleration.set(58.22770164491364);
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