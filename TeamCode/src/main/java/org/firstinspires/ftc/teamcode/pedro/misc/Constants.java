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
            c.xPodOffset.set(4.980916000726655);
            c.yPodOffset.set(1.4673267574760858);
            c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
            c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
        }
    );

    public static ForesightConfig foresightConfig = new ForesightConfig(
        c -> {
            Controller primaryTranslationalForward = Controller.proportional(0.31198041597887477);
            Controller secondaryTranslationalForward = Controller.proportional(0.115268427433948);
            Controller primaryTranslationalLateral = Controller.proportional(0.3670631044497487);
            Controller secondaryTranslationalLateral = Controller.proportional(0.13562000898739276);

            c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
            c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

            c.coast.set(Controller.proportionalFeedforward(0.013303923334453075));
            c.brake.set(Controller.proportionalFeedforward(0.011308334834285113));

            c.headingFeedback.set(Controller.proportional(4.8851154337060745));
            c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0646588224480991, 0.005129966000540181));

            c.linearBrakeCoefficients.set(Matrix.diag(0.13800990470143706, 0.08528131978450997));
            c.quadraticBrakeCoefficients.set(Matrix.diag(5.1276003633118E-4, 0.0014443619202150792));

            c.maxAchievableForwardVelocity.set(62.91320678670853);
            c.maxAchievableStrafeVelocity.set(55.56769358782532);
            c.naturalForwardDeceleration.set(39.36613413237287);
            c.naturalStrafeDeceleration.set(61.119097701565124);
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