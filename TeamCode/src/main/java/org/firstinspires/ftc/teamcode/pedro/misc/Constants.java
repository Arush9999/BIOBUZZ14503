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
                Controller primaryTranslationalForward = Controller.proportional(0.3272981495918893);
                Controller secondaryTranslationalForward = Controller.proportional(0.12092792070657658);
                Controller primaryTranslationalLateral = Controller.proportional(0.42178302045798644);
                Controller secondaryTranslationalLateral = Controller.proportional(0.15583755580935219);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012483431899796223));
                c.brake.set(Controller.proportionalFeedforward(0.01061091711482679));

                c.headingFeedback.set(Controller.proportional(4.13356587762669));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06625873487102293, 0.002779608619462871));

                c.linearBrakeCoefficients.set(Matrix.diag(0.135341435121224, 0.08115746368355699));
                c.quadraticBrakeCoefficients.set(Matrix.diag(5.369591568010557E-4, 0.0014858540033674558));

                c.maxAchievableForwardVelocity.set(67.32062685910647);
                c.maxAchievableStrafeVelocity.set(58.6033775549221);
                c.naturalForwardDeceleration.set(34.01618994932079);
                c.naturalStrafeDeceleration.set(61.25924416633701);
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