package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;

/** Blue mirror of Paths_Red_Left_M1_2_Tip.
 * x' = 144 - x, y' = 144 - y, heading' = (heading + 180) % 360. */
public class Paths_Blue_Left_M1_2_Tip {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(90.5, 137.5, 0);
    private final Pose point1 = poseFactory.of(90.5, 132, 0);
    private final Pose point2 = poseFactory.of(120, 137.5, 0);
    private final Pose point3 = poseFactory.of(134.5, 137.5, 0);
    private final Pose point4 = poseFactory.of(120, 112, 270);
    private final Pose point5 = poseFactory.of(96, 14, 180);
    private final Pose point6 = poseFactory.of(96, 24, 270);
    private final Pose point7 = poseFactory.of(96, 17, 270);
    private final Pose point8 = poseFactory.of(96, 24, 270);
    private final Pose point9 = poseFactory.of(96, 14, 180);
    private final Pose point10 = poseFactory.of(132, 48, 180);
    private final Pose point2Control = poseFactory.of(90.5, 137.5, 180);
    private final Pose point4Control = poseFactory.of(120, 132, 180);
    private final Pose point5Control = poseFactory.of(120, 14, 180);
    private final Pose point10Control = poseFactory.of(132, 14, 180);

    public Pose startPose() {
        return start;
    }

    public Path path1() {
        return line(start, point1).constant(point1)
                .with(Constants.foresightConfig.maxPathSpeed.at(1.0));
    }

    public Path path2() {
        return curve(point1, point2Control, point2).constant(point2);
    }

    public Path path3() {
        return line(point2, point3).tangent();
    }

    public Path path4() {
        return curve(point3, point4Control, point4).linear(point3, point4);
    }

    public Path path5() {
        return curve(point4, point5Control, point5).linear(point4, point5);
    }

    public Path path6() {
        return line(point5, point6).linear(point5, point6);
    }

    public Path path7() {
        return line(point6, point7).constant(point7);
    }

    public Path path8() {
        return line(point7, point8).reverseTangent();
    }

    public Path path9() {
        return line(point8, point9).linear(point8, point9);
    }

    public Path path10() {
        return curve(point9, point10Control, point10).reverseTangent();
    }

    public Path route() {
        return path(
                path1(), path2(), path3(), path4(), path5(),
                path6(), path7(), path8(), path9(), path10());
    }
}
