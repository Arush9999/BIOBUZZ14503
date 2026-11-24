package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;

/** Paths from Red_Left_M1_2_Tip.pp.json */
public class Paths_Red_Left_M1_2_Tip {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(53.5, 6.5, 180);
    private final Pose point1 = poseFactory.of(26, 6.5, 180);
    private final Pose point2 = poseFactory.of(12, 6.5, 180);
    private final Pose point3 = poseFactory.of(24, 32, 90);
    private final Pose point4 = poseFactory.of(48, 130, 0);
    private final Pose point5 = poseFactory.of(48, 120, 90);
    private final Pose point6 = poseFactory.of(48, 128, 90);
    private final Pose point7 = poseFactory.of(48, 120, 90);
    private final Pose point8 = poseFactory.of(48, 130, 0);
    private final Pose point9 = poseFactory.of(12, 96, 0);
    private final Pose point3Control = poseFactory.of(24, 12, 0);
    private final Pose point4Control = poseFactory.of(24, 130, 0);
    private final Pose point9Control = poseFactory.of(12, 130, 0);

    public Pose startPose() {
        return start;
    }

    public Path path1() {
        return line(start, point1).tangent().with(Constants.foresightConfig.maxPathSpeed.at(1.0));
    }

    public Path path2() {
        return line(point1, point2).tangent();
    }

    public Path path3() {
        return curve(point2, point3Control, point3).linear(point2, point3);
    }

    public Path path4() {
        return curve(point3, point4Control, point4).linear(point3, point4);
    }

    public Path path5() {
        return line(point4, point5).linear(point4, point5);
    }

    public Path path6() {
        return line(point5, point6).constant(point6);
    }

    public Path path7() {
        return line(point6, point7).reverseTangent();
    }

    public Path path8() {
        return line(point7, point8).linear(point7, point8);
    }

    public Path path9() {
        return curve(point8, point9Control, point9).constant(point9);
    }

    public Path route() {
        return path(
                path1(), path2(), path3(), path4(), path5(),
                path6(), path7(), path8(), path9());
    }
}
