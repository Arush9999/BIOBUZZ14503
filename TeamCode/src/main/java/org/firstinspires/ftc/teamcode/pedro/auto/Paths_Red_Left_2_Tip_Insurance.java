package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;

public class Paths_Red_Left_2_Tip_Insurance {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(63.5, 6.5, 180);
    private final Pose point1 = poseFactory.of(22, 6.5, 180);
    private final Pose point2 = poseFactory.of(10, 6.5, 180);
    private final Pose point3Control = poseFactory.of(24, 12, 0);
    private final Pose point3 = poseFactory.of(24, 32, 90);
    private final Pose point4Control = poseFactory.of(24, 120, 0);
    private final Pose point4 = poseFactory.of(44, 120, 90);
    private final Pose point5 = poseFactory.of(44, 127.5, 90);
    private final Pose point6Control = poseFactory.of(30, 127.5, 0);
    private final Pose point6 = poseFactory.of(30, 60, 180);
    private final Pose point7Control = poseFactory.of(30, 46, 0);
    private final Pose point7 = poseFactory.of(12.5, 46, 180);
    private final Pose point8Control = poseFactory.of(12, 120, 0);
    private final Pose point8 = poseFactory.of(60, 120, 180);
    private final Pose point9 = poseFactory.of(12, 120, 180);

    public Pose startPose() {
        return start;
    }

    public Path path1() {
        return line(start, point1).tangent().with(Constants.foresightConfig.maxPathSpeed.at(1.0));
    }

    public Path path2() {
        return line(point1, point2).tangent().with(Constants.foresightConfig.maxPathSpeed.at(0.25));
    }

    public Path path3() {
        return curve(point2, point3Control, point3).linear(point2, point3);
    }

    public Path path4() {
        return curve(point3, point4Control, point4).linear(point3, point4);
    }

    public Path path5() {
        return line(point4, point5).tangent()
                .with(Constants.foresightConfig.maxPathSpeed.at(0.5));
    }

    public Path path6() {
        return curve(point5, point6Control, point6).linear(point5, point6);
    }

    public Path path7() {
        return curve(point6, point7Control, point7).linear(point6, point7)
                .with(Constants.foresightConfig.maxPathSpeed.at(0.5));
    }

    public Path path8() {
        return curve(point7, point8Control, point8).linear(point7, point8);
    }

    public Path path9() {
        return line(point8, point9).tangent();
    }

    public Path route() {
        return path(path1(), path2(), path3(), path4(), path5(), path6(), path7(), path8(), path9());
    }
}
