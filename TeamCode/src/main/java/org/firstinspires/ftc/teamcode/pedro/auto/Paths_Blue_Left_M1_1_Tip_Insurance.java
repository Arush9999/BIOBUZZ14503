package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.misc.Constants;

/** Blue mirror of Paths_Red_Left_M1_1_Tip_Insurance.
 * x' = 144 - x, y' = 144 - y, heading' = (heading + 180) % 360. */
public class Paths_Blue_Left_M1_1_Tip_Insurance {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(90.5, 137.5, 0);
    private final Pose point1 = poseFactory.of(90.5, 132, 0);
    private final Pose point2 = poseFactory.of(120, 137.5, 0);
    private final Pose point3 = poseFactory.of(132, 137.5, 0);
    private final Pose point4 = poseFactory.of(90.5, 132, 0);
    private final Pose point5 = poseFactory.of(132, 48, 0);
    private final Pose point2Control = poseFactory.of(90.5, 137.5, 180);
    private final Pose point4Control = poseFactory.of(132, 132, 180);
    private final Pose point5Control = poseFactory.of(132, 132, 180);

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
        return curve(point3, point4Control, point4).constant(point4);
    }

    public Path path5() {
        return curve(point4, point5Control, point5).constant(point5);
    }

    public Path route() {
        return path(path1(), path2(), path3(), path4(), path5());
    }
}
