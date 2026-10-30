package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Paths_Red_Left {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(63.5, 6.5, 180);
    private final Pose path1 = poseFactory.of(12, 6.5, 180);
    private final Pose point2Start = poseFactory.of(12, 6.5, 180);
    private final Pose point2 = poseFactory.of(48, 120, 90);
    private final Pose point2Control1 = poseFactory.of(36, 24, 0);
    private final Pose point2Control2 = poseFactory.of(14, 120, 0);
    private final Pose point3 = poseFactory.of(48, 126, 90);
    private final Pose point4Start = poseFactory.of(48, 126, 90);
    private final Pose point4 = poseFactory.of(14, 48, 180);
    private final Pose point4Control1 = poseFactory.of(14, 120, 0);
    private final Pose point4Control2 = poseFactory.of(36, 48, 0);
    private final Pose point5Start = poseFactory.of(14, 48, 180);
    private final Pose point5 = poseFactory.of(58, 24, 90);
    private final Pose point5Control1 = poseFactory.of(14, 24, 0);

    public Pose startPose() {
        return start;
    }

    public Path path1() {
        return line(start, path1).tangent();
    }

    public Path path2() {
        return curve(point2Start, point2Control1, point2Control2, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return line(point2, point3).tangent();
    }

    public Path path4() {
        return curve(point4Start, point4Control1, point4Control2, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return curve(point5Start, point5Control1, point5).linear(point5Start, point5);
    }

    public Path route() {
        return path(path1(), path2(), path3(), path4(), path5());
    }
}
