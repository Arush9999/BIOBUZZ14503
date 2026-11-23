//package org.firstinspires.ftc.teamcode.pedro.auto;
//
//import static com.pedropathing.api.Paths.curve;
//import static com.pedropathing.api.Paths.line;
//import static com.pedropathing.api.Paths.path;
//
//import com.pedropathing.api.PoseFactory;
//import com.pedropathing.math.Pose;
//import com.pedropathing.paths.Path;
//
//import org.firstinspires.ftc.teamcode.pedro.misc.Constants;
//
///** Paths from Red_Left_1_Tip_Insurance.pp.json */
//public class Paths_Red_Left_1_Tip_Insurance {
//
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//
//    private final Pose start = poseFactory.of(53.5, 6.5, 180);
//    private final Pose point1 = poseFactory.of(24, 6.5, 180);
//    private final Pose point2 = poseFactory.of(10, 6.5, 180);
//    private final Pose point3 = poseFactory.of(54, 12, 180);
//    private final Pose point4 = poseFactory.of(12, 96, 0);
//    private final Pose point3Control = poseFactory.of(10, 12, 0);
//    private final Pose point4Control = poseFactory.of(12, 12, 0);
//
//    public Pose startPose() {
//        return start;
//    }
//
//    public Path path1() {
//        return line(start, point1).tangent().with(Constants.foresightConfig.maxPathSpeed.at(1.0));
//    }
//
//    public Path path2() {
//        return line(point1, point2).tangent();
//    }
//
//    public Path path3() {
//        return curve(point2, point3Control, point3).constant(point3);
//    }
//
//    public Path path4() {
//        return curve(point3, point4Control, point4).linear(point3, point4);
//    }
//
//    public Path route() {
//        return path(path1(), path2(), path3(), path4());
//    }
//}
