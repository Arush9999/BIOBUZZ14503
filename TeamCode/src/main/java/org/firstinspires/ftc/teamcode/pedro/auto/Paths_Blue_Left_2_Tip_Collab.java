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
///** Blue mirror of Paths_Red_Left_2_Tip_Collab.
// * x' = 144 - x, y' = 144 - y, heading' = (heading + 180) % 360. */
//public class Paths_Blue_Left_2_Tip_Collab {
//
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//
//    private final Pose start = poseFactory.of(90.5, 137.5, 0);
//    private final Pose point1 = poseFactory.of(122, 137.5, 0);
//    private final Pose point2 = poseFactory.of(134, 137.5, 0);
//    private final Pose point3 = poseFactory.of(90, 128, 0);
//    private final Pose point4 = poseFactory.of(130.5, 98, 0);
//    private final Pose point5 = poseFactory.of(90, 128, 0);
//    private final Pose point6 = poseFactory.of(132, 48, 0);
//    private final Pose point3Control = poseFactory.of(132, 128, 180);
//    private final Pose point4Control1 = poseFactory.of(106, 128, 180);
//    private final Pose point4Control2 = poseFactory.of(114, 98, 180);
//    private final Pose point5Control1 = poseFactory.of(114, 98, 180);
//    private final Pose point5Control2 = poseFactory.of(106, 128, 180);
//    private final Pose point6Control = poseFactory.of(132, 128, 180);
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
//        return curve(point3, point4Control1, point4Control2, point4).constant(point4);
//    }
//
//    public Path path5() {
//        return curve(point4, point5Control1, point5Control2, point5).constant(point5);
//    }
//
//    public Path path6() {
//        return curve(point5, point6Control, point6).tangent();
//    }
//
//    public Path route() {
//        return path(path1(), path2(), path3(), path4(), path5(), path6());
//    }
//}
