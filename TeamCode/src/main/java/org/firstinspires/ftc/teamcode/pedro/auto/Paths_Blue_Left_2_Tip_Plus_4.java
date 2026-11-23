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
///** Blue mirror of Paths_Red_Left_2_Tip_Plus_4.
// * x' = 144 - x, y' = 144 - y, heading' = (heading + 180) % 360. */
//public class Paths_Blue_Left_2_Tip_Plus_4 {
//
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//
//    private final Pose start = poseFactory.of(90.5, 137.5, 0);
//    private final Pose point1 = poseFactory.of(120, 137.5, 0);
//    private final Pose point2 = poseFactory.of(134, 137.5, 0);
//    private final Pose point3 = poseFactory.of(120, 112, 270);
//    private final Pose point4 = poseFactory.of(102, 24, 270);
//    private final Pose point5 = poseFactory.of(102, 16.5, 270);
//    private final Pose point6 = poseFactory.of(102, 24, 270);
//    private final Pose point7 = poseFactory.of(114, 84, 270);
//    private final Pose point8 = poseFactory.of(130.5, 98, 0);
//    private final Pose point9 = poseFactory.of(120, 98, 0);
//    private final Pose point10 = poseFactory.of(90, 128, 0);
//    private final Pose point11 = poseFactory.of(132, 50, 180);
//    private final Pose point3Control = poseFactory.of(120, 132, 180);
//    private final Pose point4Control = poseFactory.of(120, 24, 180);
//    private final Pose point7Control = poseFactory.of(114, 24, 180);
//    private final Pose point8Control = poseFactory.of(114, 98, 180);
//    private final Pose point10Control = poseFactory.of(120, 128, 180);
//    private final Pose point11Control = poseFactory.of(132, 128, 180);
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
//        return curve(point2, point3Control, point3).linear(point2, point3);
//    }
//
//    public Path path4() {
//        return curve(point3, point4Control, point4).constant(point4);
//    }
//
//    public Path path5() {
//        return line(point4, point5).tangent();
//    }
//
//    public Path path6() {
//        return line(point5, point6).reverseTangent();
//    }
//
//    public Path path7() {
//        return curve(point6, point7Control, point7).constant(point7);
//    }
//
//    public Path path8() {
//        return curve(point7, point8Control, point8).linear(point7, point8);
//    }
//
//    public Path path9() {
//        return line(point8, point9).reverseTangent();
//    }
//
//    public Path path10() {
//        return curve(point9, point10Control, point10).constant(point10);
//    }
//
//    public Path path11() {
//        return curve(point10, point11Control, point11).tangent();
//    }
//
//    public Path route() {
//        return path(
//                path1(), path2(), path3(), path4(), path5(), path6(),
//                path7(), path8(), path9(), path10(), path11());
//    }
//}
