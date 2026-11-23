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
///** Paths from Red_Left_2_Tip_Plus_4.pp.json */
//public class Paths_Red_Left_2_Tip_Plus_4 {
//
//    private final PoseFactory poseFactory = PoseFactory.degrees();
//
//    private final Pose start = poseFactory.of(53.5, 6.5, 180);
//    private final Pose point1 = poseFactory.of(24, 6.5, 180);
//    private final Pose point2 = poseFactory.of(10, 6.5, 180);
//    private final Pose point3 = poseFactory.of(24, 32, 90);
//    private final Pose point4 = poseFactory.of(42, 120, 90);
//    private final Pose point5 = poseFactory.of(42, 127.5, 90);
//    private final Pose point6 = poseFactory.of(42, 120, 90);
//    private final Pose point7 = poseFactory.of(30, 60, 90);
//    private final Pose point8 = poseFactory.of(13.5, 46, 180);
//    private final Pose point9 = poseFactory.of(24, 46, 180);
//    private final Pose point10 = poseFactory.of(54, 16, 180);
//    private final Pose point11 = poseFactory.of(12, 94, 0);
//    private final Pose point3Control = poseFactory.of(24, 12, 0);
//    private final Pose point4Control = poseFactory.of(24, 120, 0);
//    private final Pose point7Control = poseFactory.of(30, 120, 0);
//    private final Pose point8Control = poseFactory.of(30, 46, 0);
//    private final Pose point10Control = poseFactory.of(24, 16, 0);
//    private final Pose point11Control = poseFactory.of(12, 16, 0);
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
