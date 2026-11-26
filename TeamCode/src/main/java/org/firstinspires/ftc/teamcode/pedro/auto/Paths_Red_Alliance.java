package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Paths_Red_Alliance {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.0325, 135.0309, 90);
    private final Pose shootpose1 = poseFactory.of(59.3289, 108.5068, 180);
    private final Pose flowerposeStart = poseFactory.of(59.3289, 108.5068, 180);
    private final Pose flowerpose = poseFactory.of(46.5579, 131.5298, 90);
    private final Pose shootpose2Start = poseFactory.of(46.5579, 131.5298, 0);
    private final Pose shootpose2 = poseFactory.of(59.2156, 108.9091, 0);
    private final Pose parkPose = poseFactory.of(9.1964, 117.3823, -9.6146);

    public Path shootpose1() {
        return Paths.line(start, shootpose1).linear(start, shootpose1);
    }

    public Path flowerpose() {
        return Paths.line(flowerposeStart, flowerpose).linear(flowerposeStart, flowerpose);
    }

    public Path shootpose2() {
        return Paths.line(shootpose2Start, shootpose2).linear(shootpose2Start, shootpose2);
    }

    public Path parkPose() {
        return Paths.line(shootpose2, parkPose).reverseTangent();
    }
}
