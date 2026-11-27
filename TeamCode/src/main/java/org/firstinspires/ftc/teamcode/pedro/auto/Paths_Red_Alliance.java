package org.firstinspires.ftc.teamcode.pedro.auto;

import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class Paths_Red_Alliance {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(60.0325, 135.0309, 0);
    private final Pose shootpose1 = poseFactory.of(59.3289, 123.96, 0);
    private final Pose flowerpose = poseFactory.of(49, 120, 90);
    private final Pose flowerpose2 = poseFactory.of(49, 121, 90);
    private final Pose shootpose2 = poseFactory.of(59.3289, 123.96, 0);
    private final Pose parkPose = poseFactory.of(9.1964, 117.3823, 0);

    public Pose startPose(){ return start; }

    public Path shootpose1() {
        return Paths.line(start, shootpose1).linear(start, shootpose1);
    }

    public Path flowerpose() {
        return Paths.line(shootpose1, flowerpose).linear(shootpose1, flowerpose);
    }

    public Path flowerpick(){
        return Paths.line(flowerpose, flowerpose2).linear(flowerpose, flowerpose2);
    }

    public Path shootpose2() {
        return Paths.line(flowerpose2, shootpose2).linear(flowerpose2, shootpose2);    }

    public Path parkPose() {
        return Paths.line(shootpose2, parkPose).linear(shootpose2, parkPose);
    }

    public Path route(){return path(shootpose1(),flowerpose(),flowerpick(),shootpose2(),parkPose());}
}
