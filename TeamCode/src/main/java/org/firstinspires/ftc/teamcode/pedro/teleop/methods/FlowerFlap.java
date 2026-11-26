package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

/** Flower intake flap. Starts up; toggle goes down if not already there, else up. */
public class FlowerFlap {

    private final Servo flowerFlap;
    private double position;

    public FlowerFlap(HardwareMap hardwareMap) {
        flowerFlap = hardwareMap.get(Servo.class, "flowerflap");
        setPosition(Constants.FLOWER_FLAP_UP);
    }

    public void setPosition(double requested) {
        position = requested;
        flowerFlap.setPosition(position);
    }

    public void up() {
        setPosition(Constants.FLOWER_FLAP_UP);
    }

    public void down() {
        setPosition(Constants.FLOWER_FLAP_DOWN);
    }

    public void toggle() {
        if (Math.abs(position - Constants.FLOWER_FLAP_DOWN) < Constants.FLOWER_FLAP_TOLERANCE) {
            up();
        } else {
            down();
        }
    }

    public boolean isDown() {
        return Math.abs(position - Constants.FLOWER_FLAP_DOWN) < Constants.FLOWER_FLAP_TOLERANCE;
    }

    public double getPosition() {
        return position;
    }

    public Servo getServo() {
        return flowerFlap;
    }

    public Command toggleCommand() {
        return instant(this::toggle);
    }

    public Command upCommand() {
        return instant(this::up);
    }

    public Command downCommand() {
        return instant(this::down);
    }
}
