package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

public class Turret {

    private final Servo LT;
    private final Servo RT;
    private double position;

    public Turret(HardwareMap hardwareMap) {
        LT = hardwareMap.get(Servo.class, "LT");
        RT = hardwareMap.get(Servo.class, "RT");
        position = Constants.TURRET_CENTER;
        setPosition(position);
    }

    public void run(double stick) {
        if (Math.abs(stick) < Constants.TURRET_STICK_DEADZONE) {
            return;
        }

        setPosition(position + stick * Constants.TURRET_SPEED);
    }

    public void setPosition(double target) {
        position = Range.clip(target, Constants.TURRET_MIN, Constants.TURRET_MAX);

        LT.setPosition(position);
        RT.setPosition(Constants.TURRET_CENTER * 2.0 - position);
    }

    public void center() {
        setPosition(Constants.TURRET_CENTER);
    }

    public double getPosition() {
        return position;
    }

    public Servo getLT() {
        return LT;
    }

    public Servo getRT() {
        return RT;
    }
}
