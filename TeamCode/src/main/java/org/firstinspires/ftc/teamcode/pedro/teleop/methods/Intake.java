package org.firstinspires.ftc.teamcode.pedro.teleop.methods;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.teleop.constants.Constants;

public class Intake {

    private final DcMotor intake;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run(double forwardTrigger, double reverseTrigger) {
        if (forwardTrigger > Constants.INTAKE_TRIGGER_DEADZONE) {
            setPower(Constants.INTAKE_POWER * forwardTrigger);
        } else if (reverseTrigger > Constants.INTAKE_TRIGGER_DEADZONE) {
            setPower(-Constants.INTAKE_POWER * reverseTrigger);
        } else {
            stop();
        }
    }

    public void setPower(double power) {
        intake.setPower(power);
    }

    public void stop() {
        intake.setPower(0);
    }

    public DcMotor getMotor() {
        return intake;
    }
}
