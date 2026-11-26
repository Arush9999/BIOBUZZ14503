package org.firstinspires.ftc.teamcode.pedro.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BIOBUZZ TeleOp Red", group = "BIOBUZZ")
public class BIOBUZZ_TeleOp_Red extends BIOBUZZ_TeleOp {

    @Override
    protected boolean isRedAlliance() {
        return true;
    }
}
