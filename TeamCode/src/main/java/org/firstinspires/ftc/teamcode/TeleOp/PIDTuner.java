package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name="PIDF Tuner")
public class PIDTuner extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();

    boolean aDown = false;
    boolean bDown = false;

    @Override
    public void init() {
        robot.init(hardwareMap);

    }

    @Override
    public void loop() {
        if(gamepad1.a){
            aDown = true;
        }
        if(!gamepad1.a && aDown){
            aDown = false;

            //robot.
        }

    }
}
