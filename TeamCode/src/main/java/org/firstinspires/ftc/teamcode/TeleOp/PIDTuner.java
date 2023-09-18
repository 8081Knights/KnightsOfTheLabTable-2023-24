package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name="PIDF Tuner")
public class PIDTuner extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();

    boolean aDown = false;
    boolean bDown = false;
    boolean xDown = false;

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

            robot.pidDrive(0.5, 1000);
        }

        if(gamepad1.b){
            bDown = true;
        }
        if(!gamepad1.a && aDown){
            bDown = false;

            robot.pidDrive(0.5, -1000);
        }

        if(gamepad1.x){
            xDown = true;
        }
        if(!gamepad1.x && xDown){
            xDown = false;
            try {
                sleep(1);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

    }
}
