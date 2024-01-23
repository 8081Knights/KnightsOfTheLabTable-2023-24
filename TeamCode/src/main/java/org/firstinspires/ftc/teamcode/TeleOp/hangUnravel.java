package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name = "Hang Unravel")
public class hangUnravel extends OpMode {
    HardwareSoftware robot = new HardwareSoftware();
    @Override
    public void init() {
        robot.init(hardwareMap);

    }

    @Override
    public void loop() {
        if(gamepad1.right_trigger>0.1){
            robot.hangRight().setPower(-gamepad1.right_trigger);
        }
        else if(gamepad1.left_trigger>0.1){
            robot.hangLeft().setPower(-gamepad1.left_trigger);
        }
        else{
            robot.hang(0);
        }

    }
}
