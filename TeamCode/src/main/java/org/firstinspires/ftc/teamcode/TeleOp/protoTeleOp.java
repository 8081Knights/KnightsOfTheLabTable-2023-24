package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name="Drive Train Prototype (Non Headless)")
public class protoTeleOp extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();
    @Override
    public void init() {

        robot.init(hardwareMap);
    
    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        robot.FLdrive().setPower(y + x + rx);
        robot.BLdrive().setPower(y - x + rx);
        robot.FRdrive().setPower(y - x - rx);
        robot.BRdrive().setPower(y + x - rx);

    }
}
