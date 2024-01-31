package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

import kotlin.properties.ObservableProperty;


@TeleOp(name="Bumper Touch Test")
public class bumperTouchTest extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();

    @Override
    public void init() {

        robot.init(hardwareMap);

    }

    @Override
    public void loop() {

        telemetry.addData("Both Pressed: ", robot.isBackDrop());
        telemetry.addData("Left Is Pressed: ", robot.bumperTouchLeft().getState());
        telemetry.addData("Right is Pressed: ", robot.bumperTouchRight().getState());


        telemetry.update();

    }
}
