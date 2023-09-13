package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

public class driveStraightTest extends LinearOpMode {

    HardwareSoftware robot = new HardwareSoftware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while(robot.gyro().isCalibrating()){
            telemetry.addLine("Robot is Calibrating");
            telemetry.update();
        }

        telemetry.clear();

        waitForStart();

        robot.driveStraight(0, 1000, 30, 2000);

        sleep(1000000);

    }
}
