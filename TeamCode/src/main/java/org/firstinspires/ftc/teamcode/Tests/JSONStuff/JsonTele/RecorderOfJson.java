package org.firstinspires.ftc.teamcode.Tests.JSONStuff.JsonTele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

public class RecorderOfJson extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();
    double maxSpeed = .8;
    double minSpeed = 0.3;
    double speedMult = maxSpeed;
    boolean g1aDown = false;
    @Override
    public void init() {
        robot.init(hardwareMap);
    }
    double driveTurn;
    double gamepadXCoordinate;
    double gamepadYCoordinate;
    double robotDegree;

    double rotX;
    double rotY;
    double denominator;


    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;

    @Override
    public void loop() {
        driveTurn = -gamepad1.right_stick_x;
        gamepadXCoordinate = gamepad1.left_stick_x; //this simply gives our x value relative to the driver
        gamepadYCoordinate = -gamepad1.left_stick_y; //this simply gives our y vaue relative to the driver

        robotDegree = Math.toRadians(robot.getHeading());

        rotX = gamepadXCoordinate * Math.cos(robotDegree) - gamepadYCoordinate * Math.sin(robotDegree);
        rotY = gamepadXCoordinate * Math.sin(robotDegree) + gamepadYCoordinate * Math.cos(robotDegree);
        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(driveTurn), 1);

        frontLeftPower = ((rotY + rotX - driveTurn) / denominator)*speedMult;
        backLeftPower = ((rotY - rotX - driveTurn) / denominator)*speedMult;
        frontRightPower = ((rotY - rotX + driveTurn) / denominator)*speedMult;
        backRightPower = ((rotY + rotX + driveTurn) / denominator)*speedMult;

        robot.FRdrive().setPower(frontRightPower);
        robot.FLdrive().setPower(frontLeftPower);
        robot.BRdrive().setPower(backRightPower);
        robot.BLdrive().setPower(backLeftPower);

        if(gamepad1.a){
            g1aDown = true;
        }
        if(!gamepad1.a && g1aDown){
            g1aDown = false;
            if(speedMult==maxSpeed){
                speedMult=minSpeed;
            }
            else{
                speedMult = maxSpeed;
            }
        }

    }
}
