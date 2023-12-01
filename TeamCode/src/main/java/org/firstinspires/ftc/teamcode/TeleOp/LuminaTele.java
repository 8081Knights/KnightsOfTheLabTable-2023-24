package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name = "3. Lumina Tele")
public class LuminaTele extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();

    boolean g1aDown = false;
    double maxSpeed = 1;
    double minSpeed = 0.3;
    double speedMult = minSpeed;

    boolean g1bDown = false;
    boolean g1xDown = false;


    //TODO: Tune
    int maxSlideTarget = -2000;
    int minSlideTarget = -25;
    int slideTarget = minSlideTarget;
    int slideVelocity = 2000;

    double MAX_INTAKE_SPEED = 0.7;



    @Override
    public void init() {

        robot.init(hardwareMap);


    }

    @Override
    public void loop() {

        //Turn Variable for Headless Robot Logic
        double driveTurn = gamepad1.right_stick_x;
        //driveVertical = -gamepad1.right_stick_y;
        //driveHorizontal = gamepad1.right_stick_x;

        //Drive X and Y for Headless
        double gamepadXCoordinate = gamepad1.left_stick_x; //this simply gives our x value relative to the driver
        double gamepadYCoordinate = -gamepad1.left_stick_y; //this simply gives our y value relative to the driver



        //the inverse tangent of opposite/adjacent gives us our gamepad degree
        double robotDegree = 0;


        //Final X and Y for corrected driving (Field Centric Drive Logic)
        double rotX = gamepadXCoordinate * Math.cos(robotDegree) - gamepadYCoordinate * Math.sin(robotDegree);
        double rotY = gamepadXCoordinate * Math.sin(robotDegree) + gamepadYCoordinate * Math.cos(robotDegree);


        //Denominator makes sure the motors are never set past 1 power
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(driveTurn), 1);

        //Power Variables
        double frontLeftPower = ((rotY + rotX + driveTurn) / denominator)*speedMult;
        double backLeftPower = ((rotY - rotX + driveTurn) / denominator)*speedMult;
        double frontRightPower = ((rotY - rotX - driveTurn) / denominator)*speedMult;
        double backRightPower = ((rotY + rotX - driveTurn) / denominator)*speedMult;

        //Set Power to Motors
        robot.FRdrive().setPower(frontRightPower);
        robot.FLdrive().setPower(frontLeftPower);
        robot.BRdrive().setPower(backRightPower);
        robot.BLdrive().setPower(backLeftPower);

        if(gamepad1.right_trigger > 0.1){
            robot.intake().setPower(-gamepad1.right_trigger*MAX_INTAKE_SPEED);
        }

        else if(gamepad1.left_trigger > 0.1){
            robot.intake().setPower(gamepad1.left_trigger*MAX_INTAKE_SPEED);
        }
        else if(gamepad1.right_trigger < 0.1){
            robot.intake().setPower(0);
        }

        else if(gamepad1.left_trigger < 0.1){
            robot.intake().setPower(0);
        }



    }
}
