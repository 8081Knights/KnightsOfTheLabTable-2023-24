package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

@TeleOp(name = "1. Keaton Competition Tele")
public class CompTeleKeaton extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();

    boolean g1aDown = false;
    double maxSpeed = 1;
    double minSpeed = 0.3;
    double speedMult = maxSpeed;

    boolean g1bDown = false;
    boolean g1xDown = false;
    boolean g2aDown = false;
    boolean g2yDown = false;
    boolean g2xDown = false;
    boolean g2bDown = false;

    boolean runslide = true;




    //TODO: Tune
    int maxSlideTarget = -2000;
    int minSlideTarget = -25;
    int slideTarget = minSlideTarget;
    int slideVelocity = 2000;
    int slideMin = 50;

    double MAX_INTAKE_SPEED = 0.7;



    @Override
    public void init() {

        robot.init(hardwareMap);

        robot.dronelunch().setPosition(1);


    }

    @Override
    public void loop() {

        //Turn Variable for Headless Robot Logic
        double driveTurn = -gamepad1.right_stick_x;
        //driveVertical = -gamepad1.right_stick_y;
        //driveHorizontal = gamepad1.right_stick_x;

        //Drive X and Y for Headless
        double gamepadXCoordinate = gamepad1.left_stick_x; //this simply gives our x value relative to the driver
        double gamepadYCoordinate = -gamepad1.left_stick_y; //this simply gives our y value relative to the driver



        //the inverse tangent of opposite/adjacent gives us our gamepad degree
        double robotDegree = Math.toRadians(robot.getHeading());


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

        if(gamepad1.dpad_up){
            slideTarget = maxSlideTarget;

        }
        else if(gamepad1.dpad_down){
            slideTarget = minSlideTarget;
            //  robot.pixelServo().setPosition(0);
        }

        if(gamepad1.x){
            robot.pixelServo().setPosition(1);
            g1xDown = true;
        }
        else if(!gamepad1.x && g1xDown) {
            robot.pixelServo().setPosition(0.5);

            g1xDown = false;
        }
        if(gamepad1.b){
            robot.pixelServo().setPosition(0);
            g1bDown = true;
        }
        else if(!gamepad1.b && g1bDown) {
            robot.pixelServo().setPosition(0.5);

            g1bDown = false;
        }



        if (gamepad1.a){
            g1aDown = true;
        }
        if(!gamepad1.a && g1aDown){
            g1aDown = false;

            if(speedMult==maxSpeed){
                speedMult=minSpeed;
            }
            else{
                speedMult=maxSpeed;
            }
        }
        if (gamepad2.a){
            g2aDown = true;
        }
        if(!gamepad2.a && g2aDown){
            g2aDown = false;
            robot.gyro().zeroYaw();
        }

        if (gamepad2.y){
            g2yDown = true;
        }
        if(!gamepad2.y && g2yDown){
            g2yDown = false;
            if(robot.dronelunch().getPosition() == 0){
                robot.dronelunch().setPosition(1 );
            }
            else{
                robot.dronelunch().setPosition(0);
            }
        }

        if (gamepad2.x){
            g2xDown = true;
        }
        if(!gamepad2.x && g2xDown){
            g2xDown = false;


            if(runslide){
                runslide = false;
                robot.linearSlide().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                telemetry.addLine("Stopped Slides");
                telemetry.update();
            }
            else{
                runslide = true;
                robot.linearSlide().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                telemetry.addLine("Running Slides");
                telemetry.update();

            }
        }


        if (gamepad2.b){
            g2bDown = true;
        }
        if(!gamepad2.b && g2bDown){
            g2bDown = false;


            robot.linearSlide().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addLine("Reset SLides");
            telemetry.update();
        }




        if(runslide){
            if(Math.abs(robot.linearSlide().getCurrentPosition()) < slideMin && Math.abs(slideTarget) < minSlideTarget){
                robot.linearSlide().setPower(0);

            }
            else{
                robot.runSlides(slideTarget, slideVelocity);
            }

        }
        else{

            if(gamepad2.right_trigger > 0.1) {
                robot.linearSlide().setPower(gamepad2.right_trigger*0.5);
                telemetry.addLine("Running Slides Downward without Encoder");
                telemetry.update();

            }
            else{
                robot.linearSlide().setPower(0);
            }

        }
//        if(Math.abs(robot.linearSlide().getCurrentPosition()) < 100 && Math.abs(slideTarget) < 50 && runslide){
//            robot.linearSlide().setPower(0);
//            telemetry.addLine("Slides at Rest");
//            telemetry.update();
//
//        }
//        else if(Math.abs(robot.linearSlide().getCurrentPosition()) > 100 && Math.abs(slideTarget) > 50 && runslide){
//            robot.runSlides(slideTarget, slideVelocity);
//
//            telemetry.addLine("Running SLidess");
//            telemetry.update();
//        }






//        telemetry.addData("Linear Slide Position: " , robot.linearSlide().getCurrentPosition());

    }
}
