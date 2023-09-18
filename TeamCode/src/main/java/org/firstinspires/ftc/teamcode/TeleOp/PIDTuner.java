package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.HardwareSoftware;


@TeleOp(name="PIDF Tuner")
public class PIDTuner extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();

    double kP = 1.3;
    double kI = 0.0;
    double kD = 0.015;
    double kF = 0.0025;

    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    boolean aDown = false;
    boolean bDown = false;
    boolean xDown = false;
    boolean yDown = false;

    @Override
    public void init() {
        robot.init(hardwareMap);



    }

    @Override
    public void loop() {


        telemetry.addData("Front Left Encoder: ", robot.FLdrive().getCurrentPosition());
        telemetry.addData("Front Right Encoder: ", robot.FRdrive().getCurrentPosition());
        telemetry.addData("Back Left Encoder: ", robot.BLdrive().getCurrentPosition());
        telemetry.addData("Back Right Encoder: ", robot.BRdrive().getCurrentPosition());
        telemetry.addData("Position Error: ", robot.pidf.getPositionError());
        telemetry.addData("Velocity Error: ", robot.pidf.getVelocityError());
        telemetry.addData("Robot Heading: ", robot.getHeading());
        telemetry.update();

        if(gamepad1.y){
            yDown = true;
        }
        if(!gamepad1.y && yDown){
            yDown = false;

            robot.pidf.reset();

            robot.FLdrive().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.BLdrive().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.FRdrive().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            robot.BRdrive().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }

        if(gamepad1.a){
            aDown = true;
        }
        if(!gamepad1.a && aDown){
            aDown = false;

            robot.pidDrive(10000, 1000);
        }

        if(gamepad1.b){
            bDown = true;
        }
        if(!gamepad1.b && bDown){
            bDown = false;

            robot.pidDrive(10000, -1000);
        }

        //Manual BreakPoint
        if(gamepad1.x){
            xDown = true;
        }
        if(!gamepad1.x && xDown){
            xDown = false;


            robot.pidf.setPIDF(kP, kI, kD, kF);
            robot.yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
            robot.yawPIDController.enable(true);
        }

    }
}
