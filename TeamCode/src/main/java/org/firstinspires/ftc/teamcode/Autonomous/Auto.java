package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

import java.util.concurrent.TimeUnit;

@Disabled
@Autonomous(name = "Auto", group = "Pushbot")
public class Auto extends LinearOpMode {
    public static HardwareSoftware robot = new HardwareSoftware();

    double ticksPerInch = 41.6666666;

    public int InchConvert(double inches) {
//        double ticksPerInch = 41.6666666;
        return (int) (ticksPerInch * inches);
    }

    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        waitForStart();
        moveForward(96, 10000);

        sleep(100000);



    /*    moveForward(24, 9000);
        sleep(2000);
        moveForward(-24, 9000);
        sleep(2000);
        moveForward(24, 9000);
        sleep(2000);
        moveForward(-24, 9000);
        sleep(2000);
        moveForward(24, 9000);
        sleep(2000);
        moveForward(-24, 9000);
        sleep(2000);
        moveForward(24, 9000);
        sleep(2000);
        moveForward(-24, 9000);*/



    }

//    return(int)(ticksPerInch * inches);
  /* public static int degreeConvert(double degree) {
        double ticksPerDegree = 10.55555;
        return (int) (degree * ticksPerDegree);
    }

    public static int strafeInches(int inches) {
        double ticksPerStrafeInt = 52.631578;
        return (int) (inches * ticksPerStrafeInt);
    }

    static final double FEET_PER_METER = 3.28084;*/

    //this is were there was an if statement for if we saw the tag on interest
    //im not sure what to do here yet

    //////////////////////////////////////////////
//    public void HardwareSoftware() {
//        robot.init(HardwareSoftware);
//        robot.FLdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.FRdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.BLdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        robot.BLdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
    public void moveForward(int inches, int timeout) {
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        timer.startTime();

        robot.FLdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FRdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BLdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BLdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.FLdrive().setTargetPosition(-InchConvert(inches));
        robot.FRdrive().setTargetPosition(InchConvert(inches));
        robot.BLdrive().setTargetPosition(-InchConvert(inches));
        robot.BRdrive().setTargetPosition(InchConvert(inches));

        robot.FRdrive().setVelocity(1000);
        robot.FLdrive().setVelocity(1000);
        robot.BRdrive().setVelocity(1000);
        robot.BLdrive().setVelocity(1000);

        //i dont think this is neccisary
//        while (robot.BLdrive().isBusy() || robot.BRdrive().isBusy() || robot.FLdrive().isBusy() || robot.FRdrive().isBusy()) {
//
//
//        }

        while((robot.BLdrive().getCurrentPosition() <= robot.BLdrive().getTargetPosition() ||
                robot.FLdrive().getCurrentPosition() <= robot.FLdrive().getTargetPosition() ||
                robot.BRdrive().getCurrentPosition() <= robot.BRdrive().getTargetPosition() ||
                robot.FRdrive().getCurrentPosition() <= robot.FRdrive().getTargetPosition()) &&
                timer.time(TimeUnit.MILLISECONDS) < timeout){


            telemetry.addData("Front Left Ticks : ", robot.FLdrive().getCurrentPosition());
            telemetry.addData("Front Left Inches : ", robot.FLdrive().getCurrentPosition() / ticksPerInch);

            telemetry.addData("Front Right Ticks : ", robot.FRdrive().getCurrentPosition());
            telemetry.addData("Front Right Inches : ", robot.FRdrive().getCurrentPosition() / ticksPerInch);

            telemetry.addData("Back Left Ticks : ", robot.BLdrive().getCurrentPosition());
            telemetry.addData("Front Left Inches : ", robot.BLdrive().getCurrentPosition() / ticksPerInch);

            telemetry.addData("Back Right Ticks : ", robot.BRdrive().getCurrentPosition());
            telemetry.addData("Back Right Inches : ", robot.BRdrive().getCurrentPosition() / ticksPerInch);

            telemetry.update();

            sleep(1);



        }
        robot.FRdrive().setPower(0);
        robot.FLdrive().setPower(0);
        robot.BRdrive().setPower(0);
        robot.BRdrive().setPower(0);

        telemetry.addLine("Drive Complete");
        telemetry.update();

        sleep(10);

    }
    public void spin(double degree) {
        robot.FRdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FLdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      /*  robot.FLdrive().setTargetPosition(degreeConvert(degree));
        robot.FRdrive().setTargetPosition(-degreeConvert(degree));
        robot.BLdrive().setTargetPosition(degreeConvert(degree));
        robot.BRdrive().setTargetPosition(-degreeConvert(degree));*/

        robot.FRdrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.FLdrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BRdrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.BRdrive().setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.FRdrive().setVelocity(1000);
        robot.FLdrive().setVelocity(1000);
        robot.BRdrive().setVelocity(1000);
        robot.BRdrive().setVelocity(1000);

        while (robot.BLdrive().isBusy() || robot.BRdrive().isBusy() || robot.FLdrive().isBusy() || robot.FRdrive().isBusy()) {

        }

        robot.FRdrive().setVelocity(0);
        robot.FLdrive().setVelocity(0);
        robot.BRdrive().setVelocity(0);
        robot.BRdrive().setVelocity(0);
    }
    public void strafeRight(int inches) {
        robot.FRdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.FLdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.BRdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

      /*  robot.FLdrive().setTargetPosition(strafeInches(inches));
        robot.FRdrive().setTargetPosition(-strafeInches(inches));
        robot.BLdrive().setTargetPosition(strafeInches(inches));
        robot.BRdrive().setTargetPosition(-strafeInches(inches));
*/
        robot.FRdrive().setVelocity(1000);
        robot.FLdrive().setVelocity(1000);
        robot.BRdrive().setVelocity(1000);
        robot.BLdrive().setVelocity(1000);

        while (robot.BLdrive().isBusy() || robot.BRdrive().isBusy() || robot.FLdrive().isBusy() || robot.FRdrive().isBusy()) {

        }
        robot.FRdrive().setVelocity(0);
        robot.FLdrive().setVelocity(0);
        robot.BRdrive().setVelocity(0);
        robot.BRdrive().setVelocity(0);
    }
}
