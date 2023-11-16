package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class HardwareSoftware {

    private HardwareMap hw = null;
    DcMotorEx frontRight    = null;
    DcMotorEx backRight     = null;
    DcMotorEx backLeft      = null;
    DcMotorEx frontLeft     = null;

    DcMotorEx intake;

    DcMotorEx linearSlide = null;


    Servo pixelServo = null;


    private List<DcMotorEx> motors;


    public navXPIDController yawPIDController;
    navXPIDController.PIDResult yawPIDResult;



    //Gyro PIDF Tuning Constants
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private final double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;



    //Motor PIDF Controller Tuning Constants
    double kP = 1.3;
    double kI = 0.0;
    double kD = 0.015;
    double kF = 0.0025;

    public PIDFController pidf = new PIDFController(kP, kI, kD, kF);


    int maxVelocity = 2000;

    AHRS gyro = null;


    public void init(HardwareMap ahw){


        hw= ahw;


        //Get navX micro gyro from hardware map
        gyro = AHRS.getInstance(hw.get(NavxMicroNavigationSensor.class, "gyro"), AHRS.DeviceDataType.kProcessedData);

        gyro.zeroYaw();


        /* Create a PID Controller which uses the Yaw Angle as input. */
        yawPIDController = new navXPIDController(gyro,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);




        //Grab and configure drivetrain
        frontLeft = hw.get(DcMotorEx.class, "FLdrive");
        frontRight = hw.get(DcMotorEx.class, "FRdrive");
        backLeft = hw.get(DcMotorEx.class, "BLdrive");
        backRight = hw.get(DcMotorEx.class, "BRdrive");


        linearSlide = hw.get(DcMotorEx.class, "linearSlide");



        frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        motors = Arrays.asList(frontLeft, backLeft, backRight, frontRight);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        intake = hw.get(DcMotorEx.class, "intake");

        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        motors = Arrays.asList(frontLeft, backLeft, backRight, frontRight);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        intake = hw.get(DcMotorEx.class, "intake");

        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        pixelServo = hw.get(Servo.class, "pixelServo");



    }


    public float getHeading(){
        return gyro.getYaw();
    }

    public void resetGyro(){
        gyro.zeroYaw();
    }

    //Return Methods
    public DcMotorEx FLdrive(){
        return frontLeft;
    }

    public DcMotorEx FRdrive(){
        return frontRight;
    }

    public DcMotorEx BLdrive(){
        return backLeft;
    }

    public DcMotorEx BRdrive(){
        return backRight;
    }

    public DcMotorEx intake(){
        return intake;
    }



    public DcMotorEx linearSlide() {return linearSlide;}

    public Servo pixelServo() {return pixelServo;}

//    public void resetOdometry(){
//        frontEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//    }

    public void runSlides(int target, int velocity){
        linearSlide.setTargetPosition(target);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setVelocity(velocity);

    }

    public AHRS gyro(){ return gyro;}


    public void resetEncoders(){
        FRdrive().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        FLdrive().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BRdrive().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        BLdrive().setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    }
    double ticksPerInch = 41.6666666;

    public int InchConvert(double inches) {
//        double ticksPerInch = 41.6666666;
        return (int) (ticksPerInch * inches);
    }
    public double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }

    public void driveStraight(double targetHeading, int velocity, double distance, double timeout){
        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        timer.startTime();

        FLdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRdrive().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FLdrive().setTargetPosition(-InchConvert(distance));
        FRdrive().setTargetPosition(InchConvert(distance));
        BLdrive().setTargetPosition(-InchConvert(distance));
        BRdrive().setTargetPosition(InchConvert(distance));

        while((BLdrive().getCurrentPosition() <= BLdrive().getTargetPosition() ||
                FLdrive().getCurrentPosition() <= FLdrive().getTargetPosition() ||
                BRdrive().getCurrentPosition() <= BRdrive().getTargetPosition() ||
                FRdrive().getCurrentPosition() <= FRdrive().getTargetPosition()) &&
                timer.time(TimeUnit.MILLISECONDS) < timeout){
            /* Wait for new Yaw PID output values, then update the motors
               with the new PID value with each new output value.
             */

            /* Drive straight forward at 1/2 of full drive speed */


            if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                if (yawPIDResult.isOnTarget()) {
                    FLdrive().setVelocity(velocity);
                    BLdrive().setVelocity(velocity);
                    FRdrive().setVelocity(velocity);
                    BRdrive().setVelocity(velocity);

                } else {
                    double output = yawPIDResult.getOutput();
                    FLdrive().setVelocity(limit(velocity + (int)maxVelocity*output));
                    BLdrive().setVelocity(limit(velocity + (int)maxVelocity*output));
                    FRdrive().setVelocity(limit(velocity - (int)maxVelocity*output));
                    BRdrive().setVelocity(limit(velocity - (int)maxVelocity*output));

                }
            } else {
                /* No sensor update has been received since the last time  */
                /* the loop() function was invoked.  Therefore, there's no */
                /* need to update the motors at this time.                 */
            }


        }

        FLdrive().setPower(0);
        BLdrive().setPower(0);
        FRdrive().setPower(0);
        BRdrive().setPower(0);



    }

    public void pidDrive(double timeOut, int target){
        // Calculates the output of the PIDF algorithm based on sensor
// readings. Requires both the measured value
// and the desired setpoint

        ElapsedTime timer = new ElapsedTime();

        timer.reset();
        timer.startTime();

        /*
         * A sample control loop for a motor
         */


        // We set the setpoint here.
        // Now we don't have to declare the setpoint
        // in our calculate() method arguments.
        pidf.setSetPoint(target);
        pidf.setTolerance(25, 1);


        // perform the control loop
                /*
                 * The loop checks to see if the controller has reached
                 * the desired setpoint within a specified tolerance
                 * range
                 */
        while (!pidf.atSetPoint() && timer.time(TimeUnit.MILLISECONDS) <= timeOut) {
            double FLoutput = pidf.calculate(
                    FLdrive().getCurrentPosition()
            );

            double FRoutput = pidf.calculate(
                    FRdrive().getCurrentPosition()
            );

            double BLoutput = pidf.calculate(
                    BLdrive().getCurrentPosition()
            );

            double BRoutput = pidf.calculate(
                    BRdrive().getCurrentPosition()
            );

            FLdrive().setVelocity(FLoutput);
            BLdrive().setVelocity(BLoutput);
            FRdrive().setVelocity(FRoutput);
            BRdrive().setVelocity(BRoutput);


        }


        FLdrive().setPower(0);
        BLdrive().setPower(0);
        FRdrive().setPower(0);
        BRdrive().setPower(0);
    }



}
