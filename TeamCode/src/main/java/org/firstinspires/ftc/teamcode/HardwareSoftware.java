package org.firstinspires.ftc.teamcode;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareSoftware {

    private HardwareMap hw = null;
    DcMotorEx frontRight    = null;
    DcMotorEx backRight     = null;
    DcMotorEx backLeft      = null;
    DcMotorEx frontLeft     = null;

    AHRS gyro = null;


    public void init(HardwareMap ahw){


        hw= ahw;


        //Get navX micro gyro from hardware map
        gyro = AHRS.getInstance(hw.get(NavxMicroNavigationSensor.class, "gyro"), AHRS.DeviceDataType.kProcessedData);

        gyro.zeroYaw();




        //Grab and configure drivetrain
        frontLeft = hw.get(DcMotorEx.class, "FLdrive");
        frontRight = hw.get(DcMotorEx.class, "FRdrive");
        backLeft = hw.get(DcMotorEx.class, "BLdrive");
        backRight = hw.get(DcMotorEx.class, "BRdrive");

        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
//

        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);



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

    public AHRS gyro(){ return gyro;}


}
