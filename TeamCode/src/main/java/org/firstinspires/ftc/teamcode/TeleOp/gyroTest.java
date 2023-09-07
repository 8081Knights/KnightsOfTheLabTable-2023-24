package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

import java.text.DecimalFormat;


@TeleOp(name="Gyro Testing")
public class gyroTest extends OpMode {


    HardwareSoftware robot = new HardwareSoftware();

    @Override
    public void init() {

        robot.init(hardwareMap);

    }

    @Override
    public void stop(){
        robot.gyro().close();
    }

    @Override
    public void loop() {

        boolean connected = robot.gyro().isConnected();

        telemetry.addData("Gyro Status: ", connected ? "Connected":"Disconnected");

        String gyrocal, magcal, yaw, pitch, roll, compass_heading;
        String fused_heading, ypr, cf, motion;
        DecimalFormat df = new DecimalFormat("#.##");

        if ( connected ) {
            gyrocal = (robot.gyro().isCalibrating() ?
                    "CALIBRATING" : "Calibration Complete");
            magcal = (robot.gyro().isMagnetometerCalibrated() ?
                    "Calibrated" : "UNCALIBRATED");
            yaw = df.format(robot.gyro().getYaw());
            pitch = df.format(robot.gyro().getPitch());
            roll = df.format(robot.gyro().getRoll());
            ypr = yaw + ", " + pitch + ", " + roll;
            compass_heading = df.format(robot.gyro().getCompassHeading());
            fused_heading = df.format(robot.gyro().getFusedHeading());
            if (!robot.gyro().isMagnetometerCalibrated()) {
                compass_heading = "-------";
            }
            cf = compass_heading + ", " + fused_heading;
            if ( robot.gyro().isMagneticDisturbance()) {
                cf += " (Mag. Disturbance)";
            }
            motion = (robot.gyro().isMoving() ? "Moving" : "Not Moving");
            if ( robot.gyro().isRotating() ) {
                motion += ", Rotating";
            }
        } else {
            gyrocal =
                    magcal =
                            ypr =
                                    cf =
                                            motion = "-------";
        }
        telemetry.addData("2 GyroAccel", gyrocal );
        telemetry.addData("3 Y,P,R", ypr);
        telemetry.addData("4 Magnetometer", magcal );
        telemetry.addData("5 Compass,9Axis", cf );
        telemetry.addData("6 Motion", motion);





    }
}
