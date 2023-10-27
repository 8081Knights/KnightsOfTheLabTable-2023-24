package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.CameraLogicRed;
import org.firstinspires.ftc.teamcode.Autonomous.CameraLogicBlue;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="AutoRed")
public class AutoRed extends LinearOpMode  {
    OpenCvWebcam cam;
    HardwareSoftware robot = new HardwareSoftware();
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);




        String pos = "";

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        cam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        CameraLogicRed detector = new CameraLogicRed(telemetry);
        cam.setPipeline(detector);

        cam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }

        });


        //begin calibrating the gyro
//        telemetry.log().add("Gyro Calibrating. Do Not Move!");
//        robot.gyro().isCalibrating();
//
//        // Wait until the gyro calibration is complete
//        timer.reset();
//        while (!isStopRequested() && robot.gyro().isCalibrating()) {
//            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
//            telemetry.update();
//            sleep(50);
//        }
//
//        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
//        telemetry.clear(); telemetry.update();



        waitForStart();


        //Adds data for the position of the ducky on the barcode
        telemetry.addLine("Position: " + pos);
        telemetry.update();
        telemetry.clearAll();



        //switch case for the delivery the freight to the corresponding level on the wobble goal thingy
        switch (pos) {

            case "RIGHT":
                robot.FRdrive().setPower(1);
                robot.FLdrive().setPower(1);
                robot.BRdrive().setPower(1);
                robot.BLdrive().setPower(1);


                break;

            case "LEFT":
                robot.FRdrive().setPower(-1);
                robot.FLdrive().setPower(-1);
                robot.BRdrive().setPower(-1);
                robot.BLdrive().setPower(-1);


                break;

            case "MIDDLE":


                break;

            default:


                break;


        }
    }
    //parks in the box
//                commands.Drive(15,1500,2);
//                sleep(100);
//                commands.GyroTurn(40, 1300);
//                sleep(100);
//                commands.Drive(-28, 1500, 3);



}