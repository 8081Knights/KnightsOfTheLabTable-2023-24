package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.CameraLogicRed;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class Webcam {

    OpenCvWebcam cam = null;
    Telemetry telemetry;
    int cameraMonitorViewId;
    String position;
    Mat mat = new Mat();
    private HardwareMap hardwareMap = null;
    CameraLogicRed Detector = new CameraLogicRed(telemetry);


    public void init(HardwareMap awhMap) {
        hardwareMap = awhMap;
        cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        cam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
    }



    public void RunCamera(Telemetry t) {

        CameraLogicRed detector = new CameraLogicRed(t);
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
//        position = detector.checkPos();
        t.addData("Position", detector.checkPos());
        t.update();


    }

//    public ColorRangeTest.POSITION getPosition(){
//       return Detector.checkPos();
//
//    }
}