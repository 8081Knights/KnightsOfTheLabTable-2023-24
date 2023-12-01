package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvPipeline;

//@Disabled
//Looks for Yellow Box thing i think
public class CameraLogicRed extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();
    OpenCvCamera cam;
    public String position = "opps no value";


    public enum POSITION {
        RIGHT,
        LEFT,
        MIDDLE,
        NONE
    }

    public POSITION Position;

    public CameraLogicRed(Telemetry t) { telemetry = t; }
    boolean viewportPaused;

    // Regions of interests -------------------------//

    // One half og the screen
    static final Rect MIDDLE_ROI = new Rect(
            new Point(10,110), // Max x value = 320
            new Point(110, 190)
    );

    //The other half of the Screen
    static final Rect RIGHT_ROI = new Rect(
            new Point(190,150), // Max x value = 320
            new Point(300, 240)
    );

//    // Not important
//    static final Rect LEFT_ROI = new Rect(
//            new Point(320,100), // Max x value = 320
//            new Point(75, 20)
//    );

    static double PERCENT_COLOR_THRESHOLD = 0.1;

    @Override
    public Mat processFrame(Mat input){

        //Converting RGB image to HSV in order to easily look for color ranges
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //Low Color End
        Scalar lowHsv = new Scalar(0, 60, 20); //0, 51, 153

        //High Color End
        Scalar highHsv = new Scalar(255, 255, 255); //179, 171, 255

        Core.inRange(mat, lowHsv, highHsv, mat);

        Mat box = mat.submat(MIDDLE_ROI);
        Mat box2 = mat.submat(RIGHT_ROI);

        double Value = Core.sumElems(box).val[0] / MIDDLE_ROI.area() / 255;
        double RightValue = Core.sumElems(box2).val[0] / RIGHT_ROI.area() / 255;

        box.release();

        boolean box_Right = RightValue > PERCENT_COLOR_THRESHOLD;
        boolean box_Middle = Value > PERCENT_COLOR_THRESHOLD;



        if (box_Right && box_Middle) {
             Position = POSITION.NONE;
            position = "NONE";
            telemetry.addData("Position",POSITION.NONE);

        } else if (box_Right) {
            //  Position = POSITION.MIDDLE;
            position = "RIGHT";
            telemetry.addData("Position",POSITION.RIGHT);

        } else if (box_Middle) {
            // Position = POSITION.RIGHT;
            position = "MIDDLE";
            telemetry.addData("Position",POSITION.MIDDLE);
        } else {
            //Position = POSITION.NONE;
            position = "LEFT";
            telemetry.addData("Position",POSITION.LEFT);
        }

        telemetry.update();

        Scalar boxColor = new Scalar(255, 0, 0);

        // Convert grayscale back to RGB
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_GRAY2RGB);
        Imgproc.rectangle(mat, RIGHT_ROI, boxColor);
        Imgproc.rectangle(mat, MIDDLE_ROI, boxColor);
        return mat;

    }

    @Override
    public void onViewportTapped() {

        viewportPaused = !viewportPaused;

        if(viewportPaused) {
            cam.pauseViewport();
        }
        else {
            cam.resumeViewport();
        }
    }

    public String checkPos(){
        return position;
    }



}

