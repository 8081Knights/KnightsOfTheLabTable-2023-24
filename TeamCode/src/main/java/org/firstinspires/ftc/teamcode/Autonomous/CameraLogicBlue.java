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


// code written by Gavin Kuehn and Alex Rumer of Team 8081 The Knights of the Lab Table

@Disabled
//Looks for Yellow Box thing i think
public class CameraLogicBlue extends OpenCvPipeline {
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

    public CameraLogicBlue(Telemetry t) { telemetry = t; }
    boolean viewportPaused;

    // Regions of interests -------------------------//

    // One half oF the screen
    static final Rect MIDDLE_ROI = new Rect(
            new Point(60,70), // Max x value = 320
            new Point(150, 150)
    );

    //The other half of the Screen
    static final Rect RIGHT_ROI = new Rect(
            new Point(200,20), // Max x value = 320
            new Point(320, 100)
    );

    // Not important
    static final Rect LEFT_ROI = new Rect(
            new Point(320,90), // Max x value = 320
            new Point(75, 10)
    );

    static double MIDDLE_THRESHOLD = 0.02;
    static double RIGHT_THRESHOLD = 0.017;

    @Override
    public Mat processFrame(Mat input){

        //Converting RGB image to HSV in order to easily look for color ranges
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

        //Low Color End
        Scalar lowHsv = new Scalar(30, 45, 45); //35, 51, 51

        //High Color End
        Scalar highHsv = new Scalar(163, 255, 255); //163, 255, 255

        Core.inRange(mat, lowHsv, highHsv, mat);

        Mat box = mat.submat(MIDDLE_ROI);
        Mat box2 = mat.submat(RIGHT_ROI);

        double Value = Core.sumElems(box).val[0] / MIDDLE_ROI.area() / 255;
        double RightValue = Core.sumElems(box2).val[0] / RIGHT_ROI.area() / 255;

        box.release();

        boolean box_Right = RightValue > RIGHT_THRESHOLD;
        boolean box_Left = Value > MIDDLE_THRESHOLD;



        if (box_Right && box_Left) {
            // Position = POSITION.LEFT;
            position = "LEFT";
            telemetry.addData("Position",POSITION.LEFT);

        } else if (box_Right) {
            //  Position = POSITION.MIDDLE;
            position = "MIDDLE";
            telemetry.addData("Position",POSITION.MIDDLE);

        } else if (box_Left) {
            // Position = POSITION.RIGHT;
            position = "RIGHT";
            telemetry.addData("Position",POSITION.RIGHT);
        } else {
            //Position = POSITION.NONE;
            position = "NONE";
            telemetry.addData("Position",POSITION.NONE);
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
