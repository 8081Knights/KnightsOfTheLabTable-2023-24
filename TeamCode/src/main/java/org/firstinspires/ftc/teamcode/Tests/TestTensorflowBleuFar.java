package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
//import org.firstinspires.ftc.teamcode.Autonomous.CameraLogicBlue;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@Autonomous(name = "2. Blue Far with Camera Competition TEST")
public class TestTensorflowBleuFar extends LinearOpMode {


// THIS IS REAL
    final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_20240119_180239.tflite";
    final String[] LABELS = {
            "BlueTeamProp",
            "RedTeamProp",
    };
    private TfodProcessor tfod;
    VisionPortal visionPortal;



//    OpenCvWebcam cam;
    double backDropServoHIGH = 0.2;
    double backDropServoLOW = 0.725;

    @Override
    public void runOpMode() throws InterruptedException {


        //Declare Drive Train Handler
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        initTfod();
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        boolean isRecognised = (currentRecognitions.size() == 0) ? (false) : (true);
        int recognisedsounter = 0;
        while (!isRecognised && recognisedsounter <=5) {
            currentRecognitions = tfod.getRecognitions();
            isRecognised = (currentRecognitions.size() == 0) ? (false) : (true);
            ++recognisedsounter;
            telemetry.addData("RunLoop? ", currentRecognitions.size());
            telemetry.addData("Recognised? ", (isRecognised) ? ("yes") : ("no"));
            telemetry.update();
        }
        String pos = "MIDDLE";
        boolean isDefault = true;
        if (currentRecognitions.size() > 0){
            if (currentRecognitions.get(0).getRight() < 400) {
                pos = "LEFT";
            } else if (currentRecognitions.get(0).getRight() < 640) {
                pos = "MIDDLE";
            } else {
                pos = "RIGHT";
            }
            isDefault = false;
        }
        telemetry.addData("posotion: ", pos);
        telemetry.addData("Default?: ", (isDefault) ? ("yes") : ("No"));
        telemetry.addData("deyects?", currentRecognitions.size());
        telemetry.update();

        /*//Grab Camera Hardware Map
        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        CameraLogicBlue detector = new CameraLogicBlue(telemetry);
        cam.setPipeline(detector);

        //Set up camera permissions
        cam.setMillisecondsPermissionTimeout(2500);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });*/







        //Hardware Initialization
        HardwareSoftware robot = new HardwareSoftware();
        robot.init(hardwareMap);
        robot.intakeLock().setPosition(0.2);
        robot.pixeldrop().setPosition(1);
        robot.backDropServo().setPosition(backDropServoHIGH);

        //Processing Paths
        Pose2d start = new Pose2d(-33, -60, Math.toRadians(90));
        drive.setPoseEstimate(start);

        Trajectory toSpikeMark = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(-35, -30), Math.toRadians(90))
                .build();

        TrajectorySequence spikeLeft = drive.trajectorySequenceBuilder(toSpikeMark.end())
                .turn(Math.toRadians(90))
                .forward(4)
                .build();

        TrajectorySequence scoredSpikeLeft = drive.trajectorySequenceBuilder(spikeLeft.end())
                .back(5)
                .build();

        TrajectorySequence spikeRight = drive.trajectorySequenceBuilder(toSpikeMark.end())
                .turn(Math.toRadians(-90))
                .forward(2)
                .build();

        TrajectorySequence scoredSpikeRight = drive.trajectorySequenceBuilder(spikeRight.end())
                .back(3)
                .strafeLeft(20)
                .build();

        Trajectory spikeForward = drive.trajectoryBuilder(toSpikeMark.end())
                .back(2)
                .build();

        Trajectory scoredSpikeForward = drive.trajectoryBuilder(spikeForward.end())
                .splineToConstantHeading(new Vector2d(-33, -42), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-14, -34), Math.toRadians(90))
                .forward(28)
                .build();

        Trajectory toGateLeft = drive.trajectoryBuilder(scoredSpikeLeft.end())
                .splineTo(new Vector2d(-35, -13), Math.toRadians(90))
                .build();

        Trajectory toGateRight = drive.trajectoryBuilder(scoredSpikeRight.end())
                .splineTo(new Vector2d(-35, -13), Math.toRadians(90))
                .build();

        Trajectory toGateForward = drive.trajectoryBuilder(scoredSpikeForward.end())
                .splineTo(new Vector2d(-14, 0), Math.toRadians(90))
                .build();

        Trajectory backDrop = drive.trajectoryBuilder(new Pose2d(-31, -13, Math.toRadians(90)))
                .splineTo(new Vector2d(-117, -13), Math.toRadians(0))
                .splineTo(new Vector2d(-123, -33), Math.toRadians(0))
                .build();

        TrajectorySequence backDropProper = drive.trajectorySequenceBuilder(new Pose2d(-31, -13, 0))
                .back(86)
                .strafeRight(20)
                .back(5)
                .build();

        TrajectorySequence backDropLineUpRight = drive.trajectorySequenceBuilder(backDrop.end())
                .strafeRight(6)
                .forward(-6, SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence backDropLineUpMiddle = drive.trajectorySequenceBuilder(backDrop.end())
                .strafeRight(8)
                .forward(-6, SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence backDropLineUpLeft = drive.trajectorySequenceBuilder(backDrop.end())
                .strafeRight(13)
                .forward(-6, SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence parkProper = drive.trajectorySequenceBuilder(new Pose2d(-115, -33, 0))
                .back(6)
                .strafeLeft(20)
                .build();

        //Camera Detection Storage Variable

        // Tensorflow Initialization


        waitForStart();

        //Start of Program
        drive.followTrajectory(toSpikeMark);
        switch (pos) {
            default:
                telemetry.addLine("Going Left");

                drive.followTrajectorySequence(spikeLeft);

                //Score Spike Mark Pixel
                robot.pixeldrop().setPosition(0);
                sleep(500);

                //Line Up to BackDrop
                drive.followTrajectorySequence(scoredSpikeLeft);
                drive.followTrajectory(toGateLeft);
                drive.followTrajectorySequence(backDropProper);
                drive.followTrajectorySequence(backDropLineUpLeft);
                break;

            case "RIGHT":
                telemetry.addLine("Going Right");

                drive.followTrajectorySequence(spikeRight);

                //Score Spike Mark Pixel
                robot.pixeldrop().setPosition(0);
                sleep(500);

                //Line Up to BackDrop
                drive.followTrajectorySequence(scoredSpikeRight);
                drive.followTrajectory(toGateRight);
                drive.followTrajectorySequence(backDropProper);
                drive.followTrajectorySequence(backDropLineUpRight);
                break;

            case "MIDDLE":
                telemetry.addLine("Going Forward");
                drive.followTrajectory(spikeForward);

                //Score Spike Mark Pixel
                robot.pixeldrop().setPosition(0);
                sleep(500);

                //Line Up to BackDrop
                drive.followTrajectory(scoredSpikeForward);
                drive.followTrajectory(toGateForward);
                drive.followTrajectorySequence(backDropProper);
                drive.followTrajectorySequence(backDropLineUpMiddle);
                break;
        }

        //Deliver Back Drop Pixel
        robot.backDropServo().setPosition(backDropServoLOW);
        sleep(1500);

        //Park Robot
        drive.followTrajectorySequence(parkProper);
        drive.turn(Math.toRadians(93));

        //Prepare Hardware for TeleOp
        robot.intakeLock().setPosition(1);
        robot.backDropServo().setPosition(backDropServoHIGH);
        sleep(5000);
    }
    void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFadileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                .setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()
}
