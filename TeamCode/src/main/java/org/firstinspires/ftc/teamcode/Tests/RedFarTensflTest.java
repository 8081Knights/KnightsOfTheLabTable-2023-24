package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Autonomous.CameraLogicBlue;
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

@Autonomous(name = "FINAL #4: RedFar")
public class RedFarTensflTest extends LinearOpMode {

    final boolean USE_WEBCAM = true;
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/model_20240119_180239.tflite";
    final String[] LABELS = {
            "BlueTeamProp",
            "RedTeamProp",
    };
    private TfodProcessor tfod;
    VisionPortal visionPortal;

    double backDropServoHIGH = 0.2;
    double backDropServoLOW = 0.79;
    double driveTrainSlowedVelocity = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        HardwareSoftware robot = new HardwareSoftware();
        robot.init(hardwareMap);
        robot.intakeLock().setPosition(0.2);
        robot.pixeldrop().setPosition(1);

        //Roadrunner shenanigans (Trajectory Processing Statement)
        Pose2d start = new Pose2d(-33, 60, Math.toRadians(270));
        drive.setPoseEstimate(start);

        Trajectory toSpikeMark = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(-31, 30), Math.toRadians(270))
                .build();

        // Camera Specific
        TrajectorySequence spikeLeft = drive.trajectorySequenceBuilder(toSpikeMark.end())
                .turn(Math.toRadians(90))
                .forward(3)
                .build();
        TrajectorySequence scoredSpikeLeft = drive.trajectorySequenceBuilder(spikeLeft.end())
                .strafeRight(18)
                .build();

        TrajectorySequence spikeRight = drive.trajectorySequenceBuilder(toSpikeMark.end())
                .strafeRight(2)
                .turn(Math.toRadians(-90))
                .forward(2)
                .build();
        TrajectorySequence scoredSpikeRight = drive.trajectorySequenceBuilder(spikeRight.end())
                .back(10)
                .strafeLeft(8)
                .build();

        Trajectory spikeForward = drive.trajectoryBuilder(toSpikeMark.end())
                .back(2)
                .build();


        TrajectorySequence scoredSpikeForwardProper = drive.trajectorySequenceBuilder(spikeForward.end())
                .strafeLeft(20)
                .forward(24)
                .turn(Math.toRadians(90))
                .back(24)
                .build();



        TrajectorySequence backDropProper = drive.trajectorySequenceBuilder(new Pose2d(-31, 8, Math.toRadians(270)))
                .turn(Math.toRadians(90))
                .back(85)
                .strafeLeft(26)
                .build();

        TrajectorySequence backDropLineUpRight = drive.trajectorySequenceBuilder(backDropProper.end())
//                .strafeRight(9)
                .forward(-6, SampleMecanumDrive.getVelocityConstraint(driveTrainSlowedVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence backDropLineUpMiddle = drive.trajectorySequenceBuilder(backDropProper.end())
                .strafeRight(1)
                .forward(-4, SampleMecanumDrive.getVelocityConstraint(driveTrainSlowedVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(2)
                .build();

        TrajectorySequence backDropLineUpLeft = drive.trajectorySequenceBuilder(backDropProper.end())
                .strafeRight(22)
                //was 13
                .forward(-6, SampleMecanumDrive.getVelocityConstraint(driveTrainSlowedVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence parkProper = drive.trajectorySequenceBuilder(backDropLineUpLeft.end())
                .addTemporalMarker(0.5, () ->{
                    robot.backDropServo().setPosition(backDropServoHIGH);
                })
                .forward(6)
                .strafeRight(4)
                .build();

        robot.backDropServo().setPosition(backDropServoHIGH);
        String pos = "RIGHT";

        initTfod();

        waitForStart();

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        boolean isRecognised = (currentRecognitions.size() != 0);
        int recognisedsounter = 0;
        while (!isRecognised && recognisedsounter <=150) {
            currentRecognitions = tfod.getRecognitions();
            isRecognised = (currentRecognitions.size() != 0);
            ++recognisedsounter;
            telemetry.addData("RunLoop? ", currentRecognitions.size() + recognisedsounter);
            telemetry.addData("Recognised? ", (isRecognised) ? ("yes") : ("no"));
            telemetry.update();
            sleep(20);
        }
        boolean isDefault = true;
        if (currentRecognitions.size() > 0){
            if (currentRecognitions.get(0).getRight() < 400) {
                pos = "MIDDLE";
            } else if (currentRecognitions.get(0).getRight() < 640) {
                pos = "RIGHT";
            }
            isDefault = false;
        }
        else{
            pos="LEFT";
        }
        telemetry.addData("Did run loop? ", recognisedsounter);
        telemetry.addData("recognised", isRecognised);
        telemetry.addData("posotion: ", pos);
//        telemetry.addData("Default?: ", (isDefault) ? ("yes") : ("No"));
//        telemetry.addData("deyects?", currentRecognitions.size());
//        telemetry.addData("bounding box right ", currentRecognitions.get(0).getRight());
        telemetry.update();
        drive.followTrajectory(toSpikeMark);
        switch (pos)
        {
            default:
                telemetry.addLine("Going Left");
                telemetry.update();
                robot.intakeLock().setPosition(0.2);
                drive.followTrajectorySequence(spikeLeft);
                robot.pixeldrop().setPosition(0);
                sleep(500);
                drive.followTrajectorySequence(scoredSpikeLeft);
                drive.followTrajectorySequence(backDropProper);
                robot.intakeLock().setPosition(0);
                drive.followTrajectorySequence(backDropLineUpLeft);
                break;

            case "RIGHT":
                telemetry.addLine("Going Right");
                telemetry.update();
                drive.followTrajectorySequence(spikeRight);
                robot.pixeldrop().setPosition(0);
                sleep(500);
                drive.followTrajectorySequence(scoredSpikeRight);
                drive.followTrajectorySequence(backDropProper);
                drive.followTrajectorySequence(backDropLineUpRight);
                break;

            case "MIDDLE":
                telemetry.addLine("Going Forward");
                telemetry.update();
                drive.followTrajectory(spikeForward);
                robot.pixeldrop().setPosition(0);
                sleep(500);
                drive.followTrajectorySequence(scoredSpikeForwardProper);
                drive.followTrajectorySequence(backDropProper);
                drive.followTrajectorySequence(backDropLineUpMiddle);
                break;
        }

        robot.backDropServo().setPosition(backDropServoLOW);
        sleep(1500);
        robot.backDropServo().setPosition(backDropServoHIGH);
        drive.followTrajectorySequence(parkProper);
        drive.turn(Math.toRadians(-90));
        robot.intakeLock().setPosition(1);
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
