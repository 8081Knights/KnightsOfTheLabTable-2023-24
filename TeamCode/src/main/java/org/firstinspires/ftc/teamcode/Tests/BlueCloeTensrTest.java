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

@Autonomous(name = "FINAL #1: BlueClose")
public class BlueCloeTensrTest extends LinearOpMode {


    double backDropServoHIGH = 0.2;
    double backDropServoLOW = 0.9075;


    final boolean USE_WEBCAM = true;
    HardwareSoftware robot = new HardwareSoftware();
    String TFOD_MODEL_FILE = robot.TFOD_MODEL_FILE;
    final String[] LABELS = {
            "BlueTeamProp",
            "RedTeamProp",
    };
    private TfodProcessor tfod;
    VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {

        //DriveTrain Initialization
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        //Hardware Initialization
        robot.init(hardwareMap);
        robot.intakeLock().setPosition(0.2);
        robot.pixeldrop().setPosition(1);
        robot.backDropServo().setPosition(backDropServoHIGH);

        //Set start positioning estimate
        Pose2d start = new Pose2d(-33, -109, Math.toRadians(90));
        drive.setPoseEstimate(start);


        Trajectory toSpikeMark = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(-35, -79), Math.toRadians(90))
                .build();

        int x = 1;
        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;


        //Camera Specific
        TrajectorySequence spikeRight = drive.trajectorySequenceBuilder(toSpikeMark.end())
                .turn(Math.toRadians(-90))
                .forward(2)
                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

        TrajectorySequence scoredSpikeRight = drive.trajectorySequenceBuilder(spikeRight.end())
                //.forward(-2)
                .back(39)
                .strafeRight(3)
                //.back(2)

                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

        TrajectorySequence spikeLeft = drive.trajectorySequenceBuilder(toSpikeMark.end())
                .strafeRight(2)
                .turn(Math.toRadians(90))
                .forward(4)
                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

        TrajectorySequence scoredSpikeLeft = drive.trajectorySequenceBuilder(spikeLeft.end())
                //  .back(2)
                .strafeLeft(20)
                .turn(Math.toRadians(-185))
                .back(34)
//                .strafeLeft(30)
                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

        Trajectory spikeForward = drive.trajectoryBuilder(toSpikeMark.end())
                .back(2)
                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

        TrajectorySequence scoredSpikeForwardProper = drive.trajectorySequenceBuilder(spikeForward.end())
                .back(3)
                .turn(Math.toRadians(-90))
                .back(36)
                .strafeLeft(3)
                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

        Trajectory backDrop = drive.trajectoryBuilder(new Pose2d(-31, -83, Math.toRadians(90)))
                .splineTo(new Vector2d(-109, -15), Math.toRadians(0))
                .splineTo(new Vector2d(-114, -40), Math.toRadians(0))
                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

        TrajectorySequence backDropLineUpLeft = drive.trajectorySequenceBuilder(scoredSpikeLeft.end())
                .strafeLeft(6)
                .forward(-6,SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL) )
                .forward(5)
                .addTemporalMarker(1.4, () ->{
                    robot.backDropServo().setPosition(backDropServoLOW);

                })
                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

        TrajectorySequence backDropLineUpMiddle = drive.trajectorySequenceBuilder(scoredSpikeForwardProper.end())
                .forward(2)
                .strafeRight(10)
                .forward(-6,SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL) )
                .forward(5)
                .addTemporalMarker(1.6, () ->{
                    robot.backDropServo().setPosition(backDropServoLOW);

                })
                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

        TrajectorySequence backDropLineUpRight = drive.trajectorySequenceBuilder(scoredSpikeRight.end())
                .back(3)
                .strafeRight(4.5)

                .forward(-1,SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL) )
               // .back(2)
                .forward(5)
                .addTemporalMarker(1.1, () ->{
                    robot.backDropServo().setPosition(backDropServoLOW);

                })
                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;


        TrajectorySequence parkProper = drive.trajectorySequenceBuilder(backDropLineUpMiddle.end())
                .addTemporalMarker(0.25, () ->{
                    robot.backDropServo().setPosition(backDropServoHIGH);

                })
                .forward(8)
                .strafeRight(20)
                .turn(Math.toRadians(90))
                .back(3)
                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();

        String pos = "MIDDLE";

        initTfod();

        boolean g1a = false;
        boolean wait = false;
        while(!isStarted() && !isStopRequested()){
            if(gamepad1.a){
                g1a = true;
            }
            else if(g1a && !gamepad1.a){
                g1a = false;
                if(!wait){
                    wait = true;
                    telemetry.addLine("5 Second wait Added");
                    telemetry.update();
                }
                else{
                    wait = false;
                    telemetry.addLine("Press a to add a 5 second wait");
                    telemetry.update();
                }
            }
        }

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
            if (currentRecognitions.get(0).getRight() < 400 ) { //was 0
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

        telemetry.addLine("Press a to add a 5 second wait");
        telemetry.update();

        
        if(wait){
            sleep(5000);
        }


//        telemetry.addData("Default?: ", (isDefault) ? ("yes") : ("No"));
//        telemetry.addData("deyects?", currentRecognitions.size());
//        telemetry.addData("bounding box right ", currentRecognitions.get(0).getRight());
        telemetry.update();
//
        drive.followTrajectory(toSpikeMark);
        switch(pos){
            default:
                telemetry.addLine("Going Left");
                telemetry.update();
                drive.followTrajectorySequence(spikeLeft);

                //Deliver Spike Mark Pixel
                robot.pixeldrop().setPosition(0);
                sleep(500);

                drive.followTrajectorySequence(scoredSpikeLeft);
                robot.intakeLock().setPosition(0);
                drive.followTrajectorySequence(backDropLineUpLeft);

                break;

            case "RIGHT":
                telemetry.addLine("Going Right");
                telemetry.update();
                drive.followTrajectorySequence(spikeRight);

                //Deliver Spike Mark Pixel
                robot.pixeldrop().setPosition(0);
                sleep(500);

                drive.followTrajectorySequence(scoredSpikeRight);
                drive.followTrajectorySequence(backDropLineUpRight);


                break;


            case "MIDDLE":
                telemetry.addLine("Going Forward");
                telemetry.update();
                drive.followTrajectory(spikeForward);

                //Deliver Spike Mark Pixel
                robot.pixeldrop().setPosition(0);
                sleep(500);
                drive.followTrajectorySequence(scoredSpikeForwardProper);
                drive.followTrajectorySequence(backDropLineUpMiddle);

                break;

        }

        //Deliver BackDrop Pixel
        sleep(500);


        //Park Robot
        drive.followTrajectorySequence(parkProper);

        //Set Up Hardware for TeleOp
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
