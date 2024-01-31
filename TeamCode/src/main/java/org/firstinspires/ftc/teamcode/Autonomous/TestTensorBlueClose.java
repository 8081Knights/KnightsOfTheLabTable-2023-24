package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name="1. Blue Close with Camera Competition TEST")
public class TestTensorBlueClose extends LinearOpMode {

    OpenCvWebcam cam;

    double backDropServoHIGH = 0.2;
    double backDropServoLOW = 0.725;

    @Override
    public void runOpMode() throws InterruptedException {

        //DriveTrain Initialization
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Grab Camera Hardware ID
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        CameraLogicBlue detector = new CameraLogicBlue(telemetry);
        cam.setPipeline(detector);

        //Set up permissions for Camera Stream
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


        //Hardware Initialization
        HardwareSoftware robot = new HardwareSoftware();
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
                .forward(3)
                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

        TrajectorySequence scoredSpikeRight = drive.trajectorySequenceBuilder(spikeRight.end())
                .back(39)
                .strafeRight(3)
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
                .back(2)
                .strafeLeft(24)
                .turn(Math.toRadians(-180))
                .back(34)
                .strafeLeft(30)
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
                .strafeRight(24)
                .forward(-6,SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL) )
                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

        TrajectorySequence backDropLineUpMiddle = drive.trajectorySequenceBuilder(scoredSpikeForwardProper.end())
                .strafeRight(10)
                .forward(-6,SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL) )
                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

        TrajectorySequence backDropLineUpRight = drive.trajectorySequenceBuilder(scoredSpikeRight.end())

                .forward(-6,SampleMecanumDrive.getVelocityConstraint(18, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL) )
                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;


        TrajectorySequence parkProper = drive.trajectorySequenceBuilder(backDropLineUpMiddle.end())
                .addTemporalMarker(0.5, () ->{
                    robot.backDropServo().setPosition(backDropServoHIGH);

                })
                .forward(5)
                .strafeRight(20)
                .turn(Math.toRadians(90))
                .build();

        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();

        //Camera Detection Storage Variable
        String pos = "LEFT";


        waitForStart();

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
        robot.backDropServo().setPosition(backDropServoLOW);
        sleep(1500);


        //Park Robot
        drive.followTrajectorySequence(parkProper);

        //Set Up Hardware for TeleOp
        robot.intakeLock().setPosition(1);
        sleep(5000);

    }
}
