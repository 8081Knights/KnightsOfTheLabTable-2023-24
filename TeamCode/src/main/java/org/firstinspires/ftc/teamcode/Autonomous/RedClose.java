package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous(name = "Red Close with Camera Competition")
public class RedClose extends LinearOpMode {

    OpenCvWebcam cam;

    double backDropServoHIGH = 0.2;
    double backDropServoLOW = 0.9;
    double driveTrainSlowedVelocity = 20;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initializing the roadRunner tracking system
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        //Getting a hardware map handle for the webcam
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        CameraLogicBlue detector = new CameraLogicBlue(telemetry);
        cam.setPipeline(detector);


        //Establishing Permissions and Async Camera Streaming
        cam.setMillisecondsPermissionTimeout(2500);
        cam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        //Initializing Robot Hardware
        HardwareSoftware robot = new HardwareSoftware();
        robot.init(hardwareMap);
        robot.intakeLock().setPosition(0.2);
        robot.pixeldrop().setPosition(1);


        //Set Start Pose of the Robot
        Pose2d start = new Pose2d(-33, 109, Math.toRadians(270));
        drive.setPoseEstimate(start);


        //Trajectory to drive to the Spike Mark
        Trajectory toSpikeMark = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(-31, 79), Math.toRadians(270))
                .build();


        //Trajectory to deliver Left Spike Mark
        TrajectorySequence spikeLeft = drive.trajectorySequenceBuilder(toSpikeMark.end())
                .turn(Math.toRadians(90))
                .forward(3)
                .build();

        //Trajectory to backdrop from scored Spike Mark
        TrajectorySequence scoredSpikeLeft = drive.trajectorySequenceBuilder(spikeLeft.end())
                .back(39)
                .strafeRight(3)
                .build();

        //Trajectory to deliver Right Spike Mark
        TrajectorySequence spikeRight = drive.trajectorySequenceBuilder(toSpikeMark.end())
                .strafeLeft(2)
                .turn(Math.toRadians(-90))
                .strafeRight(2)
                .forward(3)
                .build();

        //Trajectory to backdrop from scored Spike Mark
        TrajectorySequence scoredSpikeRight = drive.trajectorySequenceBuilder(spikeRight.end())
                .back(1)
                .strafeLeft(15)
                .turn(Math.toRadians(180))
                .back(36)
                .forward(1)
                .strafeLeft(19)
                .back(2)
                .build();

        //Trajectory to deliver Forward Spike Mark
        Trajectory spikeForward = drive.trajectoryBuilder(toSpikeMark.end())
                .back(2)
                .build();

        //TRUNCATED TO: scoredSpikeForwardProper
        Trajectory scoredSpikeForward = drive.trajectoryBuilder(spikeForward.end())
                .splineToConstantHeading(new Vector2d(-33, 91), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-14, 83), Math.toRadians(270))
                .build();

        //Trajectory to backdrop from scored Spike Mark
        TrajectorySequence scoredSpikeForwardProper = drive.trajectorySequenceBuilder(spikeForward.end())
                .turn(Math.toRadians(90))
                .back(36)
                .build();



        //Trajectory to line up Right backdrop delivery
        TrajectorySequence backDropLineUpRight = drive.trajectorySequenceBuilder(scoredSpikeRight.end())
                .strafeRight(2)
                .forward(-3, SampleMecanumDrive.getVelocityConstraint(driveTrainSlowedVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //Trajectory to line up Forward backdrop delivery
        TrajectorySequence backDropLineUpMiddle = drive.trajectorySequenceBuilder(scoredSpikeLeft.end())
                .strafeRight(10)
                .forward(-3, SampleMecanumDrive.getVelocityConstraint(driveTrainSlowedVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        //Trajectory to line up Left backdrop delivery
        TrajectorySequence backDropLineUpLeft = drive.trajectorySequenceBuilder(scoredSpikeForwardProper.end())
                .strafeRight(13)
                .forward(-3, SampleMecanumDrive.getVelocityConstraint(driveTrainSlowedVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //Trajectory to park
        TrajectorySequence parkProper = drive.trajectorySequenceBuilder(new Pose2d(-111, 20, 0))
                .strafeRight(4)
                .build();

        robot.backDropServo().setPosition(backDropServoHIGH);

        String pos = "LEFT";

        waitForStart();

        drive.followTrajectory(toSpikeMark);
        switch (pos) {
            default:
                telemetry.addLine("NO CASE!! Guessing Left");
                telemetry.update();
                robot.intakeLock().setPosition(0.2);
                drive.followTrajectorySequence(spikeLeft);
                robot.pixeldrop().setPosition(0);
                sleep(500);
                drive.followTrajectorySequence(scoredSpikeLeft);
                robot.intakeLock().setPosition(0);
                drive.followTrajectorySequence(backDropLineUpLeft);
                break;

            case "LEFT":
                telemetry.addLine("Going Left");
                telemetry.update();
                robot.intakeLock().setPosition(0.2);
                drive.followTrajectorySequence(spikeLeft);
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
                robot.pixeldrop().setPosition(0);
                sleep(500);
                drive.followTrajectorySequence(scoredSpikeRight);
                drive.followTrajectorySequence(backDropLineUpRight);
                break;

            case "MIDDLE":
                telemetry.addLine("Going Forward");
                telemetry.update();
                drive.followTrajectory(spikeForward);
                robot.pixeldrop().setPosition(1);
                sleep(500);
                drive.followTrajectorySequence(scoredSpikeForwardProper);
                drive.followTrajectorySequence(backDropLineUpMiddle);
                break;
        }

        robot.backDropServo().setPosition(backDropServoLOW);
        sleep(2000);
        robot.backDropServo().setPosition(backDropServoHIGH);
        drive.followTrajectorySequence(parkProper);
        drive.turn(Math.toRadians(-90));
        robot.intakeLock().setPosition(1);
        sleep(5000);
    }
}