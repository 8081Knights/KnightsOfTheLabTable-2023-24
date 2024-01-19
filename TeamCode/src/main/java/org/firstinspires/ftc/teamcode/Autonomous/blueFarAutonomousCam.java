package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
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

@Autonomous(name = "2. Blue Far with Camera Competition")
public class blueFarAutonomousCam extends LinearOpMode {

    OpenCvWebcam cam;
    double backDropServoHIGH = 0.2;
    double backDropServoLOW = 0.725;

    @Override
    public void runOpMode() throws InterruptedException {

        //Declare Drive Train Handler
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //Grab Camera Hardware Map
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
        });


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
        String pos = "MIDDLE";

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
}
