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

@Autonomous(name = "4. Red Far with Camera Competition")
public class RedFar extends LinearOpMode {

    OpenCvWebcam cam;

    double backDropServoHIGH = 0.2;
    double backDropServoLOW = 0.79;
    double driveTrainSlowedVelocity = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        cam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        CameraLogicBlue detector = new CameraLogicBlue(telemetry);
        cam.setPipeline(detector);
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
                .strafeLeft(24)
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
                .strafeRight(2)
                .forward(-6, SampleMecanumDrive.getVelocityConstraint(driveTrainSlowedVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence backDropLineUpMiddle = drive.trajectorySequenceBuilder(backDropProper.end())
                .strafeRight(9)
                .forward(-4, SampleMecanumDrive.getVelocityConstraint(driveTrainSlowedVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .back(2)
                .build();

        TrajectorySequence backDropLineUpLeft = drive.trajectorySequenceBuilder(backDropProper.end())
                .strafeRight(9)
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
        waitForStart();

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
}
