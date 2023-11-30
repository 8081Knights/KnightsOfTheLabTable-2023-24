package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name="Red Far with Camera")
public class redFarAutonomousCam extends LinearOpMode {

    OpenCvWebcam cam;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        cam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        CameraLogicRed detector = new CameraLogicRed(telemetry);
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

        Pose2d start = new Pose2d(-33, 60, Math.toRadians(270));

        HardwareSoftware robot = new HardwareSoftware();
        robot.init(hardwareMap);

        drive.setPoseEstimate(start);

        Trajectory toSpikeMark = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(-31, 33), Math.toRadians(0))
                .build();

        //Camera Specific
        TrajectorySequence spikeLeft = drive.trajectorySequenceBuilder(toSpikeMark.end())
                .turn(Math.toRadians(90))
                .build();
        TrajectorySequence spikeRight = drive.trajectorySequenceBuilder(toSpikeMark.end())
                .turn(Math.toRadians(-90))
                .build();

        Trajectory spikeForward = drive.trajectoryBuilder(toSpikeMark.end())
                .splineToConstantHeading(new Vector2d(-19, 33), Math.toRadians(270))
                .splineTo(new Vector2d(-19, 0), Math.toRadians(270))
                .splineTo(new Vector2d(-31, 0), Math.toRadians(270))
                .build();

        Trajectory toGateLeft = drive.trajectoryBuilder(spikeLeft.end())
                .splineTo(new Vector2d(-31, 0), Math.toRadians(270))
                .build();

        Trajectory toGateRight = drive.trajectoryBuilder(spikeRight.end())
                .splineTo(new Vector2d(-31, 0), Math.toRadians(270))
                .build();

        Trajectory underGate = drive.trajectoryBuilder(new Pose2d(-31, 0, Math.toRadians(270)))
                .splineTo(new Vector2d(-65, 0), Math.toRadians(180))
                .build();

        Trajectory scoreZone = drive.trajectoryBuilder(underGate.end())
                .splineTo(new Vector2d(-110, 0), Math.toRadians(180))
                .build();

        Trajectory backDrop = drive.trajectoryBuilder(scoreZone.end())
                .splineTo(new Vector2d(-107, 33), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-113, 33), Math.toRadians(0))
                .build();

        Trajectory park = drive.trajectoryBuilder(backDrop.end())
                .splineToConstantHeading(new Vector2d(-113, 60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-116, 60), Math.toRadians(0))
                .build();

        robot.pixeldrop().setPosition(0);


        waitForStart();
        drive.followTrajectory(toSpikeMark);
        switch(detector.position){
            default:
                robot.pixeldrop().setPosition(0.5);
                sleep(500);
                drive.followTrajectory(spikeForward);

            case "LEFT":
                drive.followTrajectorySequence(spikeLeft);
                robot.pixeldrop().setPosition(0.5);
                sleep(500);
                drive.followTrajectory(toGateLeft);

            case "RIGHT":
                drive.followTrajectorySequence(spikeRight);
                robot.pixeldrop().setPosition(0.5);
                sleep(500);
                drive.followTrajectory(toGateRight);

            case "MIDDLE":
                robot.pixeldrop().setPosition(0.5);
                sleep(500);
                drive.followTrajectory(spikeForward);

        }
        sleep(2000);
        robot.pixeldrop().setPosition(1);
        drive.followTrajectory(underGate);
        drive.followTrajectory(scoreZone);
        drive.followTrajectory(backDrop);
        sleep(2000);
        drive.followTrajectory(park);

    }
}
