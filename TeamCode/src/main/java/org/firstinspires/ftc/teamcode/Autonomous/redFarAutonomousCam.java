package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
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
                .splineTo(new Vector2d(-31, 33), Math.toRadians(270))
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

        Trajectory toGateLeft = drive.trajectoryBuilder(new Pose2d(-31, 33, Math.toRadians(0)))
                .splineToConstantHeading(new Vector2d(-31, 15), Math.toRadians(0))
                .splineTo(new Vector2d(-31, 0), Math.toRadians(270))
                .build();

        Trajectory toGateRight = drive.trajectoryBuilder(spikeRight.end())
                .splineToConstantHeading(new Vector2d(-27, 15), Math.toRadians(180))
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

        robot.pixeldrop().setPosition(1);

        String pos = "";

        while(!opModeIsActive()){
            pos = detector.position;
            telemetry.addData("Going: ", pos);
            telemetry.update();
        }

        waitForStart();


        drive.followTrajectory(toSpikeMark);
        switch(pos){
            default:
                robot.pixeldrop().setPosition(0);
                sleep(500);
                drive.followTrajectory(spikeForward);
                break;

            case "LEFT":

                telemetry.addLine("Going Left");
                telemetry.update();
                drive.followTrajectorySequence(spikeLeft);
                robot.intake().setPower(-.5);
                sleep(700);
                robot.intake().setPower(0);
                drive.followTrajectory(toGateLeft);

                break;

            case "RIGHT":
                telemetry.addLine("Going Right");
                telemetry.update();
                drive.followTrajectorySequence(spikeRight);
                robot.intake().setPower(-.7);
                sleep(1000);
                robot.intake().setPower(0);
                drive.followTrajectory(toGateRight);
                break;

            case "MIDDLE":
                telemetry.addLine("Going Forward");
                telemetry.update();
                robot.intake().setPower(-.4);
                sleep(700);
                robot.intake().setPower(0);
                drive.followTrajectory(spikeForward);
                break;

        }

       // robot.pixeldrop().setPosition(1);
        drive.followTrajectory(underGate);
        drive.followTrajectory(scoreZone);
        drive.followTrajectory(backDrop);
        sleep(2000);
        drive.followTrajectory(park);

    }
}
