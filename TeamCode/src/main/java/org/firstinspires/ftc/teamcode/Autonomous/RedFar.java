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


@Autonomous(name="2. Red Far with Camera Competition")
public class RedFar extends LinearOpMode {

    OpenCvWebcam cam;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        cam = OpenCvCameraFactory.getInstance()
                .createWebcam(hardwareMap.get(WebcamName.class, "Webcam"), cameraMonitorViewId);
        CameraLogicBlue detector = new CameraLogicBlue(telemetry);
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



        HardwareSoftware robot = new HardwareSoftware();
        robot.init(hardwareMap);
        robot.intakeLock().setPosition(0.2);


        Pose2d start = new Pose2d(-33, 60, Math.toRadians(270));

        drive.setPoseEstimate(start);

        Trajectory toSpikeMark = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(-31, 30), Math.toRadians(270))
                .build();
//
        int x = 1;
        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

        //Camera Specific
        TrajectorySequence spikeLeft = drive.trajectorySequenceBuilder(toSpikeMark.end())
//                //.strafeRight(2)
//                .turn(Math.toRadians(90))
//                .forward(3)
//                //.back(2)
//                .build();
                .turn(Math.toRadians(90))
                .forward(3)
                .build();
        TrajectorySequence scoredSpikeLeft = drive.trajectorySequenceBuilder(spikeLeft.end())
//                .back(4)
//                // .turn(Math.toRadians(-90))
////                .back(2)
               // .back(10)
                .strafeRight(18)
                // .turn(Math.toRadians(90))
                .build();
        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;
        TrajectorySequence spikeRight = drive.trajectorySequenceBuilder(toSpikeMark.end())
        .strafeRight(2)
                .turn(Math.toRadians(-90))
               // .forward(4)
                //.back(1)
                .build();
        TrajectorySequence scoredSpikeRight = drive.trajectorySequenceBuilder(spikeRight.end())
//                .back(3)
//                .strafeLeft(7)
//                // .turn(Math.toRadians(90))
//                .build();
                .back(10)
                .strafeLeft(8)
                // .turn(Math.toRadians(-90))
//                .back(2)
        .build();
        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;


//        TrajectorySequence underHang = drive.trajectorySequenceBuilder(toSpikeMark.end())
//
//                .back(6)
//                .strafeLeft(40)
//                .build();
        Trajectory spikeForward = drive.trajectoryBuilder(toSpikeMark.end())
                .back(2)
                .build();

        Trajectory scoredSpikeForward = drive.trajectoryBuilder(spikeForward.end())
                // .splineToConstantHeading(new Vector2d(0, -5), 180)
                //.back(4)
                .splineToConstantHeading(new Vector2d(-33, 42), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-14, 34), Math.toRadians(270))
                .forward(18)
                //.splineTo(new Vector2d(-25, -4), Math.toRadians(90))
                .build();

//        Trajectory spikeForward = drive.trajectoryBuilder(toSpikeMark.end())
//                .splineToConstantHeading(new Vector2d(-20, -31), Math.toRadians(90))
//                .splineTo(new Vector2d(-20, 0), Math.toRadians(90),
//                        SampleMecanumDrive.getVelocityConstraint(25, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
//                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .splineToConstantHeading(new Vector2d(-31, 0), Math.toRadians(90))
//                .build();
        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

        Trajectory toGateLeft = drive.trajectoryBuilder(scoredSpikeLeft.end())
                .splineTo(new Vector2d(-35, 13), Math.toRadians(270))
                .build();
        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

        Trajectory toGateRight = drive.trajectoryBuilder(scoredSpikeRight.end())
                .splineTo(new Vector2d(-20, 0), Math.toRadians(270))
                .build();
        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;


        Trajectory toGateForward = drive.trajectoryBuilder(scoredSpikeForward.end())
                .splineTo(new Vector2d(-14, 0), Math.toRadians(270))
                .build();
        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;


        Trajectory backDrop = drive.trajectoryBuilder(new Pose2d(-31, 15, Math.toRadians(270)))
                .splineTo(new Vector2d(-109, 15), Math.toRadians(0))
                .splineTo(new Vector2d(-114, 33), Math.toRadians(0))
                // .splineToConstantHeading(new Vector2d(-110, -33), Math.toRadians(0))
                .build();
        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();













        x++;

        //TODO: Change start back to backDrop.end()
        Trajectory park = drive.trajectoryBuilder(new Pose2d(-106, 33, 0))
                .splineTo(new Vector2d(-110, 33), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-111, 10), Math.toRadians(0))
                .build();

        TrajectorySequence parkProper = drive.trajectorySequenceBuilder(new Pose2d(-109, 10, 0))
                //.back(6)
                .strafeRight(8)
                .back(10)
                .build();
        telemetry.addData("Trajectory setup success: ", x);
        telemetry.update();
        x++;

//        robot.pixeldrop().setPosition(1);

        String pos = "LEFT";
//
//        while(!opModeIsActive() && !isStopRequested()){
//            pos = detector.position;
//            telemetry.addData("Going: ", pos);
//            telemetry.update();
//        }

        waitForStart();

//
        drive.followTrajectory(toSpikeMark);
        switch(pos){
            default:
                telemetry.addLine("Going Left");
                telemetry.update();
                drive.followTrajectorySequence(spikeLeft);

                robot.intake().setPower(-.7);
                sleep(1500);
                robot.intake().setPower(0);
                drive.followTrajectorySequence(scoredSpikeLeft);
                drive.followTrajectory(toGateLeft);
                drive.followTrajectory(backDrop);
                break;

            case "NONE":
                telemetry.addLine("Going Left");
                telemetry.update();
                drive.followTrajectorySequence(spikeLeft);

                robot.intake().setPower(-.5);
                sleep(1000);
                robot.intake().setPower(0);
                drive.followTrajectorySequence(scoredSpikeLeft);
                drive.followTrajectory(toGateLeft);
                drive.followTrajectory(backDrop);
                break;

            case "LEFT":

                telemetry.addLine("Going Left");
                telemetry.update();
                robot.intakeLock().setPosition(0.2);
                drive.followTrajectorySequence(spikeLeft);
                robot.intake().setPower(-.45);
                sleep(500);
                robot.intake().setPower(0);
                drive.followTrajectorySequence(scoredSpikeLeft);
                drive.followTrajectory(toGateLeft);
                drive.followTrajectory(backDrop);
                robot.intakeLock().setPosition(0);
                break;

            case "RIGHT":
                telemetry.addLine("Going Right");

                telemetry.update();
                drive.followTrajectorySequence(spikeRight);
                robot.intake().setPower(-.45);
                sleep(500);
                robot.intake().setPower(0);
                drive.followTrajectorySequence(scoredSpikeRight);
                drive.followTrajectory(toGateRight);
                drive.followTrajectory(backDrop);
                break;


            case "MIDDLE":
                telemetry.addLine("Going Forward");
                telemetry.update();
                drive.followTrajectory(spikeForward);
                robot.intake().setPower(-.45);
                sleep(500);
                robot.intake().setPower(0);
                drive.followTrajectory(scoredSpikeForward);
                drive.followTrajectory(toGateForward);
                drive.followTrajectory(backDrop);
                break;

        }

        //   robot.pixeldrop().setPosition(1);

        robot.runSlides(-1800, 2000);
        sleep(1700);
        robot.pixelServo().setPosition(1);
        sleep(500);
        robot.pixelServo().setPosition(0.5);
        robot.runSlides(-25, 2000);
        sleep(1500);
//        drive.followTrajectory(park);
        drive.followTrajectorySequence(parkProper);
        drive.turn(Math.toRadians(90));

        robot.intakeLock().setPosition(1);
        sleep(5000);

    }
}
