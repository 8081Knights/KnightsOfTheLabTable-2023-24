package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Timer;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Red Close with Bumper Sensor")
public class RedCloseBumperTouch extends LinearOpMode {


    double backDropServoHIGH = 0.2;
    double backDropServoLOW = 0.98;
    double driveTrainSlowedVelocity = 20;

    @Override
    public void runOpMode() throws InterruptedException {

        //Initializing the roadRunner tracking system
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);




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
                // .forward(1)
                .strafeLeft(17)
                .back(2)
                .build();

        //Trajectory to deliver Forward Spike Mark
        Trajectory spikeForward = drive.trajectoryBuilder(toSpikeMark.end())
                .back(2)
                // .strafeLeft(7)
                // .strafeLeft(7)
                .build();

        //TRUNCATED TO: scoredSpikeForwardProper
        //  Trajectory scoredSpikeForward = drive.trajectoryBuilder(spikeForward.end())
        //.splineToConstantHeading(new Vector2d(-33, 91), Math.toRadians(270))
        // .splineToConstantHeading(new Vector2d(-14, 83), Math.toRadians(270))
        // .strafeLeft(7)
        //  .build();

        //Trajectory to backdrop from scored Spike Mark
        TrajectorySequence scoredSpikeForwardProper = drive.trajectorySequenceBuilder(spikeForward.end())

                .back(2)
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
        TrajectorySequence backDropLineUpMiddle = drive.trajectorySequenceBuilder(new Pose2d(-63, 79, Math.toRadians(0)))
                .forward(-5, SampleMecanumDrive.getVelocityConstraint(driveTrainSlowedVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        //Trajectory to line up Left backdrop delivery
        TrajectorySequence backDropLineUpLeft = drive.trajectorySequenceBuilder(scoredSpikeForwardProper.end())
                .strafeRight(13)
                .forward(-3, SampleMecanumDrive.getVelocityConstraint(driveTrainSlowedVelocity, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        //Trajectory to park
        TrajectorySequence parkProper = drive.trajectorySequenceBuilder(backDropLineUpMiddle.end())
                .addTemporalMarker(0.5, () -> {
                    robot.backDropServo().setPosition(backDropServoHIGH);
                })
                .forward(6)
                .turn(Math.toRadians(-90))
                .strafeRight(4)
                .back(20)
                .build();

        Trajectory returnToStart = drive.trajectoryBuilder(parkProper.end())
                .splineTo(new Vector2d(-33, 109), Math.toRadians(-90))
                .build();

        robot.backDropServo().setPosition(backDropServoHIGH);

        String pos = "MIDDLE";

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
                //  drive.followTrajectorySequence(scoredSpikeForward);
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
                robot.pixeldrop().setPosition(0);
                sleep(500);
                drive.followTrajectorySequence(scoredSpikeForwardProper);
                driveToHit(drive, robot, -0.2, 2000);
                drive.followTrajectorySequence(backDropLineUpMiddle);
                break;
        }

        robot.backDropServo().setPosition(backDropServoLOW);
        sleep(1500);

        drive.followTrajectorySequence(parkProper);
        robot.intakeLock().setPosition(1);
        drive.followTrajectory(returnToStart);
        sleep(5000);
    }

    public void driveToHit(SampleMecanumDrive drive, HardwareSoftware robot, double power, double timeout){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        timer.startTime();

        while(!robot.isBackDrop() && timer.time(TimeUnit.MILLISECONDS) <= timeout){
            drive.setMotorPowers(power, power, power, power);
        }
        drive.setMotorPowers(0,0,0,0);
    }
}