package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Autonomous(name="roadRunnerBlueClose")
public class roadRunnerBlueClose extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(-81, -60, Math.toRadians(90));

        HardwareSoftware robot = new HardwareSoftware();
        robot.init(hardwareMap);

        drive.setPoseEstimate(start);

        Trajectory traj1 = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(-83, -33), Math.toRadians(90))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(-107, -33), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-113, -33), Math.toRadians(0))
                .build();

        Trajectory traj6 = drive.trajectoryBuilder(traj4.end())
                .splineToConstantHeading(new Vector2d(-117, -33), Math.toRadians(0))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj6.end())
                .splineToConstantHeading(new Vector2d(-113, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-125, -60), Math.toRadians(0))
                .build();


        waitForStart();
        robot.pixeldrop().setPosition(1);
        drive.followTrajectory(traj1);
        sleep(2000);
        robot.pixeldrop().setPosition(0);
        drive.followTrajectory(traj4);
        drive.followTrajectory(traj6);
        sleep(2000);
        drive.followTrajectory(traj5);
    }
}

