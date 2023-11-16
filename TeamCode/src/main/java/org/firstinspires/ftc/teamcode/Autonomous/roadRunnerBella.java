package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@Autonomous(name="roadrunner bella test")
public class roadRunnerBella extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(-33, 60, Math.toRadians(270));

        drive.setPoseEstimate(start);

        Trajectory traj1 = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(-6, 60), 0)
                .build();

//               Trajectory turn1 = drive.trajectoryBuilder(new Pose2d (), false)
//                 .splineTo(new Vector2d(-6, -60), Math.toRadians(90))
//                 .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(-3)
                .build();


        Trajectory turn2 = drive.trajectoryBuilder(new Pose2d (), false)
                .splineTo(new Vector2d(36, -58), Math.toRadians(180))
                .build();


        Trajectory traj3 = drive.trajectoryBuilder(turn2.end())
                .forward(27)
                .build();



        waitForStart();
        drive.followTrajectory(traj1);
    }
}
