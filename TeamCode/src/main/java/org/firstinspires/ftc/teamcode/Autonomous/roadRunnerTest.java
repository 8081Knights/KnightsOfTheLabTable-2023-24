package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



@Autonomous(name="RoadRunner Trajectory Test")
public class roadRunnerTest extends LinearOpMode {



    @Override
    public void runOpMode() throws InterruptedException {
        HardwareSoftware robot = new HardwareSoftware();

        robot.init(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory test1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(70, 0), 0)
                .build();

        Trajectory test2 = drive.trajectoryBuilder(test1.end())
                .splineTo(new Vector2d(0, 0), 0)
                .build();


        waitForStart();

        drive.followTrajectory(test1);
        drive.followTrajectory(test2);
    }
}
