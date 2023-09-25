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

    HardwareSoftware robot = new HardwareSoftware();
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        Trajectory test = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(10, 0), 0)
                .build();


        waitForStart();

        drive.followTrajectory(test);
    }
}
