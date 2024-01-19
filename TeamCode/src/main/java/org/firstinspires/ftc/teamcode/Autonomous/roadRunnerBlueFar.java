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
@Autonomous(name="roadRunnerBlueFar")
public class roadRunnerBlueFar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(-33, -60, Math.toRadians(90));

        HardwareSoftware robot = new HardwareSoftware();
        robot.init(hardwareMap);

        drive.setPoseEstimate(start);

        Trajectory traj1 = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(-31, -33), Math.toRadians(90))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(-31, 0), Math.toRadians(90))
                .build();

        Trajectory turn1 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(-65, 0), Math.toRadians(180))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(turn1.end())
                .splineTo(new Vector2d(-110, 0), Math.toRadians(180))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(-107, -33), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-113, -33), Math.toRadians(0))
                .build();

        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineToConstantHeading(new Vector2d(-113, -60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-116, -60), Math.toRadians(0))
                .build();


        waitForStart();
           robot.pixeldrop().setPosition(0);
        drive.followTrajectory(traj1);
        sleep(2000);
        drive.followTrajectory(traj2);
        drive.followTrajectory(turn1);
           robot.pixeldrop().setPosition(1);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        sleep(2000);
        drive.followTrajectory(traj5);
    }
}
