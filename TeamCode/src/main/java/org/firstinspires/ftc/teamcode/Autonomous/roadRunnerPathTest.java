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
@Autonomous(name="RoadRunner Path Test")
public class roadRunnerPathTest extends LinearOpMode {

    SampleMecanumDrive drive;
    HardwareSoftware robot = new HardwareSoftware();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(-35, -61, Math.toRadians(90));

        drive.setPoseEstimate(start);

        Trajectory traj1 = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(-35, 0), 0)
                .splineTo(new Vector2d(45, 0), Math.toRadians(0))
                .splineTo(new Vector2d(50, -37), Math.toRadians(0))
                .build();

        Trajectory traj1Back = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(0, -37), Math.toRadians(0))
                .build();


        waitForStart();

        drive.followTrajectory(traj1);
        sleep(100);
//        drive.followTrajectory(traj1Back);


        sleep(1000000);



    }
}
