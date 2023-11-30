package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.CameraLogicRed;
import org.firstinspires.ftc.teamcode.Autonomous.CameraLogicBlue;
import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceRunner;
import org.firstinspires.ftc.teamcode.util.LynxModuleUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kA;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kStatic;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.kV;



@Autonomous(name="AutoRed")
public class AutoRed extends LinearOpMode  {
    SampleMecanumDrive drive;

    OpenCvWebcam cam;
    HardwareSoftware robot = new HardwareSoftware();
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        robot.init(hardwareMap);


        String pos = "";

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


        //begin calibrating the gyro
//        telemetry.log().add("Gyro Calibrating. Do Not Move!");
//        robot.gyro().isCalibrating();
//
//        // Wait until the gyro calibration is complete
//        timer.reset();
//        while (!isStopRequested() && robot.gyro().isCalibrating()) {
//            telemetry.addData("calibrating", "%s", Math.round(timer.seconds()) % 2 == 0 ? "|.." : "..|");
//            telemetry.update();
//            sleep(50);
//        }
//
//        telemetry.log().clear(); telemetry.log().add("Gyro Calibrated. Press Start.");
//        telemetry.clear(); telemetry.update();



        waitForStart();


        //Adds data for the position of the ducky on the barcode
        telemetry.addLine("Position: " + pos);
        telemetry.update();
        telemetry.clearAll();




        Pose2d start = new Pose2d(-33, 60, Math.toRadians(270));
        HardwareSoftware robot = new HardwareSoftware();
        robot.init(hardwareMap);
        drive.setPoseEstimate(start);
        Trajectory traj1 = drive.trajectoryBuilder(start)
                .splineTo(new Vector2d(-31, 33), Math.toRadians(270))
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(-31, 0), Math.toRadians(270))
                .build();
        Trajectory turn1 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(-65, 0), Math.toRadians(180))
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(turn1.end())
                .splineTo(new Vector2d(-110, 0), Math.toRadians(180))
                .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .splineTo(new Vector2d(-107, 33), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-113, 33), Math.toRadians(0))
                .build();
        Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
                .splineToConstantHeading(new Vector2d(-113, 60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-116, 60), Math.toRadians(0))
                .build();
        robot.pixeldrop().setPosition(0);
        waitForStart();
        drive.followTrajectory(traj1);
        sleep(2000);
        robot.pixeldrop().setPosition(1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(turn1);
        drive.followTrajectory(traj3);
        drive.followTrajectory(traj4);
        sleep(2000);
        drive.followTrajectory(traj5);

    }


    //  break;

         //   case "MIDDLE":


             //   break;

          //  default:


            //    break;


        }

