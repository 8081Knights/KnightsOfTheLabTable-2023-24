package org.firstinspires.ftc.teamcode.Tests.JSONStuff.JsonTele;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.opencsv.CSVWriter;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareSoftware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class RecorderOfJsonWrittenLinearly extends LinearOpMode {

    public static double[][][] boxes = new double[3][100][5];
    public static String filePath = "";
    ElapsedTime timeryy = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public int threadIterationset = 0;
    int dataEntry = 0;

    Thread[] runWriter ={
            new Thread(new Writer(0)),
            new Thread(new Writer(1)),
            new Thread(new Writer(2))
    };

    private void reset() {
        timeryy.reset();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        HardwareSoftware robot = new HardwareSoftware();
        robot.init(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        reset();
        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("Gyro Heading:", robot.getHeading());

            if (dataEntry == 100) {
                dataEntry = 0;
                if (threadIterationset < 3) {
                    ++threadIterationset;
                } else {
                    threadIterationset = 0;
                }
            }
            boxes[threadIterationset][dataEntry][0] = poseEstimate.getX();
            boxes[threadIterationset][dataEntry][1] = poseEstimate.getY();
            boxes[threadIterationset][dataEntry][2] = Math.toDegrees(poseEstimate.getHeading());
            boxes[threadIterationset][dataEntry][3] = robot.getHeading();
            boxes[threadIterationset][dataEntry][4] = timeryy.time();
            ++dataEntry;
        }
    }


    public static class Writer implements Runnable {

        int iterationNumber;

        public Writer(int iterationNumber) {
            this.iterationNumber = iterationNumber;
        }
        @Override
        public void run() {
            File file = new File(RecorderOfJsonWrittenLinearly.filePath);
            try {
                //Check to make sure destination file exists. If not create the destination file
                if (!file.exists()) {
                    file.createNewFile();
                }

                // create FileWriter object with file as parameter
                FileWriter outPutFile = new FileWriter(file, true);

                // create CSVWriter object fileWriter object as parameter
                CSVWriter writer = new CSVWriter(outPutFile);

                //create reader to check if file already has a header
                Scanner reader;
                try {
                    reader = new Scanner(new FileReader(RecorderOfJsonWrittenLinearly.filePath));
                } catch (FileNotFoundException e) {
                    throw e;
                }

                reader.useDelimiter("\n");

                if (!reader.hasNext()) {
                    // adding header to csv
                    String[] header = {"EstimatedX", "EstimatedY","EncoderHeading", "Gyro Heading", "Timestamp"};
                    writer.writeNext(header);
                }

                int localArrayLength = 0;
                while (localArrayLength < 100) {
                    writer.writeNext(new String[]{
                            Double.toString(RecorderOfJsonWrittenLinearly.boxes[iterationNumber][localArrayLength][0]),
                            Double.toString(RecorderOfJsonWrittenLinearly.boxes[iterationNumber][localArrayLength][1]),
                            Double.toString(RecorderOfJsonWrittenLinearly.boxes[iterationNumber][localArrayLength][2]),
                            Double.toString(RecorderOfJsonWrittenLinearly.boxes[iterationNumber][localArrayLength][3]),
                            Double.toString(RecorderOfJsonWrittenLinearly.boxes[iterationNumber][localArrayLength][4])
                    });
                    ++localArrayLength;
                }

                // closing writer connection
                writer.close();
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }
        }
    }
}
