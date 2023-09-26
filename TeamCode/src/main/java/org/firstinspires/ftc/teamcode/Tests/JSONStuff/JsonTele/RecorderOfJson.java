package org.firstinspires.ftc.teamcode.Tests.JSONStuff.JsonTele;

import com.opencsv.CSVWriter;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.HardwareSoftware;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Scanner;

public class RecorderOfJson extends OpMode {

    HardwareSoftware robot = new HardwareSoftware();
    double maxSpeed = .8;
    double minSpeed = 0.3;
    double speedMult = maxSpeed;
    boolean g1aDown = false;
    @Override
    public void init() {
        robot.init(hardwareMap);
    }
    double driveTurn;
    double gamepadXCoordinate;
    double gamepadYCoordinate;
    double robotDegree;

    double rotX;
    double rotY;
    double denominator;


    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;
    float[][][] boxes = new float[3][100][7];
    String filePath = "";

    @Override
    public void loop() {
        driveTurn = -gamepad1.right_stick_x;
        gamepadXCoordinate = gamepad1.left_stick_x; //this simply gives our x value relative to the driver
        gamepadYCoordinate = -gamepad1.left_stick_y; //this simply gives our y vaue relative to the driver

        robotDegree = Math.toRadians(robot.getHeading());

        rotX = gamepadXCoordinate * Math.cos(robotDegree) - gamepadYCoordinate * Math.sin(robotDegree);
        rotY = gamepadXCoordinate * Math.sin(robotDegree) + gamepadYCoordinate * Math.cos(robotDegree);
        denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(driveTurn), 1);

        frontLeftPower = ((rotY + rotX - driveTurn) / denominator)*speedMult;
        backLeftPower = ((rotY - rotX - driveTurn) / denominator)*speedMult;
        frontRightPower = ((rotY - rotX + driveTurn) / denominator)*speedMult;
        backRightPower = ((rotY + rotX + driveTurn) / denominator)*speedMult;

        robot.FRdrive().setPower(frontRightPower);
        robot.FLdrive().setPower(frontLeftPower);
        robot.BRdrive().setPower(backRightPower);
        robot.BLdrive().setPower(backLeftPower);

        if(gamepad1.a){
            g1aDown = true;
        }
        if(!gamepad1.a && g1aDown){
            g1aDown = false;
            if(speedMult==maxSpeed){
                speedMult=minSpeed;
            }
            else{
                speedMult = maxSpeed;
            }
        }

    }
    /*public void recordPositionData(String filePath, double timeStamp)
    {
        // first create file object for file placed at location
        // specified by filepath
        File file = new File(filePath);


        //Format retrieved data to be stored in CSV
        String[] data = {
                Double.toString(timeStamp),
                Float.toString(frontLeft.getCurrentPosition()),
                Float.toString(frontRight.getCurrentPosition()),
                Float.toString(backLeft.getCurrentPosition()),
                Float.toString(backRight.getCurrentPosition()),
                Float.toString(gyro.getHeading())};
        try {

            //Check to make sure destination file exists. If not create the destination file
            if(!file.exists()){
                file.createNewFile();
            }




            // create FileWriter object with file as parameter
            FileWriter outPutFile = new FileWriter(file, true);

            // create CSVWriter object fileWriter object as parameter
            CSVWriter writer = new CSVWriter(outPutFile);

            //create reader to check if file already has a header
            Scanner reader;
            try{reader = new Scanner(new FileReader(filePath));}
            catch(FileNotFoundException e){throw e;}

            reader.useDelimiter("\n");

            if(!reader.hasNext()) {
                // adding header to csv
                String[] header = {"timeStamp", "frontLeft Encoder", "frontRight Encoder","backLeft Encoder","backRight Encoder", "Gyro Heading"};
                writer.writeNext(header);
            }

            writer.writeNext(data);

            // closing writer connection
            writer.close();
        }
        catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }*/

    public void writeArray(float[][][] array, int iteration) {
        File file = new File(filePath);
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
                reader = new Scanner(new FileReader(filePath));
            } catch (FileNotFoundException e) {
                throw e;
            }

            reader.useDelimiter("\n");

            if (!reader.hasNext()) {
                // adding header to csv
                String[] header = {"timeStamp", "frontLeft Encoder", "frontRight Encoder", "backLeft Encoder", "backRight Encoder", "Gyro Heading"};
                writer.writeNext(header);
            }

            int localArrayLength = 0;
            while (localArrayLength < 100) {
                writer.writeNext(new String[]{
                        Float.toString(array[iteration][localArrayLength][1]),
                        Float.toString(array[iteration][localArrayLength][2]),
                        Float.toString(array[iteration][localArrayLength][3]),
                        Float.toString(array[iteration][localArrayLength][4]),
                        Float.toString(array[iteration][localArrayLength][5]),
                        Float.toString(array[iteration][localArrayLength][6]),
                        Float.toString(array[iteration][localArrayLength][7]),
                });
            }

            // closing writer connection
            writer.close();
        } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
        }
    }
}
