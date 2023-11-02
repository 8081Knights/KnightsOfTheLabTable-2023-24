package org.firstinspires.ftc.teamcode;


/*
This code is a basic write out of the odometry tracking algorithm 8081 Knights of the Lab Table intends to use
in order to track the overall position of the robot

This is by no means a final product and will likely require a lot more work

Largely pioneered by Alex S. Rumer over the course of a few days and lots of website tutorials and pseudocode
 */


public class OdometryCalc {

    //Distance between the two parallel odometers
    //TODO: Actually measure and tune this value
    private final double trackWidth = 5.5;

    //Distance of the perpendicular odometer from the center of the robot
    //TODO: Actually measure and tune this value
    private final double forwardOffset = 7;

    //TODO: Actually measure and tune this value
    private final double ticksToInch = 1235.45;


    //Calculates the Pose (XY Coordinates, and the Heading in Degrees)
    //prevL and prevR are the previously recorded encoder values of the two parallel odometry wheels
    //prevH is the previously recorded encoder value of the perpendicular odometry wheel
    //prevHeading is the previously recorded heading of the robot in degrees
            //Can either be a gyro input or calculated from previous odometry data
    public double[] poseCalc(HardwareSoftware robot, int prevL, int prevR, int prevH, double prevHeading){

        //Initialize the return variable as an array of doubles
        double[] pose;


        //Convert Degree heading to radians for further calculations
        prevHeading = Math.toRadians(prevHeading);


        //Get the wheel travel values (Inches) from encoder inputs
        double dL = -(robot.leftEncoder().getCurrentPosition() - prevL)/ticksToInch;
        double dR = (robot.rightEncoder().getCurrentPosition() - prevR)/ticksToInch;
        double dH = (robot.frontEncoder().getCurrentPosition() - prevH)/ticksToInch;

        //Calculate the change in heading from previous heading using encoder data
        double dHead = prevHeading + (dL - dR)/trackWidth;

        //Calculate the Center movement of the robot
        //Just averages the Left and Right change in X
        double dCenter = (dL + dR)/2;

        //Calculates the overall Horizontal movement of the robot
        //Accounts for changes in heading
        double dHorizontal = dH - (forwardOffset*dHead);

        //Calculate overall movement in the X direction
        //Utilizes a Method called pose exponential, it took an hour to multiply this fatass matrix :'(
        double dX = (dCenter*(Math.cos(prevHeading)*Math.sin(dHead) - Math.sin(prevHeading)*(1-Math.cos(dHead)))
                + dHorizontal*(Math.cos(prevHeading)*(Math.cos(dHead) - 1) - Math.sin(prevHeading)*Math.sin(dHead)))/dHead;


        //Calculate overall movement in the Y direction
        //Utilizes a Method called pose exponential, it took an hour to multiply this fatass matrix :'(
        double dY = (dCenter*(Math.sin(prevHeading)*Math.sin(dHead) + Math.cos(prevHeading)*(1 - Math.cos(dHead)))
                + dHorizontal*(Math.sin(prevHeading)*(Math.cos(dHead) - 1) + Math.cos(prevHeading)*Math.sin(dHead)))/dHead;


        //Return the overall XY change and the change in heading
        pose = new double[]{dX, dY, Math.toDegrees(dHead)};
        return pose;

    }
}
