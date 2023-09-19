package org.firstinspires.ftc.teamcode;

public class OdometryCalc {

    //Distance between the two parallel odometers
    private final double trackWidth = 10;

    //Distance of the perpendicular odometer from the center of the robot
    private final double forwardOffset = 10;
    public double[] poseCalc(HardwareSoftware robot, int prevL, int prevR, int prevH, double prevHeading){
        double[] pose;


        int dL = robot.FLdrive().getCurrentPosition() - prevL;
        int dR = robot.FRdrive().getCurrentPosition() - prevR;

        int dH = robot.BLdrive().getCurrentPosition() - prevH;

        double dHead = (dL - dR)/trackWidth;

        double dCenter = (dL + dR)/2;

        double dHorizontal = dH - (forwardOffset*dHead);

        double dX = (dCenter*(Math.cos(prevHeading)*Math.sin(dHead) - Math.sin(prevHeading)*(1-Math.cos(dHead)))
                + dHorizontal*(Math.cos(prevHeading)*(Math.cos(dHead) - 1) - Math.sin(prevHeading)*Math.sin(dHead)))/dHead;

        double dY = (dCenter*(Math.sin(prevHeading)*Math.sin(dHead) + Math.cos(prevHeading)*(1 - Math.cos(dHead)))
                + dHorizontal*(Math.sin(prevHeading)*(Math.cos(dHead) - 1) + Math.cos(prevHeading)*Math.sin(dHead)))/dHead;


        pose = new double[]{dX, dY, dHead};







        return pose;

    }
}
