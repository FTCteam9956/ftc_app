//Position.java
package org.firstinspires.ftc.teamcode;

public class Position{

    public static RRHardwarePresets robot = new RRHardwarePresets();

    public int shoulderPosition;
    public double elbowPosition;
    public double wristPosition;

    public Position(int sp, double ep, double wp){
        this.shoulderPosition = sp;
        this.elbowPosition = ep;
        this.wristPosition = wp;
    }

    public Position(){}

    public void execute(){
        robot.shoulder.setTargetPosition(shoulderPosition);
        robot.elbow.setPosition(elbowPosition);
        robot.wrist.setPosition(wristPosition);
    }
}
