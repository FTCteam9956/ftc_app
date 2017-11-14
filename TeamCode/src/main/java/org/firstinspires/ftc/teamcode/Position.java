package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class Position{
    double wristPosition;
    double elbowPosition;
    int shoulderPosition;

    //Constructor
    public Position(double wp, double ep, int sp){
        this.wristPosition = wp;
        this.elbowPosition = ep;
        this.shoulderPosition = sp;
    }

}
