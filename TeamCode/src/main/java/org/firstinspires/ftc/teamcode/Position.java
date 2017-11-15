package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.ArrayList;

import java.util.Locale;

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

    //Takes in 2 imus and returns 2 Strings in an ArrayList about their headings.
    public static ArrayList<String> getHeadingInfo(BNO055IMU IMU1, BNO055IMU IMU2){

        ArrayList<String> returnList = new ArrayList<>();

        //Orientations of IMU1 and IMU2.
        Orientation angles1 = IMU1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        Orientation angles2 = IMU2.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        returnList.add(formatAngle(angles1.angleUnit, angles1.firstAngle));
        returnList.add(formatAngle(angles2.angleUnit, angles2.firstAngle));

        return(returnList);
    }

    //These 2 methods are from SensorBNO055IIMU.java
    public static String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    public static String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}