//RRHardwarePresets.java
//Test

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RRHardwarePresets{

    HardwareMap HwMap;

    //DC Motors
    DcMotor left1;
    DcMotor left2;
    DcMotor right1;
    DcMotor right2;

    //Servos
    Servo claw;
    Servo smacker;

    //Sensors
    ColorSensor joule;
    ColorSensor floor;

    //Constant values
    final int DRIVE_STRAIGHT = 5000;
    final double CLAW_OPEN = 0.7;
    final double CLAW_CLOSED = 0.2;
    final double SMACKER_UP = 0.28;
    final double SMACKER_DOWN = 1;

    int driveMode;

    //Constructor
    public RRHardwarePresets(int dm){ //dm = 0 RUN_USING_ENCODER, dm = 1 RUN_TO_POSITION
        this.driveMode = dm;
        System.out.println("Created new RRHardwarePresets Object!");
    }

    public void init(HardwareMap hwm){
        HwMap = hwm;

        //Tank drive DC Motors mappings
        left1 = HwMap.dcMotor.get("left1");
        left2 = HwMap.dcMotor.get("left2");
        right1 = HwMap.dcMotor.get("right1");
        right2 = HwMap.dcMotor.get("right2");

        //Servo mappings
        claw = HwMap.servo.get("claw");
        smacker = HwMap.servo.get("smacker");

        //Servo starting positions.
        //claw.setPosition(CLAW_CLOSED);
        //smacker.setPosition(SMACKER_UP);

        //Sensor mappings
        joule = HwMap.colorSensor.get("joule");
        floor = HwMap.colorSensor.get("floor");

        if(this.driveMode == 0){
            //DC Motor encoder modes.
            left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }else if(this.driveMode == 1){
            left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        //DC Motor directions.
        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);
        right1.setDirection(DcMotorSimple.Direction.FORWARD);
        right2.setDirection(DcMotorSimple.Direction.FORWARD);

        //DC Motor stop behavior.
        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    void resetEncoders(){
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
