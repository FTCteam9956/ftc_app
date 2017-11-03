//RRHardwarePresets.java
//Test

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RRHardwarePresets{
    public DcMotor left1;
    public DcMotor left2;
    public DcMotor right1;
    public DcMotor right2;
    public Servo claw;
    public Servo jewelArm;
    public ColorSensor jewelSensor;
    public ColorSensor floorSensor;
    HardwareMap HwMap;

    //Constructor
    public RRHardwarePresets(){System.out.println("Created new RRHardwarePresets Object!");}

    public void init(HardwareMap hwm){
        HwMap = hwm;

        //Mappings.
        left1 = HwMap.dcMotor.get("left1");
        left2 = HwMap.dcMotor.get("left2");
        right1 = HwMap.dcMotor.get("right1");
        right2 = HwMap.dcMotor.get("right2");
        claw = HwMap.servo.get("claw");
        jewelArm = HwMap.servo.get("jewelArm");
        jewelSensor = HwMap.colorSensor.get("jewelSensor");
        floorSensor = HwMap.colorSensor.get("floorSensor");

        //DC Motor directions.
        left1.setDirection(DcMotorSimple.Direction.FORWARD);
        left2.setDirection(DcMotorSimple.Direction.FORWARD);
        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);

        //DC Motor stop behavior.
        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Sensor LED control.
        jewelSensor.enableLed(true);
        floorSensor.enableLed(true);

        //Initial Servo positions.
        claw.setPosition(0.2); //Closed
        jewelArm.setPosition(0.28); //Raised

    }
}