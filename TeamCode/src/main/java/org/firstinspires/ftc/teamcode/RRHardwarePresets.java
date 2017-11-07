//RRHardwarePresets.java

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
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
    public DcMotor turretMotor;
    public Servo claw;
    public Servo jewelArm;
    public ColorSensor jewelSensor;
    public ColorSensor floorSensor;
    BNO055IMU imu;
    HardwareMap HwMap;

    //Constants
    public final double JEWEL_ARM_DOWN = 0.05;
    public final double JEWEL_ARM_UP = 0.70;

    //Need to get these values correct for followLine() to work.
    public final double FLOOR_COLOR = 0.0;
    public final double RED_LINE_COLOR = 0.0;
    public final double BLUE_LINE_COLOR = 0.0;

    //Constructor
    public RRHardwarePresets(){
        System.out.println("Created new RRHardwarePresets Object!");
    }

    public void init(HardwareMap hwm){
        HwMap = hwm;

        //Mappings.
        left1 = HwMap.dcMotor.get("left1");
        left2 = HwMap.dcMotor.get("left2");
        right1 = HwMap.dcMotor.get("right1");
        right2 = HwMap.dcMotor.get("right2");
        turretMotor = HwMap.dcMotor.get("turretMotor");
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
        jewelSensor.enableLed(false);
        floorSensor.enableLed(false);

        //IMU initialization parameters
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //IMU initialization
        imu = HwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //Initial Servo positions.
        //claw.setPosition(0.2); //Closed
        //jewelArm.setPosition(0.30); //Raised
    }
    //Sets the run mode of all DC motors.
    public void setRunMode(String input){
        if(input.equals("STOP_AND_RESET_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if(input.equals("RUN_WITHOUT_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(input.equals("RUN_USING_ENCODER")) {
            this.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(input.equals("RUN_TO_POSITION")) {
            this.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
}