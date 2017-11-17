//RRHardwarePresets.java
package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


public class RRHardwarePresets{

    //Motors, Servos and Sensors
    public static DcMotor left1;
    public static DcMotor left2;
    public static DcMotor right1;
    public static DcMotor right2;
    public static DcMotor turretMotor;
    public static Servo claw;
    public static Servo jewelArm;
    public static Servo elbow;
    public static Servo wrist;
    public static ColorSensor jewelSensor;
    public static ColorSensor floorSensor;
    public static BNO055IMU imu;
    //public static BNO055IMU imu2;
    public static HardwareMap HwMap;

    //Vuforia Information
    public static final String TAG = "Vuforia VuMark Sample";
    public static OpenGLMatrix lastLocation = null;
    public static VuforiaLocalizer vuforia;

    //Constants
    public static final double JEWEL_ARM_UP = 0.70;
    public static final double JEWEL_ARM_DOWN = 0.05;
    public static final double ELBOW_UNFOLDED = 0.30;
    public static final double ELBOW_FOLDED = 1.00;
    public static final double WRIST_UNFOLDED = 0.30;
    public static final double WRIST_FOLDED = 1.00;

    //Need to get these values correct for followLine() to work.
    public static final double FLOOR_COLOR = 0.0;
    public static final double RED_LINE_COLOR = 0.0;
    public static final double BLUE_LINE_COLOR = 0.0;

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
        elbow = HwMap.servo.get("elbow");
        wrist = HwMap.servo.get("wrist");

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
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Sensor LED control.
        jewelSensor.enableLed(false);
        floorSensor.enableLed(false);

        //Initial Servo positions.
        jewelArm.setPosition(JEWEL_ARM_UP); //Raised

        //IMU initialization parameters
        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        IMUParameters.loggingEnabled = true;
        IMUParameters.loggingTag = "IMU";
        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //imu1 initialization, change name to imu1 when we add the second IMU. Will need to be updated on phone config.
        imu = HwMap.get(BNO055IMU.class, "imu1");
        imu.initialize(IMUParameters);

//        //imu2 initialization
//        imu2 = HwMap.get(BNO055IMU.class, "imu2");
//        imu2.initialize(IMUParameters);

        //Vuforia Initialization parameters.
        //Sets camera feed to display on phone.
        int cameraMonitorViewId = HwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", HwMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters VuforiaParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        // OR...  Do Not Activate the Camera Monitor View, to save power
        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        //License obtained.
        VuforiaParameters.vuforiaLicenseKey = "AU0kxmH/////AAAAGV4QPVzzlk6Hl969cSL2pmM4F6TuzhWZS/dKbY45MEzS31OYJxLbKewdt1CSFrmpvrpPnIYZyBJt3kFRJQCtEXet0LHd2KtBB5NsDTuBADfgIsQk+7TSWSTFDjSi8SpKaXtAjZPKePwGDaIKf5VK6mRBYaWxqTHpZFBlelejLHxib8qweOFrJjKTsbgsb2pwVNFhDeJabbI5aed8JSI8LxHs0368ezQfnCz3UK9u8pC1DkKgcwdgoJ0OXBKChXB4v2lEnIrQf7ROYcPtVuRJJ5/prBoyfR11pvp69iCA25Cttz9xVsdZ9VliuQJ4UO37Hzhz1dB2SPnxTQQmCJMDoDKqe3wpiCFu8ThQ4pmS05ka";
        //Sets phone to use back camera.
        VuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        //Initializes VuforiaLocalizer object as vuforia.
        this.vuforia = ClassFactory.createVuforiaLocalizer(VuforiaParameters);
    }

    //---UNIVERSAL METHODS BELOW---

    //Servo we want to move, Position we want to move to, Number of servo movements we want, the time we want this movement to occur over in milliseconds.
    public void moveServo(Servo targetServo, double targetPosition, int steps, long timeInMilli){
        //Total distance to travel.
        double distanceToTravel = Math.abs(targetServo.getPosition() - targetPosition);
        //Unit conversion to nanoseconds.
        long time = timeInMilli * 1000000;
        //Per Step values.
        //double distanceToTravelPerStep = (distanceToTravel / steps);
        long timePerStep = time / steps;
        //Loops number of steps.
        double distanceToTravelPerStep;
        if(targetPosition - targetServo.getPosition() >= 0){
            distanceToTravelPerStep = (distanceToTravel / steps);
        }else{
            distanceToTravelPerStep = (distanceToTravel / steps) * -1;
        }
        for(int counter = 0; counter < steps; counter++){
            double initialTime = System.nanoTime();
            double currentPosition = targetServo.getPosition(); //Gets current arm position.
            //if(movementFlag == 0) {
            targetServo.setPosition(currentPosition + distanceToTravelPerStep);//Moves the arm.
            while((System.nanoTime() - initialTime) < timePerStep){
                //Wait.
            }
        }
    }

    //Drives at given power and a given distance unless floorSensors interrupt it by seeing the given color. ("red" or "blue").
    public void driveForwardWithInterrupt(double power, int distance, String color) {
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        //Sets target distance. Set to negative distance because motor was running backwards.
        setAllTargetPositions(-distance);
        //Sets to RUN_TO_POSITION mode
        setRunMode("RUN_TO_POSITION");
        //Sets power for DC Motors.
        setMotorPower(power);
        //Waits while driving to position.
        while(anyMotorsBusy()){
            if (color.equals("red")) {
                if(floorSensor.red() > 50) {//Level of Red required to stop.
                    setMotorPower(0.0);
                }
            }
            if(color.equals("blue")){
                if (floorSensor.blue() > 50) {//Level of Blue required to stop.
                    setMotorPower(0.0);
                }
            }
        }
        //Stops driving by setting power to 0.0.
        setMotorPower(0.0);
        //Sets back to RUN_USING_ENCODER mode.
        setRunMode("RUN_USING_ENCODER");
    }

    //Takes power and distance to rotate and "CW" clockwise or "CCW" as directional input.
    public void turnDirection(double power, int distance, String direction) {
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        setRunMode("RUN_TO_POSITION");
        if (direction.equals("CCW")) {
            this.left1.setTargetPosition(distance);
            this.left2.setTargetPosition(distance);
            this.right1.setTargetPosition(-distance);
            this.right2.setTargetPosition(-distance);
        } else if (direction.equals("CW")) {
            this.left1.setTargetPosition(-distance);
            this.left2.setTargetPosition(-distance);
            this.right1.setTargetPosition(distance);
            this.right2.setTargetPosition(distance);
        }
        setMotorPower(power);
        //Waits while turning.
        while(anyMotorsBusy()){
            //Spinning
            //Waiting while turning.
        }
        //Stop motors.
        setMotorPower(0.0);
        //Sets mode back to RUN_USING_ENCODER
        setRunMode("RUN_USING_ENCODER");
    }

    //Drives forward a certain distance at a certain speed. Only use if no intention to interrupt.
    public void driveForwardSetDistance(double power, int distance) {
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        //Sets target distance. Set to negative distance because motor was running backwards.
        setAllTargetPositions(-distance);
        //Sets to RUN_TO_POSITION mode
        setRunMode("RUN_TO_POSITION");
        //Sets power for DC Motors.
        setMotorPower(power);
        //Waits while driving to position.
        while(anyMotorsBusy()){
            //Spinning.
            //Waiting for robot to arrive at destination.
        }
        //Stops driving by setting power to 0.0.
        setMotorPower(0.0);
        //Sets back to RUN_USING_ENCODER mode.
        setRunMode("RUN_USING_ENCODER");
    }

    //Sets the run mode of all DC motors. Test is this works in both autonomous and teleOp modes.
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

    //Returns TRUE if any drive motors are busy and FALSE if not.
    public boolean anyMotorsBusy() {
        if (this.left1.isBusy() || this.left2.isBusy() || this.right1.isBusy() || this.right2.isBusy()) {
            return (true);
        } else {
            return (false);
        }
    }

    //Sets all drive motor power.
    public void setMotorPower(double power) {
        this.left1.setPower(power);
        this.left2.setPower(power);
        this.right1.setPower(power);
        this.right2.setPower(power);
    }

    //Sets all motors target position.
    public void setAllTargetPositions(int distance) {
        left1.setTargetPosition(distance);
        left2.setTargetPosition(distance);
        right1.setTargetPosition(distance);
        right2.setTargetPosition(distance);
    }
}