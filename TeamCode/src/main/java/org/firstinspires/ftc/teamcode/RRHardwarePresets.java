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
    public DcMotor left1;
    public DcMotor left2;
    public DcMotor right1;
    public DcMotor right2;
    public DcMotor turretMotor;
    public Servo claw;
    public Servo jewelArm;
    public Servo elbow;
    public Servo wrist;
    public ColorSensor jewelSensor;
    public ColorSensor floorSensor;
    public BNO055IMU imu1;
    public BNO055IMU imu2;
    HardwareMap HwMap;

    //Vuforia Information
    public static final String TAG = "Vuforia VuMark Sample";
    public OpenGLMatrix lastLocation = null;
    public VuforiaLocalizer vuforia;

    //Constants
    public final double JEWEL_ARM_UP = 0.70;
    public final double JEWEL_ARM_DOWN = 0.05;

    //Servo positional constant.
    public final double ELBOW_UNFOLDED = 0.30;
    public final double ELBOW_FOLDED = 1.00;
    public final double WRIST_UNFOLDED = 0.30;
    public final double WRIST_FOLDED = 1.00;


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

//        //IMU initialization parameters
//        BNO055IMU.Parameters IMUParameters = new BNO055IMU.Parameters();
//        IMUParameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        IMUParameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        IMUParameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        IMUParameters.loggingEnabled = true;
//        IMUParameters.loggingTag = "IMU";
//        IMUParameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//
//        //imu1 initialization
//        imu1 = HwMap.get(BNO055IMU.class, "imu1");
//        imu1.initialize(IMUParameters);
//
//        //imu2 initialization
//        imu2 = HwMap.get(BNO055IMU.class, "imu2");
//        imu2.initialize(IMUParameters);


        //Vuforia Initialization
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

    //My attempt at creating servoSpeed.
    //Servo we want to move, Position we want to move to, Number of servo movements we want, the time we want this movement to occur over in milliseconds.
    public static void moveServo(Servo targetServo, double targetPosition, int steps, long timeInMilli){
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


















    public static void moveMultipleServo(Servo targetServo1, Servo targetServo2, double targetPosition1, double targetPosition2, int steps, long timeInMilli){
        //Total distance to travel.
        double distanceToTravel1 = Math.abs(targetServo1.getPosition() - targetPosition1);
        double distanceToTravel2 = Math.abs(targetServo2.getPosition() - targetPosition2);
        //Unit conversion to nanoseconds.
        long time = timeInMilli * 1000000;
        //Per Step values.
        //double distanceToTravelPerStep = (distanceToTravel / steps);
        long timePerStep = time / steps;
        //Loops number of steps.
        double distanceToTravelPerStep1;
        double distanceToTravelPerStep2;

        if(targetPosition1 - targetServo1.getPosition() >= 0){
            distanceToTravelPerStep1 = (distanceToTravel1 / steps);
        }else{
            distanceToTravelPerStep1 = (distanceToTravel1 / steps) * -1;
        }

        if(targetPosition2 - targetServo2.getPosition() >= 0){
            distanceToTravelPerStep2 = (distanceToTravel2 / steps);
        }else{
            distanceToTravelPerStep2 = (distanceToTravel2 / steps) * -1;
        }

        for(int counter = 0; counter < steps; counter++){
            double initialTime = System.nanoTime();

            double currentPosition1 = targetServo1.getPosition(); //Gets current arm position.
            double currentPosition2 = targetServo2.getPosition(); //Gets current arm position.

            targetServo1.setPosition(currentPosition1 + distanceToTravelPerStep1);//Moves the arm.
            targetServo2.setPosition(currentPosition2 + distanceToTravelPerStep2);//Moves the arm.
            while((System.nanoTime() - initialTime) < timePerStep){
                //Wait.
            }
        }
    }
}