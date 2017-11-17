//AutonomousTest2.java

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "AutonomousTest2", group = "Autonomous")
//@Disabled

//TO DO LIST:
//Test if methods can be put into RRHardwarePresets.java
//Test scanForVuMark()
//Clean up constants in RRHardwarePresets
//Test followLine()
//Move relevant code from TeleopForArms into TeleOpTest
//See if we can collect IMU data in TeleOpTest
//See if we can turn on LEDs for sensors and set initial arm positions in RRHardwarePresets
//Work with new Position class
//Update FIRST SDK
//Use difference in IMU readings to tell where our turret is pointed.



public class AutonomousTest2 extends LinearOpMode{
    RRHardwarePresets robot = new RRHardwarePresets();

    private ElapsedTime runtime = new ElapsedTime();
    private double servoPosition;


    private double servoDelayTime = 1000;


    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.jewelArm.setPosition(0.7);
        double elbow = gamepad2.left_stick_y;
        double elbowG = 0.01;
        //Vuforia Trackables.
        VuforiaTrackables relicTrackables = robot.vuforia.loadTrackablesFromAsset("RelicVuMark"); //I believe this loads VuMark data from the assets folder in FtcRobotController.
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        setRunMode("STOP_AND_RESET_ENCODER");
        setRunMode("RUN_USING_ENCODER");

        waitForStart();

        //Relic Trackables
        relicTrackables.activate();

        //CHANGE THIS BOOLEAN TO RUN TEST AREA. PUT IN SO WE DON'T HAVE TO RUN ENTIRE SCRIPT TO TEST.
        boolean testArea = true;

        if (testArea == true) {
//            //--TEST SCRIPT START--
//            //robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_DOWN_COMPLETE, 1000, 5000);
//            //robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_UP, 1000, 5000);//Moves jewelArm to JEWEL_ARM_DOWN_COMPLETE over a time period of 5 seconds with 10000 individual movements.
//            //robot.moveMultipleServo(robot.wrist, robot.elbow, robot.WRIST_UNFOLDED, robot.ELBOW_UNFOLDED, 5000, 5000);
//            String testString = scanForVuMark(0.05, 300, relicTemplate);
//            if(testString.equals("left")){
//                turnDirection(0.15, 1000, "CCW");
//            }
//            if(testString.equals("right")){
//                turnDirection(0.15, 1000, "CW");
//            }
//            if(testString.equals("center")){
//                driveForwardSetDistance(0.15, 1000);
//            }
//            if(testString.equals("none")){
//                //:(
//            }
//         while (opModeIsActive()){
//             if(runtime.time() > servoDelayTime){
//             servoPosition =+ elbow;
//             servoPosition = Range.clip(servoPosition, 0 , 1);
//                 robot.elbow.setPosition(servoPosition);
//                 runtime.reset();
//             }
//            currentPosition = changeNumber - distance;
//            targetServo.setPosition(currentPosition);
//            sleep(20);
//            changeNumber = changeNumber - distance;
//            stepNumber++;
            double currentPosition = 0;
            while(gamepad2.left_stick_y > 0){
//                servoPosition = Range.clip(servoPosition, 0 , 1);
//                robot.elbow.setPosition(currentPosition);
//                currentPosition = servoPosition += elbowG;
//                servoPosition = servoPosition + elbowG;
//                telemetry.addData("Position", currentPosition);
         }



        }else{
            //set testArea to true to only run code in the test area. Allows us to test individual components without running entire autonomous script.

            //--AUTO SCRIPT START--

            //Lowers jewel arm.
            robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_DOWN, 1000, 2000);

            //Reads color of ball and calls knockOffBall(0), knockOffBall(1) or does nothing.
            int loopBreak = 0;
            while (loopBreak == 0) {
                sleep(1000);
                if (robot.jewelSensor.red() > 52) {
                    knockOffBall(1);
                    telemetry.addData("Status", "Confirmed Red Ball!");
                    loopBreak = 1;
                } else if (robot.jewelSensor.red() <= 52) {
                    if (robot.jewelSensor.blue() > 20) {
                        knockOffBall(0);
                        telemetry.addData("Status", "Confirmed Blue Ball!");
                        loopBreak = 1;
                    } else {
                        telemetry.addData("Status", "Cannot determine color!");
                        loopBreak = 1;
                    }
                }
                telemetry.addData("Jewel Sensor - Red", robot.jewelSensor.red());
                telemetry.addData("Jewel Sensor - Blue", robot.jewelSensor.blue());
                telemetry.update();
            }
            sleep(500);

            //Raise JewelArm.
            robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_UP, 1000, 2000);
            sleep(500);

            //Drive forward off of balance stone.
            driveForwardSetDistance(0.15, 1200);
            sleep(500);

            //Search for and confirm VuMark.
            String targetPosition = scanForVuMark(0.15, 500, relicTemplate);

            //Turn dependent on what we read from vuMark.
            if(targetPosition.equals("left")){ //Turn CCW, then drive forward.
                turnDirection(0.15, 500, "CCW");
                driveForwardSetDistance(0.15, 100);
            }
            if(targetPosition.equals("right")){ //Turn CW, then drive forward.
                turnDirection(0.15, 500, "CW");
                driveForwardSetDistance(0.15, 100);
            }
            if(targetPosition.equals("center")){ //Just drive forward.
                driveForwardSetDistance(0.15, 100);
            }
            if(targetPosition.equals("none")){
                //Didn't find vuMark :(
            }

            //Drives forward and stops on line.

            //Turn towards triangle.

            //Continue...

        }
    }

    //Moves the jewelArm depending on what the input is.
    public void knockOffBall(int selection) {
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        setRunMode("RUN_TO_POSITION");
        if (selection == 0) {
            robot.turretMotor.setTargetPosition(200);
        }
        if (selection == 1) {
            robot.turretMotor.setTargetPosition(-200);
        }
        robot.turretMotor.setPower(0.15);
        while(robot.turretMotor.isBusy()){
            //Waiting while turret turns.
        }
        sleep(500);
        robot.turretMotor.setPower(0.0);
        setRunMode("RUN_USING_ENCODER");
    }

    //followValue should be the average of the 2 sensor.argb() color values.
    //lineColor can either be "red" or "blue"
    public void followLine(String lineColor, double speed) {
        //Sets mode to RUN_USING_ENCODER
        setRunMode("RUN_USING_ENCODER");
        int loopFlag = 0;
        double correction;
        double leftPower;
        double rightPower;
        double followValue = 0;
        //Decides follow value. Try with sensor.argb() values.
        if (lineColor.equals("red")) {
            followValue = ((robot.RED_LINE_COLOR + robot.FLOOR_COLOR) / 2);
        }
        if (lineColor.equals("blue")) {
            followValue = ((robot.BLUE_LINE_COLOR + robot.FLOOR_COLOR) / 2);
        }
        //Drives forward until it hits a line.
        driveForwardWithInterrupt(0.15, 300, lineColor);
        //Corrects power on left and right dcMotors to follow a line.
        while (loopFlag == 0) {
            //Get a correction
            correction = (followValue - robot.floorSensor.argb());
            if (correction <= 0.0) {
                leftPower = speed - correction;
                rightPower = speed;
            } else { //correction > 0.0
                leftPower = speed;
                rightPower = speed + correction;
            }
            //Setting motor speed.
            robot.left1.setPower(leftPower);
            robot.left2.setPower(leftPower);
            robot.right1.setPower(rightPower);
            robot.right2.setPower(rightPower);
        }
    }

    //Scans for VuForia Target. returns a string of either "none", "right", "left", or "center"
    //power sets scanning speed, distance sets range of "scan", relicTemplate is the VuforiaTrackable we are looking for.
    public String scanForVuMark(double power, int distance, VuforiaTrackable relicTemp){
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        setRunMode("RUN_TO_POSITION");
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemp);
        String decidingMark = "none"; //Used to return String
        //First movement.
        robot.turretMotor.setTargetPosition(distance); //Turns to distance.
        robot.turretMotor.setPower(power);
        while (robot.turretMotor.isBusy()){
            //Waiting for turret to stop moving.
            if(vuMark == RelicRecoveryVuMark.LEFT){//Left seen.
                decidingMark = "left";
                robot.turretMotor.setPower(0.0);
            }else if(vuMark == RelicRecoveryVuMark.CENTER){ //Center seen.
                decidingMark = "center";
                robot.turretMotor.setPower(0.0);
            }else if(vuMark == RelicRecoveryVuMark.RIGHT){ //Right seen.
                decidingMark = "right";
                robot.turretMotor.setPower(0.0);
            }else{ //No VuMark seen.
            }
        }
        //Second movement.
        robot.turretMotor.setTargetPosition(-distance); //Turns to distance.
        robot.turretMotor.setPower(power);
        while (robot.turretMotor.isBusy()){
            //Waiting for turret to stop moving.
            if(vuMark == RelicRecoveryVuMark.LEFT){//Left seen.
                decidingMark = "left";
                robot.turretMotor.setPower(0.0);
            }else if(vuMark == RelicRecoveryVuMark.CENTER){ //Center seen.
                decidingMark = "center";
                robot.turretMotor.setPower(0.0);
            }else if(vuMark == RelicRecoveryVuMark.RIGHT){ //Right seen.
                decidingMark = "right";
                robot.turretMotor.setPower(0.0);
            }else{ //No VuMark seen.
            }
        }
        //Third movement.
        robot.turretMotor.setTargetPosition(0); //Re-centers the turret.
        robot.turretMotor.setPower(power);

        return (decidingMark);
    }

    //Takes power and distance to rotate and "CW" clockwise or "CCW" as directional input.
    public void turnDirection(double power, int distance, String direction) {
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        setRunMode("RUN_TO_POSITION");
        if (direction.equals("CCW")) {
            robot.left1.setTargetPosition(distance);
            robot.left2.setTargetPosition(distance);
            robot.right1.setTargetPosition(-distance);
            robot.right2.setTargetPosition(-distance);
        } else if (direction.equals("CW")) {
            robot.left1.setTargetPosition(-distance);
            robot.left2.setTargetPosition(-distance);
            robot.right1.setTargetPosition(distance);
            robot.right2.setTargetPosition(distance);
        }
        setMotorPower(power);
        //Waits while turning.
        while (anyMotorsBusy()) {
            //Spinning
            //Waiting while turning.
        }
        //Stop motors.
        setMotorPower(0.0);
        //Sets mode back to RUN_USING_ENCODER
        setRunMode("RUN_USING_ENCODER");
    }

    //Drives at given power and a given distance unless floorSensors interrupt it
    // by seeing the given color. ("red" or "blue").
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
        while (anyMotorsBusy()) {
            telemetry.addData("Floor Sensor (blue):", robot.floorSensor.blue());
            telemetry.addData("Floor Sensor (red):", robot.floorSensor.red());
            if (color.equals("red")) {
                if (robot.floorSensor.red() > 50) {//Level of Red required to stop.
                    setMotorPower(0.0);
                }
            }
            if (color.equals("blue")) {
                if (robot.floorSensor.blue() > 50) {//Level of Blue required to stop.
                    setMotorPower(0.0);
                }
            }
            telemetry.update();
        }
        //Stops driving by setting power to 0.0.
        setMotorPower(0.0);
        //Sets back to RUN_USING_ENCODER mode.
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
        while (anyMotorsBusy()) {
            //Spinning.
            //Waiting for robot to arrive at destination.
        }
        //Stops driving by setting power to 0.0.
        setMotorPower(0.0);
        //Sets back to RUN_USING_ENCODER mode.
        setRunMode("RUN_USING_ENCODER");
    }

    //Sets the run mode of all DC motors. Also sets the turret motor now.
    public void setRunMode(String input) {
        if (input.equals("STOP_AND_RESET_ENCODER")) {
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if (input.equals("RUN_WITHOUT_ENCODER")) {
            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (input.equals("RUN_USING_ENCODER")) {
            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (input.equals("RUN_TO_POSITION")) {
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

    }

    //Sets all motors target position.
    public void setAllTargetPositions(int distance) {
        robot.left1.setTargetPosition(distance);
        robot.left2.setTargetPosition(distance);
        robot.right1.setTargetPosition(distance);
        robot.right2.setTargetPosition(distance);
    }

    //Sets all drive motor power.
    public void setMotorPower(double power) {
        robot.left1.setPower(power);
        robot.left2.setPower(power);
        robot.right1.setPower(power);
        robot.right2.setPower(power);
    }
    //Returns TRUE if any drive motors are busy and FALSE if not.
    public boolean anyMotorsBusy() {
        if (robot.left1.isBusy() || robot.left2.isBusy() || robot.right1.isBusy() || robot.right2.isBusy()) {
            return (true);
        } else {
            return (false);
        }
    }

    public void servoSpeed(double startingPosition, double finalPosition, int timeLimitMiliseconds, int numberOfSteps, Servo targetServo) {
        ElapsedTime timer = new ElapsedTime();
        double distance = 0;
        int stepTime = 0;
        int stepNumber = 0;
        double changeNumber = startingPosition;
        double currentPosition = 1;
        distance = Math.abs(startingPosition - finalPosition) / numberOfSteps;
        stepTime = (timeLimitMiliseconds / numberOfSteps);
        timer.reset();
        while (timer.milliseconds() < (timeLimitMiliseconds * 1000) && stepNumber < numberOfSteps) {
            currentPosition = changeNumber - distance;
            targetServo.setPosition(currentPosition);
            sleep(20);
            changeNumber = changeNumber - distance;
            stepNumber++;
        }
    }
}

