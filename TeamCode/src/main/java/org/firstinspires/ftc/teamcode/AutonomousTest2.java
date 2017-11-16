//AutonomousTest2.java

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "AutonomousTest2", group = "Autonomous")
//@Disabled

//TO DO LIST:
//Test scanForVuMark()
//Test followLine()
//See if we can collect IMU data in TeleOpTest
//See if we can turn on LEDs for sensors and set initial arm positions in RRHardwarePresets
//Work with new Position class
//Update FIRST SDK
//Use difference in IMU readings to tell where our turret is pointed.

public class AutonomousTest2 extends LinearOpMode{
    RRHardwarePresets robot = new RRHardwarePresets();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        robot.jewelArm.setPosition(0.7);
        //Vuforia Trackables.
        VuforiaTrackables relicTrackables = robot.vuforia.loadTrackablesFromAsset("RelicVuMark"); //I believe this loads VuMark data from the assets folder in FtcRobotController.
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.setRunMode("RUN_USING_ENCODER");

        waitForStart();

        //Relic Trackables
        relicTrackables.activate();

        //CHANGE THIS BOOLEAN TO RUN TEST AREA. PUT IN SO WE DON'T HAVE TO RUN ENTIRE SCRIPT TO TEST.
        boolean testArea = true;

        if (testArea == true) {
            //--TEST SCRIPT START--
            //robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_DOWN_COMPLETE, 1000, 5000);
            //robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_UP, 1000, 5000);//Moves jewelArm to JEWEL_ARM_DOWN_COMPLETE over a time period of 5 seconds with 10000 individual movements.
            //robot.moveMultipleServo(robot.wrist, robot.elbow, robot.WRIST_UNFOLDED, robot.ELBOW_UNFOLDED, 5000, 5000);
            String testString = scanForVuMark(0.05, 300, relicTemplate);
            if(testString.equals("left")){
                robot.turnDirection(0.15, 1000, "CCW");
            }
            if(testString.equals("right")){
                robot.turnDirection(0.15, 1000, "CW");
            }
            if(testString.equals("center")){
                robot.driveForwardSetDistance(0.15, 1000);
            }
            if(testString.equals("none")){
                //:(
            }
        }else{
            //set testArea to true to only run code in the test area. Allows us to test individual components without running entire autonomous script.

            //--AUTO SCRIPT START--

            //Lowers jewel arm into JEWEL_ARM_DOWN position with 1000 steps over 2 seconds.
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

            //Raises jewel arm into JEWEL_ARM_UP position with 1000 steps over 2 seconds.
            robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_UP, 1000, 2000);
            sleep(500);

            //Drive forward 1200 units at a speed of 0.15 off of the balance stone.
            robot.driveForwardSetDistance(0.15, 1200);
            sleep(500);

            //Search for and confirm VuMark.
            String targetPosition = scanForVuMark(0.15, 500, relicTemplate);

            //Turn dependent on what we read from vuMark.
            if(targetPosition.equals("left")){ //Turn CCW, then drive forward.
                robot.turnDirection(0.15, 500, "CCW");
                robot.driveForwardSetDistance(0.15, 100);
            }
            if(targetPosition.equals("right")){ //Turn CW, then drive forward.
                robot.turnDirection(0.15, 500, "CW");
                robot.driveForwardSetDistance(0.15, 100);
            }
            if(targetPosition.equals("center")){ //Just drive forward.
                robot.driveForwardSetDistance(0.15, 100);
            }
            if(targetPosition.equals("none")){
                //Didn't find vuMark :(
            }

            //Drives forward and stops on line.

            //Turn towards triangle.

            //Continue...

        }
    }

    //---AUTONOMOUS ONLY METHODS BELOW---

    //Moves the jewelArm depending on what the input is.
    public void knockOffBall(int selection) {
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.setRunMode("RUN_TO_POSITION");
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
        robot.setRunMode("RUN_USING_ENCODER");
    }

    //NEEDS TO BE TESTED.
    //followValue should be the average of the 2 sensor.argb() color values. lineColor can either be "red" or "blue".
    public void followLine(String lineColor, double speed) {
        //Sets mode to RUN_USING_ENCODER
        robot.setRunMode("RUN_USING_ENCODER");
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
        robot.driveForwardWithInterrupt(0.15, 300, lineColor);
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

    //NEEDS TO BE TESTED.
    //Scans for VuForia Target. returns a string of either "none", "right", "left", or "center"
    //power sets scanning speed, distance sets range of "scan", relicTemplate is the VuforiaTrackable we are looking for.
    public String scanForVuMark(double power, int distance, VuforiaTrackable relicTemp){
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.setRunMode("RUN_TO_POSITION");
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
}

