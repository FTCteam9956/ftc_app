//AutonomousTest2.java

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.lang.reflect.Type;
import java.util.ArrayList;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import java.lang.InterruptedException;

@Autonomous(name = "RedStraight", group = "Autonomous")
//@Disabled

public class RedStraight extends LinearOpMode{
    public RRHardwarePresets robot = new RRHardwarePresets();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap); //Robot moves during init().

        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.setRunMode("RUN_USING_ENCODER");

        //Vuforia Trackables.
        VuforiaTrackables relicTrackables = robot.vuforia.loadTrackablesFromAsset("RelicVuMark"); //I believe this loads VuMark data from the assets folder in FtcRobotController.
        VuforiaTrackable relicTemplate = relicTrackables.get(1);

        waitForStart();

        //Relic Trackables
        relicTrackables.activate();

        boolean testArea = true; //CHANGE THIS BOOLEAN TO RUN TEST AREA. PUT IN SO WE DON'T HAVE TO RUN ENTIRE SCRIPT TO TEST.

        if(testArea == true){
            //--TEST SCRIPT START--

            //Testing with scanForVumark().
            //String testString = scanForVuMark(relicTemplate, 10000);

            //scanForVuMark(relicTemplate, 20000);
            robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_UNFOLDED, robot.WRIST_UNFOLDED, 1000, 2000);

        }else{
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
            String targetPosition = scanForVuMark(relicTemplate, 5000);

            robot.moveServo(robot.elbow, 0.75, 1000, 2000);
            robot.moveServo(robot.wrist, 0.5, 1000, 2000);

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

            //Drives forward and stops on line.

            //Turn towards triangle.

            //Continue...

        }
    }

    //---AUTONOMOUS ONLY METHODS BELOW---

    //Moves the jewelArm depending on what the input is.
    public void knockOffBall(int selection){
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
    public String scanForVuMark(VuforiaTrackable relicTemp, long timeOutInMilli){
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemp);
        String decidingMark = "none"; //Used to return String
        long time = timeOutInMilli * 1000000;
        double initTime = System.nanoTime();
        while(((System.nanoTime() - initTime) <= time) || !decidingMark.equals("none")){
            if(vuMark.equals(RelicRecoveryVuMark.LEFT)){//Left seen.
                decidingMark = "left";
            }else if(vuMark.equals(RelicRecoveryVuMark.CENTER)){ //Center seen.
                decidingMark = "center";
            }else if(vuMark.equals(RelicRecoveryVuMark.RIGHT)){ //Right seen.
                decidingMark = "right";
            }
            telemetry.addData("status", decidingMark);
            telemetry.update();
        }
        return(decidingMark);
    }
}

