//AutonomousTest2.java

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "AutonomousTest2", group = "Autonomous")
//@Disabled

public class AutonomousTest2 extends LinearOpMode{
    RRHardwarePresets robot = new RRHardwarePresets();
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        //Vuforia Trackables.
        VuforiaTrackables relicTrackables = robot.vuforia.loadTrackablesFromAsset("RelicVuMark"); //I believe this loads VuMark data from the assets folder in FtcRobotController.
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        setRunMode("STOP_AND_RESET_ENCODER");
        setRunMode("RUN_USING_ENCODER");

        waitForStart();

        boolean testArea = false;
        if(testArea == true){
            //--TEST SCRIPT START--
            //Should tell us when we see an identified VuMark, not using 3d pose navigation yet.
            while(opModeIsActive()){
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if(vuMark != RelicRecoveryVuMark.UNKNOWN){ //If we identify a vuMark.
                    telemetry.addData("VuMark", "%s visible", vuMark);
                }
                else{
                    telemetry.addData("VuMark", "not visible");
                }
                telemetry.update();
            }
        }
        else{ //set testArea to true to only run code in the test area. Allows us to test individual components without running entire autonomous script.

            //--AUTO SCRIPT START--
            //Set jewelArm into up position. Should put this into RRHardwarePresets.init().
            robot.jewelArm.setPosition(robot.JEWEL_ARM_UP);
            sleep(500);

            //Lower jewelArm into down position.
            robot.jewelArm.setPosition(robot.JEWEL_ARM_DOWN);
            sleep(500);

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
            robot.jewelArm.setPosition(robot.JEWEL_ARM_UP);
            sleep(500);

            //Read glyph on wall.

            //See testArea.

            //Drive forward off of balance stone.
            driveForwardSetDistance(0.15, 650);
            sleep(500);

            driveForwardWithInterrupt(0.10, 750, "red");
            sleep(500);

            //Turn towards triangle.

            //Continue...
        }
    }

    //Moves the jewelArm depending on what the input is.
    public void knockOffBall(int selection){
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        setRunMode("RUN_TO_POSITION");
        if(selection == 0){
            robot.turretMotor.setTargetPosition(200);
        }
        if(selection == 1){
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
    public void followLine(String lineColor, double speed){
        //Sets mode to RUN_USING_ENCODER
        setRunMode("RUN_USING_ENCODER");
        int loopFlag = 0;
        double correction;
        double leftPower;
        double rightPower;
        double followValue = 0;
        //Decides follow value. Try with sensor.argb() values.
        if(lineColor.equals("red")){
            followValue = ((robot.RED_LINE_COLOR + robot.FLOOR_COLOR)/2);
        }
        if(lineColor.equals("blue")){
            followValue = ((robot.BLUE_LINE_COLOR + robot.FLOOR_COLOR)/2);
        }
        //Drives forward until it hits a line.
        driveForwardWithInterrupt(0.15, 300, lineColor);
        //Corrects power on left and right dcMotors to follow a line.
        while(loopFlag == 0){
            //Get a correction
            correction = (followValue - robot.floorSensor.argb());
            if(correction <= 0.0){
                leftPower = speed - correction;
                rightPower = speed;
            }else{ //correction > 0.0
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

    //Takes power and distance to rotate and "CW" clockwise or "CCW" as directional input.
    public void turnDirection(double power, int distance, String direction){
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        setRunMode("RUN_TO_POSITION");
        if(direction.equals("CCW")){
            robot.left1.setTargetPosition(distance);
            robot.left2.setTargetPosition(distance);
            robot.right1.setTargetPosition( -distance);
            robot.right2.setTargetPosition(-distance);
        }
        else if(direction.equals("CW")){
            robot.left1.setTargetPosition(-distance);
            robot.left2.setTargetPosition(-distance);
            robot.right1.setTargetPosition(distance);
            robot.right2.setTargetPosition(distance);
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

    //Drives at given power and a given distance unless floorSensors interupt it
    // by seeing the given color. ("red" or "blue").
    public void driveForwardWithInterrupt(double power, int distance, String color){
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
            telemetry.addData("Floor Sensor (blue):", robot.floorSensor.blue());
            telemetry.addData("Floor Sensor (red):", robot.floorSensor.red());
            if(color.equals("red")){
                if(robot.floorSensor.red() > 50){//Level of Red required to stop.
                    setMotorPower(0.0);
                }
            }
            if(color.equals("blue")){
                if(robot.floorSensor.blue() > 50){//Level of Blue required to stop.
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
    public void driveForwardSetDistance(double power, int distance){
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

    //Sets the run mode of all DC motors. Also sets the turret motor now.
    public void setRunMode(String input){
        if(input.equals("STOP_AND_RESET_ENCODER")) {
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if(input.equals("RUN_WITHOUT_ENCODER")) {
            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(input.equals("RUN_USING_ENCODER")) {
            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(input.equals("RUN_TO_POSITION")) {
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    //Sets all motors target position.
    public void setAllTargetPositions(int distance){
        robot.left1.setTargetPosition(distance);
        robot.left2.setTargetPosition(distance);
        robot.right1.setTargetPosition(distance);
        robot.right2.setTargetPosition(distance);
    }

    //Sets all motors power.
    public void setMotorPower(double power){
        robot.left1.setPower(power);
        robot.left2.setPower(power);
        robot.right1.setPower(power);
        robot.right2.setPower(power);
    }

    //Returns TRUE if any drive motors are busy and FALSE if not.
    public boolean anyMotorsBusy(){
        if(robot.left1.isBusy() || robot.left2.isBusy() || robot.right1.isBusy() || robot.right2.isBusy()){
            return(true);
        }else{
            return(false);
        }
    }
}
