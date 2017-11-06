//AutonomousTest2.java

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutonomousTest2", group = "Autonomous")
//@Disabled

public class AutonomousTest2 extends LinearOpMode{
    RRHardwarePresets robot = new RRHardwarePresets();
    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        setRunMode("STOP_AND_RESET_ENCODER");
        setRunMode("RUN_USING_ENCODER");
        waitForStart();

        //--AUTO SCRIPT START--

        //Set jewelArm into up position. Should put this into RRHardwarePresets.init().
        robot.jewelArm.setPosition(robot.JEWEL_ARM_UP);
        sleep(500);

        //Lower jewelArm into down position.
        robot.jewelArm.setPosition(robot.JEWEL_ARM_DOWN);
        sleep(500);

        int loopBreak = 0;
        while(loopBreak == 0){
            //Read color of Jewel.
            sleep(1000);
            if(robot.jewelSensor.red() > 52){
                telemetry.addData("Status", "Red Ball Seen!");
                telemetry.addData("Red", robot.jewelSensor.red());
                telemetry.update();
                knockOffBall(1);
                loopBreak = 1;
            }else if(robot.jewelSensor.red() <= 52) {
                telemetry.addData("Status", "Red Ball Not Seen!");
                telemetry.addData("Red", robot.jewelSensor.red());
                telemetry.update();

                if(robot.jewelSensor.blue() > 20) {
                    telemetry.addData("Status", "Blue Ball Seen!");
                    telemetry.addData("Blue", robot.jewelSensor.blue());
                    telemetry.update();
                    knockOffBall(0);
                    loopBreak = 1;
                }else{
                    loopBreak = 1;
                }
            }
        }

        //Raise JewelArm.
        sleep(500);
        robot.jewelArm.setPosition(robot.JEWEL_ARM_UP);
        sleep(500);

        //Read glyph on wall.

        //Drive forward off of balance stone.
        driveForwardSetDistance(0.15, 650);
        sleep(500);

        driveForwardWithInterrupt(0.10, 750, "red");
        sleep(500);

        //Continue...
    }

    public int readJewel(){
        if(robot.jewelSensor.red() > 52){
            //Red Ball
            return(0);
        }else if(robot.jewelSensor.red() <= 52) {
            //Blue Ball
            return(1);
        }else{
            //Neither
            return(-1);
        }
    }

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
                if(robot.floorSensor.red() > 50){
                    setMotorPower(0.0);
                }
            }
            if(color.equals("blue")){
                if(robot.floorSensor.blue() > 50){
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

    //Sets the run mode of all DC motors.
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
