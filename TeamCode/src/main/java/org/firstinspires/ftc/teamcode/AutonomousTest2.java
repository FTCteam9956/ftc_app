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

        //Lower jewelArm
        robot.jewelArm.setPosition(1.0); //Lowered arm.

        //Read jewelSensor

        //Turn turret to knock off appropriate jewel, and then turn back.

        //Raise jewelArm
        robot.jewelArm.setPosition(0.28); //Raised arm.

        //Read glyph on wall.

        //Drive forward off of balance stone.
        driveForwardDistance(0.30, 1000);

        //Continue...

    }


    //Drives forward a certain distance at a certain speed. Only use if no intention to interrupt.
    public void driveForwardDistance(double power, int distance){
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        setRunMode("STOP_AND_RESET_ENCODER");
        //Sets target distance.
        robot.left1.setTargetPosition(distance);
        robot.left2.setTargetPosition(distance);
        robot.right1.setTargetPosition(distance);
        robot.right2.setTargetPosition(distance);
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
        }
        if(input.equals("RUN_WITHOUT_ENCODER")) {
            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(input.equals("RUN_USING_ENCODER")) {
            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(input.equals("RUN_TO_POSITION")) {
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
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
