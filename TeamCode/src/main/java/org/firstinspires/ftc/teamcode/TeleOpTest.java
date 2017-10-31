//TeleOpTest.java

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpTest", group = "Teleop")
//@Disabled
public class TeleOpTest extends LinearOpMode{
    RRHardwarePresets robot = new RRHardwarePresets(0);
    //Because we are giving 0 as a parameter the drive mode will be set to RUN_USING_ENCODERS

    //Declare Constants
    final double ARM_OUT = 1;
    final double ARM_IN = 0;
    final double SMACKER_UP = 0.25;
    final double SMACKER_DOWN = 1;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();

        while(opModeIsActive()){
            //Sets power for leading motors.
            robot.left1.setPower(speedAdjust(gamepad1.left_stick_y));
            robot.left2.setPower(speedAdjust(gamepad1.left_stick_y));
            robot.right1.setPower(speedAdjust(-gamepad1.right_stick_y));
            robot.right2.setPower(speedAdjust(-gamepad1.right_stick_y));

            //Sets position of by using the triggers
            if(gamepad1.right_trigger > 0.5){
                robot.claw.setPosition(ARM_OUT);
            }
            if(gamepad1.left_trigger > 0.5){
                robot.claw.setPosition(ARM_IN);
            }
            if(gamepad1.dpad_up){
                robot.smacker.setPosition(SMACKER_UP);
            }
            if(gamepad1.dpad_down){
                robot.smacker.setPosition(SMACKER_DOWN);
            }

            if(robot.joule.blue() < robot.joule.red()){
                telemetry.addData("Jewel Color", "Red");
            }
            else if(robot.joule.blue() > robot.joule.red()){
                telemetry.addData("Jewel Color", "Blue");
            }
            else{
                telemetry.addData("Jewel Color", "Neither");
            }

            if(robot.floor.blue() < robot.floor.red()){
                telemetry.addData("Floor Color", "Red");
            }
            else if(robot.joule.blue() > robot.joule.red()){
                telemetry.addData("Jewel Color", "Blue");
            }
            else{
                telemetry.addData(" Floor Color", "Neither");
            }

            //Telemetry
            telemetry.addData("ENCODER: (left1)", robot.left1.getCurrentPosition());
            telemetry.addData("ENCODER: (left2)", robot.left2.getCurrentPosition());
            telemetry.addData("ENCODER: (right1)", robot.right1.getCurrentPosition());
            telemetry.addData("ENCODER: (right2)", robot.right2.getCurrentPosition());
            telemetry.addData("Left1 Power", robot.left1.getPower());
            telemetry.addData("Left2 Power", robot.left2.getPower());
            telemetry.addData("Right1 Power", robot.right1.getPower());
            telemetry.addData("Right2 Power", robot.right2.getPower());
           // telemetry.addData("Claw Position", robot.claw.getPosition());
            telemetry.addData("Smacker Position", robot.smacker.getPosition());
            telemetry.addData("Left Stick", gamepad1.left_stick_y);
            telemetry.addData("Right Stick", gamepad1.right_stick_y);
            telemetry.update();

            idle();
        }
    }

    public static float speedAdjust(float stickInput){
        if(stickInput > 0){
            return(stickInput * stickInput);
        }else if(stickInput < 0){
            return(-1 * (stickInput * stickInput));
        }else{
            return(stickInput);
        }
    }
}