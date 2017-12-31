package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "State Teleop", group = "Teleop")
//@Disabled

//-----Teleop Controls ------

//Gamepad 1
//Left Stick Y- Left Side Drive
//Right Stick Y- Right Side Drive

//Gamepad2

public class GrantsNewTeleop extends LinearOpMode{
    public GrantsTeleopHardware robot = new GrantsTeleopHardware();

    //These are constants that are used to change the position of a servo or switch modes
    public static int clawmode = 0;
    public static int slidermode = 0;
    public static int shoulderPosition = 0;
    public static int endGameMode = 0;
    public static int sliderTwsitMode = 0;

    public void runOpMode(){

        //Allows us to switch modes to have more control
        if(gamepad1.start){
            endGameMode = 1;
        }else if (gamepad2.start){
            endGameMode = 0;
        }

        if(opModeIsActive() && endGameMode == 0){
            //Drive Motors
            robot.left1.setPower(speedAdjust(gamepad1.left_stick_y));
            robot.left2.setPower(speedAdjust(gamepad1.left_stick_y));
            robot.right1.setPower(speedAdjust(gamepad1.left_stick_y));
            robot.right2.setPower(speedAdjust(gamepad1.left_stick_y));

            //Claw Controls
            //WINCH CONTROLS
            if(gamepad2.dpad_up){
                robot.winch.setTargetPosition(robot.winch.getTargetPosition() + 15);
                robot.winch.setPower(0.35);
            }else if (gamepad2.dpad_down){
                robot.winch.setTargetPosition(robot.winch.getTargetPosition() - 15);  //TODO the motor may move too fast might have to decrease/increase the target additive
                robot.winch.setPower(0.35);
            }else{
                robot.winch.setTargetPosition(robot.winch.getTargetPosition());
                robot.winch.setPower(0.99);
            }
            //CLAW SERVO CONTROLS
            if(gamepad1.right_bumper && clawmode == 0){
                robot.clawTop.setPosition(robot.BLOCK_CLAW_CLOSED);
                robot.clawBottom.setPosition(robot.BLOCK_CLAW_CLOSED);
                clawmode++;
            }else if (gamepad1.right_bumper && clawmode == 1){   //TODO test to make sure constants are correct
                robot.clawTop.setPosition(robot.BLOCK_CLAW_OPEN);
                robot.clawBottom.setPosition(robot.BLOCK_CLAW_OPEN);
                clawmode--;
            }
            //SHOULDER CONTROLS
            if(gamepad1.dpad_left){
                shoulderPosition = shoulderPosition + 10;
                robot.shoulder.setTargetPosition(shoulderPosition);
                robot.shoulder.setPower(0.2);
            }else if(gamepad1.dpad_right){  //TODO the target additive may need to be increased
                shoulderPosition = shoulderPosition - 10; //TODO the power may need to be increased
                robot.shoulder.setTargetPosition(shoulderPosition);
                robot.shoulder.setPower(0.2);
            }else{
                robot.shoulder.setTargetPosition(shoulderPosition);
            }
        }
        if(endGameMode == 1){
            //Drive Motors
            if(gamepad2.left_stick_y < 0.05){
                robot.setMotorPower(0.5);
            } else if(gamepad2.left_stick_y > -0.05){
                robot.setMotorPower(-0.5);
            }else if (gamepad2.left_stick_x < 0.05){
                robot.left1.setPower(0.5);
                robot.left2.setPower(0.5);
                robot.right1.setPower(-0.5);  //TODO the direction of the turns may be reversed
                robot.right2.setPower(-0.5);  //TODO the power of the motors may need to be increased/decreased
            }else if (gamepad2.left_stick_x > -0.05){
                robot.left1.setPower(-0.5);
                robot.left2.setPower(-0.5);
                robot.right1.setPower(0.5);
                robot.right2.setPower(0.5);
            }

            //Claw Controls
            //WINCH CONTROLS
            if(gamepad2.dpad_up){
                robot.winch.setTargetPosition(robot.winch.getTargetPosition() + 15);
                robot.winch.setPower(0.35);
            }else if (gamepad2.dpad_down){ //TODO target additive may need to be changed
                robot.winch.setTargetPosition(robot.winch.getTargetPosition() - 15);
                robot.winch.setPower(0.35);
            }else{
                robot.winch.setTargetPosition(robot.winch.getTargetPosition());
                robot.winch.setPower(0.99);
            }
            //CLAW SERVO CONTROLS
            if(gamepad1.right_bumper && clawmode == 0){
                robot.clawTop.setPosition(robot.BLOCK_CLAW_CLOSED);
                robot.clawBottom.setPosition(robot.BLOCK_CLAW_CLOSED); //TODO test constants
                clawmode++;
            }else if (gamepad1.right_bumper && clawmode == 1){
                robot.clawTop.setPosition(robot.BLOCK_CLAW_OPEN);
                robot.clawBottom.setPosition(robot.BLOCK_CLAW_OPEN);
                clawmode--;
            }
            //SHOULDER CONTROLS
            if(gamepad1.dpad_left){
                shoulderPosition = shoulderPosition + 10;
                robot.shoulder.setTargetPosition(shoulderPosition);
                robot.shoulder.setPower(0.2); //TODO might need to change target additive or power
            }else if (gamepad1.dpad_right){
                shoulderPosition = shoulderPosition - 10;
                robot.shoulder.setTargetPosition(shoulderPosition);
                robot.shoulder.setPower(0.2);
            }else{
                robot.shoulder.setTargetPosition(shoulderPosition);
            }

            //Slider controls
            //Slider Motor Controls
            robot.slider.setPower(speedAdjust(gamepad2.right_stick_y));
            //Slider Grabbing Controls
            if(gamepad2.left_bumper && slidermode == 0){
                robot.relicClaw.setPosition(robot.RELIC_CLAW_CLOSED);
                slidermode++;
            }else if (gamepad2.left_bumper && slidermode == 1){
                robot.relicClaw.setPosition(robot.RELIC_CLAW_MIDDLE);
                slidermode++;
            }else if(gamepad2.left_bumper && slidermode == 2){
                robot.relicClaw.setPosition(robot.RELIC_CLAW_OPENED);
                slidermode = 0;
            }
            //Slider Twisting Controls
            if(gamepad2.y && sliderTwsitMode == 0){
                robot.relicTwist.setPosition(robot.RELIC_TWIST_DOWN);
                slidermode = 1;
            }
            if (gamepad2.y && sliderTwsitMode == 1){
                robot.relicTwist.setPosition(robot.RELIC_TWIST_UP);
                sliderTwsitMode = 0;
            }
        }
    }
    public static int controllerToPosition(float stickValue){
        float returnValue = 0;
        if(stickValue > 0){
            returnValue = stickValue * 2;
        }
        if(stickValue < 0){
            returnValue = stickValue * -2;
        }
        return((int)returnValue);
    }
    //Used to smooth out acceleration of robot.
    public static double speedAdjust(double stickInput){
        double returnValue;
        if(stickInput > 0){
            returnValue = (stickInput * stickInput);
        }else if(stickInput < 0){
            returnValue = (-1 * (stickInput * stickInput));
        }else{
            returnValue = (stickInput);
        }
        return(returnValue);
    }
}