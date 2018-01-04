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
    public GrantsTeleopHardware robot =  new GrantsTeleopHardware();

    //These are constants that are used to change the position of a servo or switch modes
    public static int clawmode = 1;
    public static int slidermode = 0;
    public static int shoulderPos = 0;
    public static int endGameMode = 0;
    public static int sliderTwistMode = 0;

    public float rightPower;
    public float leftPower;

    public void runOpMode() {

        robot.init(hardwareMap);
        robot.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            //Allows us to switch modes to have more control
            if (gamepad1.start) {
                endGameMode = 1;
            } else if (gamepad2.start) {
                endGameMode = 0;
            }

            if (opModeIsActive() && endGameMode == 0) {

                //Drive Motor
                robot.left1.setPower(speedAdjust(gamepad1.left_stick_y /2));
                robot.left2.setPower(speedAdjust(gamepad1.left_stick_y /2));
                robot.right1.setPower(speedAdjust(gamepad1.right_stick_y /2));
                robot.right2.setPower(speedAdjust(gamepad1.right_stick_y /2));

                //Claw Controls
                //WINCH CONTROLS
                if(gamepad1.dpad_up){
                    robot.winch.setTargetPosition(robot.winch.getTargetPosition() + 15);
                    robot.winch.setPower(0.20);
                }else if(gamepad1.dpad_down){
                    robot.winch.setTargetPosition(robot.winch.getTargetPosition() - 15);
                    robot.winch.setPower(0.20);
                }else{
                    robot.winch.setTargetPosition(robot.winch.getTargetPosition());
                    robot.winch.setPower(0.20);
                }

                //CLAW SERVO CONTROLS
                if (gamepad1.right_bumper && clawmode == 0) {
                    robot.clawTop.setPosition(robot.BLOCK_CLAW_CLOSED_TOP);
                    robot.clawBottom.setPosition(robot.BLOCK_CLAW_CLOSED_BOTTOM);
                    sleep(1000);
                    clawmode++;
                } else if (gamepad1.right_bumper && clawmode == 1) {   //TODO test to make sure constants are correct
                    robot.clawTop.setPosition(robot.BLOCK_CLAW_OPEN_TOP);
                    robot.clawBottom.setPosition(robot.BLOCK_CLAW_OPEN_BOTTOM);
                    sleep(1000);
                    clawmode--;
                }
                //SHOULDER CONTROLS
                if (gamepad1.dpad_left) {
                    shoulderPos = shoulderPos + 1;
                    robot.shoulder.setTargetPosition(shoulderPos);
                    robot.shoulder.setPower(0.2);
                } else if (gamepad1.dpad_right) {
                    shoulderPos = shoulderPos - 1;
                    robot.shoulder.setTargetPosition(shoulderPos);
                    robot.shoulder.setPower(0.2);
                } else {
                    robot.shoulder.setTargetPosition(shoulderPos);
                }
                telemetry.addData("Jewel Sensor - Red", robot.jewelArm.red());
                telemetry.addData("Jewel Sensor - Blue", robot.jewelArm.blue());
                telemetry.update();
            }
            if (endGameMode == 1) {
                //Drive Motors
//            if(gamepad2.left_stick_y  < 0.05 && gamepad2.left_stick_y != 0){
//                robot.left1.setPower(speedAdjust(0.5));
//                robot.left2.setPower(speedAdjust(0.5));
//                robot.right1.setPower(speedAdjust(0.5));
//                robot.right2.setPower(speedAdjust(0.5));
//            } else if(gamepad2.left_stick_y > -0.05){
//                robot.left1.setPower(speedAdjust(-0.5));
//                robot.left2.setPower(speedAdjust(-0.5));
//                robot.right1.setPower(speedAdjust(-0.5));
//                robot.right2.setPower(speedAdjust(-0.5));
//            }else if (gamepad2.left_stick_x < 0.05){
//                robot.left1.setPower(speedAdjust(0.5));
//                robot.left2.setPower(speedAdjust(0.5));
//                robot.right1.setPower(speedAdjust(-0.5));
//                robot.right2.setPower(speedAdjust(-0.5));
//            }else if (gamepad2.left_stick_x > -0.05){
//                robot.left1.setPower(speedAdjust(-0.5));
//                robot.left2.setPower(speedAdjust(-0.5));
//                robot.right1.setPower(speedAdjust(0.5));
//                robot.right2.setPower(speedAdjust(0.5));
//            }
                leftPower = (gamepad2.left_stick_y + gamepad2.left_stick_x) /2;
                rightPower = (gamepad2.left_stick_y - gamepad2.left_stick_x) /2;

                robot.left1.setPower(leftPower);
                robot.left2.setPower(leftPower);
                robot.right1.setPower(rightPower);
                robot.right2.setPower(rightPower);

                //Claw Controls
                //WINCH CONTROLS
                if(gamepad1.dpad_up){
                    robot.winch.setTargetPosition(robot.winch.getTargetPosition() + 15);
                    robot.winch.setPower(0.20);
                }else if(gamepad1.dpad_down){
                    robot.winch.setTargetPosition(robot.winch.getTargetPosition() - 15);
                    robot.winch.setPower(0.20);
                }else{
                    robot.winch.setTargetPosition(robot.winch.getTargetPosition());
                    robot.winch.setPower(0.20);
                }

                //CLAW SERVO CONTROLS
                if (gamepad1.right_bumper && clawmode == 0) {
                    robot.clawTop.setPosition(robot.BLOCK_CLAW_CLOSED_TOP);
                    robot.clawBottom.setPosition(robot.BLOCK_CLAW_CLOSED_BOTTOM);
                    sleep(1000);
                    clawmode++;
                } else if (gamepad1.right_bumper && clawmode == 1) {   //TODO test to make sure constants are correct
                    robot.clawTop.setPosition(robot.BLOCK_CLAW_OPEN_TOP);
                    robot.clawBottom.setPosition(robot.BLOCK_CLAW_OPEN_BOTTOM);
                    sleep(1000);
                    clawmode--;
                }

                //SHOULDER CONTROLS
                if (gamepad1.dpad_left) {
                    shoulderPos = shoulderPos + 5;
                    robot.shoulder.setTargetPosition(shoulderPos);
                    robot.shoulder.setPower(0.2);
                } else if (gamepad1.dpad_right) {
                    shoulderPos = shoulderPos - 5;
                    robot.shoulder.setTargetPosition(shoulderPos);
                    robot.shoulder.setPower(0.2);
                } else {
                    robot.shoulder.setTargetPosition(shoulderPos);
                }

                //Slider controls
                //Slider Motor Controls
                robot.slider.setPower(speedAdjust(gamepad2.right_stick_y));
                //Slider Grabbing Controls
                if (gamepad2.left_bumper && slidermode == 0) {
                    robot.relicClaw.setPosition(robot.RELIC_CLAW_CLOSED);
                    slidermode++;
                } else if (gamepad2.left_bumper && slidermode == 1) {
                    robot.relicClaw.setPosition(robot.RELIC_CLAW_MIDDLE);
                    slidermode++;
                } else if (gamepad2.left_bumper && slidermode == 2) {
                    robot.relicClaw.setPosition(robot.RELIC_CLAW_OPENED);
                    slidermode = 0;
                }
                //Slider Twisting Controls
                if (gamepad2.y && sliderTwistMode == 0) {
                    robot.relicTwist.setPosition(robot.RELIC_TWIST_UP);
                    sliderTwistMode++;
                }
                if (gamepad2.y && sliderTwistMode == 1) {
                    robot.relicTwist.setPosition(robot.RELIC_TWIST_DOWN);
                    sliderTwistMode--;
                }
            }
            telemetry.addData("Winch Position", robot.winch.getTargetPosition());
            telemetry.addData("Winch Power", robot.winch.getPower());
            telemetry.addData("Endgame Mode", endGameMode);
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