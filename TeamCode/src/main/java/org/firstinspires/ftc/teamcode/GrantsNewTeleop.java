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
    public static int Rclawmode = 1;
    public static int Lclawmode = 1;
    public static int sliderClawmode = 0;
    public static int shoulderPos = 0;
    public static int endGameMode = 0;
    public static int sliderTwistMode = 0;

    public float rightPower;
    public float leftPower;

//    public int limit (int shoulderPos){
//        return Math.max(500, Math.min(shoulderPos ,0));
//    }
    public void runOpMode() {

        robot.init(hardwareMap);
        robot.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        robot.relicClaw.setPosition(robot.RELIC_CLAW_OPENED);
        //Set jewel arm teleop position
        robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_UP, 500, 1000);
        robot.rotateArm.setPosition(0.6);
//



        while (opModeIsActive()) {

            //Allows us to switch modes to have more control
            if (gamepad1.start) {
                endGameMode = 1;
            } else if (gamepad2.start) {
                endGameMode = 0;
            }

            if (opModeIsActive() && endGameMode == 0) {

                //Drive Motor
                robot.left1.setPower(speedAdjust(gamepad1.left_stick_y /1.5));
                robot.left2.setPower(speedAdjust(gamepad1.left_stick_y /1.5));
                robot.right1.setPower(speedAdjust(gamepad1.right_stick_y /1.5));
                robot.right2.setPower(speedAdjust(gamepad1.right_stick_y /1.5));

                //Claw Controls
                //WINCH CONTROLS
                if(gamepad1.dpad_up){
                    robot.winch.setTargetPosition(robot.winch.getTargetPosition() + 30);
                    robot.winch.setPower(0.40);
                }else if(gamepad1.dpad_down){
                    robot.winch.setTargetPosition(robot.winch.getTargetPosition() - 30);
                    robot.winch.setPower(0.40);
                }else{
                    robot.winch.setTargetPosition(robot.winch.getTargetPosition());
                    robot.winch.setPower(0.20);
                }

                //CLAW SERVO CONTROLS
                if(gamepad1.right_bumper){
                    robot.clawTop.setPosition(robot.BLOCK_CLAW_OPEN_TOP);
                }
                if(gamepad1.right_trigger > 0.5){
                    robot.clawBottom.setPosition(robot.BLOCK_CLAW_OPEN_BOTTOM);
                }
                if(gamepad1.left_bumper){
                    robot.clawTop.setPosition(robot.BLOCK_CLAW_CLOSED_TOP);
                }
                if(gamepad1.left_trigger > 0.5){
                    robot.clawBottom.setPosition(robot.BLOCK_CLAW_CLOSED_BOTTOM);
                }

                //SHOULDER CONTROLS
                if (gamepad1.dpad_left) {
                    shoulderPos = shoulderPos + 15;
                    robot.shoulder.setTargetPosition(shoulderPos);
                    robot.shoulder.setPower(0.2);
                } else if (gamepad1.dpad_right) {
                    shoulderPos = shoulderPos - 15;
                    robot.shoulder.setTargetPosition(shoulderPos);
                    robot.shoulder.setPower(0.2);
                } else {
                    robot.shoulder.setTargetPosition(shoulderPos);
                    robot.shoulder.setPower(0.5);
                }
                if(shoulderPos <= -581){
                    robot.shoulder.setTargetPosition(-580);
                    shoulderPos = -580;
                }
                telemetry.addData("Jewel Sensor - Red", robot.jewelArm.red());
                telemetry.addData("Jewel Sensor - Blue", robot.jewelArm.blue());
                telemetry.update();
            }
            if (endGameMode == 1) {
                //Drive Motors
                leftPower = (gamepad2.left_stick_y + gamepad2.left_stick_x);
                rightPower = (gamepad2.left_stick_y - gamepad2.left_stick_x);

                robot.left1.setPower(-leftPower / 3);
                robot.left2.setPower(-leftPower / 3);
                robot.right1.setPower(-rightPower / 3);
                robot.right2.setPower(-rightPower / 3);

                //Slider controls
                //Slider Motor Controls
                robot.slider.setPower(speedAdjust(gamepad2.right_stick_y));
                //Slider Grabbing Controls
                if(gamepad2.left_bumper && sliderClawmode == 0){
                    robot.relicClaw.setPosition(robot.RELIC_CLAW_CLOSED);
                    sleep(500);
                    sliderClawmode++;
                }else if(gamepad2.left_bumper && sliderClawmode == 1){
                    robot.relicClaw.setPosition(robot.RELIC_CLAW_OPENED);
                    sleep(500);
                    sliderClawmode--;
                }
                //Slider Twisting Controls
                if(gamepad2.right_bumper && sliderTwistMode == 0){
                    robot.relicTwist.setPosition(robot.RELIC_TWIST_UP);
                    sleep(500);
                    sliderTwistMode++;
                }else if(gamepad2.right_bumper && sliderTwistMode == 1) {
                    robot.relicTwist.setPosition(robot.RELIC_TWIST_DOWN);
                    sleep(500);
                    sliderTwistMode--;
                }
            }
            telemetry.addData("Winch Position", robot.winch.getTargetPosition());
            telemetry.addData("Winch Power", robot.winch.getPower());
            telemetry.addData("Endgame Mode", endGameMode);
            telemetry.addData("RELIC CLAW POS", robot.relicClaw.getPosition());
            telemetry.addData("SHOULDER POS", shoulderPos);
            telemetry.addData("Shoulder Encoder", robot.shoulder.getCurrentPosition());
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