//TeleOpTest.java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOpTest", group = "Teleop")
//@Disabled

//---TeleOp Controls---

//Gamepad 1
//Left Stick Y- Left Side Drive
//Right Stick Y- Right Side Drive
//Left Trigger - Rotate turret left
//Right Trigger - Rotate turret right
//DPad Down - Toggle JewelArm position

//Gamepad 2
//Right Stick X - Move Arm at Elbow Servo
//Left Stick X - Move Arm at Wrist Servo
//DPad Left + Right - Move Arm at Shoulder
//DPad Up - Raise with winch
//DPad Down - Lower with winch
//A Button - Fully Retract Arm
//Y Button - Fully Extend Arm
//Right Bumper - Toggle Claw Position
//Left Bumper - Toggle Claw Rotation

public class TeleOpTest extends LinearOpMode{

    //Instantiation of RRHardwarePresets
    RRHardwarePresets robot = new RRHardwarePresets();

    //Variables used for toggles
    public static int clawMode = 0;
    public static int clawTwistMode = 0;
    public static int shoulderPosition = 0;
    public static int wristMode = 0;
    public static int wristAngle = 90;
    public static int wristVar = 0;

    @Override
    public void runOpMode() {
        //Initialize Motors, Sensors, Servos, Constants, and some commonly used functions
        robot.init(hardwareMap);

        //Set Motor modes
        robot.setRunMode("STOP_AND_RESET_ENCODER"); //Resets all encoders.
        robot.setRunMode("RUN_USING_ENCODER");
        robot.winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Changes shoulder motor to RUN_TO_POSITION mode.

        waitForStart();
        //Set winch and servo starting positions
        robot.winchMotor.setTargetPosition(400);
        robot.winchMotor.setPower(0.3);
        sleep(500);
        robot.initServoPositions();

        while (opModeIsActive()){

            //---GAMEPAD 1---

            //Set jewel arm up
            robot.jewelArm.setPosition(0);

            //TANK DRIVE Move the drive wheels with the gamepad #1 sticks
            robot.left1.setPower(speedAdjust(gamepad1.left_stick_y /2));
            robot.left2.setPower(speedAdjust(gamepad1.left_stick_y /2));
            robot.right1.setPower(speedAdjust(gamepad1.right_stick_y /2));
            robot.right2.setPower(speedAdjust(gamepad1.right_stick_y /2));

            //---GAMEPAD 2---

            //WINCH Move it up and down with the d-pad
            if (gamepad2.dpad_up) {
                robot.winchMotor.setTargetPosition(robot.winchMotor.getTargetPosition() + 15);
                robot.winchMotor.setPower(0.35);
            } else if (gamepad2.dpad_down) {
                robot.winchMotor.setTargetPosition(robot.winchMotor.getTargetPosition() - 15);
                robot.winchMotor.setPower(0.35);
            } else {
                robot.winchMotor.setTargetPosition(robot.winchMotor.getTargetPosition());
                robot.winchMotor.setPower(0.99);
            }

            //TURRET Spin it with the left stick x
            if (gamepad2.left_stick_x < 0.05) {
                robot.turretMotor.setPower(gamepad2.left_stick_x * -0.25);
            } else if (gamepad2.left_stick_x > -0.05){
                robot.turretMotor.setPower(gamepad2.left_stick_x * -0.25);
            } else {
                robot.turretMotor.setPower(0.0);
            }

            //ARM POSITION PRESETS
            if(gamepad2.y){//Move arm all the way out if the y button is pressed
                shoulderPosition = 420;
            }
            if(gamepad2.a){//Move arm all the way in
                shoulderPosition = 0;
            }

            //Set the mode of the arm
            if(gamepad2.right_trigger > 0.5){
                if (wristMode == 0){
                    wristMode = 1;
                    sleep(200);
                }else if (wristMode == 1){
                    wristMode = 0;
                    wristVar = 90;
                    sleep(200);
                } else {
                }
            }
            if(wristMode == 0) {
                wristAngle = wristVar;
                if (gamepad2.x){
                    wristVar += 2;
                }if (gamepad2.b){
                    wristVar += -2;
                }
            }
            if(wristMode == 1){
                wristAngle = 90;

            }if(shoulderPosition < 0){
                shoulderPosition = 0;
            }if (wristVar < 0){
                wristVar = 0;
            }if (wristAngle < 0){
                wristAngle = 0;
            }if (gamepad2.right_stick_y < 0.05){//Set shoulder position of arm with the gamepad right stick
                shoulderPosition = shoulderPosition + controllerToPosition(gamepad2.right_stick_y);
                robot.shoulder.setTargetPosition(shoulderPosition);
                robot.shoulder.setPower(0.2);
                robot.elbow.setPosition(1 - ((robot.shoulder.getCurrentPosition() / 4.6) * 0.00388) * 2);
                robot.wrist.setPosition(((robot.shoulder.getCurrentPosition() / 4.6) * 0.00388) + (wristAngle * 0.00388));
            } else if (gamepad2.right_stick_y > 0.05) {
                shoulderPosition = shoulderPosition - controllerToPosition(gamepad2.right_stick_y);
                robot.shoulder.setTargetPosition(shoulderPosition);
                robot.shoulder.setPower(0.2);
                robot.elbow.setPosition(1 - ((robot.shoulder.getCurrentPosition() / 4.6) * 0.00388) * 2);
                robot.wrist.setPosition(((robot.shoulder.getCurrentPosition() / 4.6) * 0.00388) + (wristAngle * 0.00388));
            } else {
                robot.shoulder.setTargetPosition(shoulderPosition);
                robot.elbow.setPosition(1 - ((robot.shoulder.getCurrentPosition() / 4.6) * 0.00388) * 2);
                robot.wrist.setPosition((robot.shoulder.getCurrentPosition() / 4.6) * 0.00388);
            }

            //ELBOW //Move the elbow on it's own with the right stick x in mode 1
            if (Math.abs(gamepad2.right_stick_x) >= 0.05) {
                robot.elbow.setPosition(robot.elbow.getPosition() + (-gamepad2.right_stick_x * 0.01));
            } else {
                robot.elbow.setPosition(robot.elbow.getPosition());
            }

            //CLAW Open and close claw with the left bumper
            if (gamepad2.left_bumper && clawMode == 0) {
                robot.claw.setPosition(robot.CLAW_OPENED);
                this.clawMode = 1;
                sleep(300);
            }
            if (gamepad2.left_bumper && clawMode == 1) {
                robot.claw.setPosition(robot.CLAW_MID);
                this.clawMode = 2;
                sleep(300);
            }
            if (gamepad2.left_bumper && clawMode == 2) {
                robot.claw.setPosition(robot.CLAW_CLOSED);
                this.clawMode = 3;
                sleep(300);
            }
            if (gamepad2.left_bumper && clawMode == 3) {
                robot.claw.setPosition(robot.CLAW_MID);
                this.clawMode = 0;
                sleep(300);
            }

            //CLAW TWIST Move the relic to a 90 degree turn with the right bumper
            if (gamepad2.right_bumper && clawTwistMode == 0) {
                robot.clawTwist.setPosition(robot.TWIST_DOWN);
                this.clawTwistMode = 1;
                sleep(500);
            }
            if (gamepad2.right_bumper && clawTwistMode == 1) {
                robot.clawTwist.setPosition(robot.TWIST_UP);
                this.clawTwistMode = 0;
                sleep(500);
            }
//            if(gamepad2.dpad_left){
//                moveToRelic(0.59, 0.55, 72, 8);
//            }

            //---TELEMETRY--- shows up on the phone to tell us important things
            telemetry.addData("TurretMotor", robot.turretMotor.getCurrentPosition());
            telemetry.addData("ShoulderPosition", shoulderPosition);
            telemetry.addData("Shoulder Encoder", robot.shoulder.getCurrentPosition());
            telemetry.addData("Stick", gamepad2.right_stick_y);
            telemetry.addData("WristMode", wristMode);
            telemetry.update();
            idle();
        }//WhileOpModeIsActive() End.
    }

    //---TELEOP ONLY METHODS BELOW---

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
//    public void moveToRelic(double wristPos, double elbowPos, int shoulderPos, int turretPos){
//        robot.wrist.setPosition(wristPos);
//        sleep(500);
//        robot.elbow.setPosition(elbowPos);
//        sleep(500);
//        robot.shoulder.setTargetPosition(shoulderPos);
//        sleep(500);
//        robot.turretMotor.setTargetPosition(turretPos);
//        robot.turretMotor.setPower(0.3);
//    }
}
