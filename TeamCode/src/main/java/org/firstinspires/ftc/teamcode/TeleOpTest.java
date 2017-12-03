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

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        robot.setRunMode("STOP_AND_RESET_ENCODER"); //Resets all encoders.
        robot.setRunMode("RUN_USING_ENCODER");
        robot.winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION); //Changes shoulder motor to RUN_TO_POSITION mode.
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        robot.initServoPositions(); //This is where our motors move into their initial positions.

        while (opModeIsActive()){

            //---GAMEPAD 1---

            //TANK DRIVE
            robot.left1.setPower(speedAdjust(gamepad1.left_stick_y /4));
            robot.left2.setPower(speedAdjust(gamepad1.left_stick_y /4));
            robot.right1.setPower(speedAdjust(gamepad1.right_stick_y /4));
            robot.right2.setPower(speedAdjust(gamepad1.right_stick_y /4));

            //---GAMEPAD 2---

            //WINCH
            if (gamepad2.dpad_up) {
                robot.winchMotor.setTargetPosition(robot.winchMotor.getTargetPosition() + 30);
                robot.winchMotor.setPower(0.35);
            } else if (gamepad2.dpad_down) {
                robot.winchMotor.setTargetPosition(robot.winchMotor.getTargetPosition() - 30);
                robot.winchMotor.setPower(0.35);
            } else {
                robot.winchMotor.setTargetPosition(robot.winchMotor.getTargetPosition());
                robot.winchMotor.setPower(0.99);
            }

            //TURRET
            if (gamepad2.left_stick_x > 0.05) {
                robot.turretMotor.setTargetPosition(robot.turretMotor.getTargetPosition() + 10);
                robot.turretMotor.setPower(0.4);
            } else if (gamepad2.left_stick_x < -0.05){
                robot.turretMotor.setTargetPosition(robot.turretMotor.getTargetPosition() - 10);
                robot.turretMotor.setPower(0.4);
            } else {
                robot.turretMotor.setTargetPosition(robot.turretMotor.getTargetPosition());
                robot.turretMotor.setPower(0.99);
            }


            //ARM POSITION PRESETS
            if(gamepad2.y){
                shoulderPosition = 420;
                //robot.moveMultipleServo(robot.wrist, robot.elbow, robot.WRIST_UNFOLDED, robot.ELBOW_UNFOLDED, 300, 70);
            }
            if(gamepad2.a){
                shoulderPosition = 0;
                //robot.moveMultipleServo(robot.wrist, robot.elbow, robot.WRIST_FOLDED, robot.ELBOW_FOLDED, 300, 70);
            }

            //ARM
            if (gamepad2.right_stick_y < 0.05) {
                shoulderPosition = shoulderPosition + controllerToPosition(gamepad2.right_stick_y);
                robot.shoulder.setTargetPosition(shoulderPosition);
                robot.shoulder.setPower(0.1);

                robot.elbow.setPosition(1 - ((robot.shoulder.getCurrentPosition() / 4.6) * 0.00388) * 2);
                robot.wrist.setPosition(((robot.shoulder.getCurrentPosition() / 4.6) * 0.00388) + (90 * 0.00388));

            }else if(gamepad2.right_stick_y > 0.05){
                shoulderPosition = shoulderPosition - controllerToPosition(gamepad2.right_stick_y);
                robot.shoulder.setTargetPosition(shoulderPosition);
                robot.shoulder.setPower(0.1);

                robot.elbow.setPosition(1 - ((robot.shoulder.getCurrentPosition() / 4.6) * 0.00388) * 2);
                robot.wrist.setPosition(((robot.shoulder.getCurrentPosition() / 4.6) * 0.00388) + (90 * 0.00388));

            } else {
                robot.shoulder.setTargetPosition(shoulderPosition);
                robot.elbow.setPosition(1 - ((robot.shoulder.getCurrentPosition() / 4.6) * 0.00388) * 2);
                robot.wrist.setPosition((robot.shoulder.getCurrentPosition() / 4.6) * 0.00388);
            }

            //ELBOW
            if (Math.abs(gamepad2.right_stick_x) >= 0.05) {
                robot.elbow.setPosition(robot.elbow.getPosition() + (-gamepad2.right_stick_x * 0.01));
            } else {
                robot.elbow.setPosition(robot.elbow.getPosition());
            }

            //WRIST
            if (Math.abs(gamepad2.left_stick_x) >= 0.05) {
                robot.wrist.setPosition(robot.wrist.getPosition() + (-gamepad2.left_stick_x * 0.01));
            } else {
                robot.wrist.setPosition(robot.wrist.getPosition());
            }

            //CLAW
            if (gamepad2.right_bumper && clawMode == 0) {
                robot.claw.setPosition(robot.CLAW_OPENED);
                this.clawMode = 1;
                sleep(500);
            }
            if (gamepad2.right_bumper && clawMode == 1) {
                robot.claw.setPosition(robot.CLAW_MID);
                this.clawMode = 2;
                sleep(500);
            }
            if (gamepad2.right_bumper && clawMode == 2) {
                robot.claw.setPosition(robot.CLAW_CLOSED);
                this.clawMode = 3;
                sleep(500);
            }
            if (gamepad2.right_bumper && clawMode == 3) {
                robot.claw.setPosition(robot.CLAW_MID);
                this.clawMode = 0;
                sleep(500);
            }

            //CLAW TWIST
            if (gamepad2.left_bumper && clawTwistMode == 0) {
                robot.clawTwist.setPosition(robot.TWIST_DOWN);
                this.clawTwistMode = 1;
                sleep(500);
            }
            if (gamepad2.left_bumper && clawTwistMode == 1) {
                robot.clawTwist.setPosition(robot.TWIST_UP);
                this.clawTwistMode = 0;
                sleep(500);
            }

            //---TELEMETRY---
            telemetry.addData("left1 encoder", robot.left1.getCurrentPosition());
            telemetry.addData("left2 encoder", robot.left2.getCurrentPosition());
            telemetry.addData("right1 encoder", robot.right1.getCurrentPosition());
            telemetry.addData("right2 encoder", robot.right2.getCurrentPosition());
            telemetry.addData("Left1 Power", robot.left1.getPower());
            telemetry.addData("Left2 Power", robot.left2.getPower());
            telemetry.addData("Right1 Power", robot.right1.getPower());
            telemetry.addData("Right2 Power", robot.right2.getPower());
            telemetry.addData("Claw Position", robot.claw.getPosition());
            telemetry.addData("JewelArm Position", robot.jewelArm.getPosition());
            telemetry.addData("Left Stick", gamepad1.left_stick_y);
            telemetry.addData("Right Stick", gamepad1.right_stick_y);
            telemetry.addData("TurretMotor", robot.turretMotor.getCurrentPosition());
            telemetry.addData("Floor Sensor", robot.floorSensor.argb());
            telemetry.addData("Jewel Sensor", robot.jewelSensor.argb());
            telemetry.addData("Elbow Position", robot.elbow.getPosition());
            telemetry.addData("Wrist Position", robot.wrist.getPosition());
            telemetry.addData("ShoulderPosition", robot.shoulder.getCurrentPosition());
            telemetry.addData("ShoulderPower", robot.shoulder.getPower());
            telemetry.addData("ShoulderBehavior", robot.shoulder.getZeroPowerBehavior());
            telemetry.addData("WinchShoulder", robot.winchMotor.getCurrentPosition());
            telemetry.addData("ShoulderPosition", shoulderPosition);
            telemetry.addData("Shoulder Encoder", robot.shoulder.getCurrentPosition());
            telemetry.addData("Stick", gamepad2.right_stick_y);
            telemetry.update();
            idle();

            }//WhileOpModeIsActive() End.
        }

    //---TELEOP ONLY METHODS BELOW---

    public static int controllerToPosition(float stickValue){
        float returnValue = 0;
        if(stickValue > 0){
            returnValue = stickValue * 8;
        }
        if(stickValue < 0){
            returnValue = stickValue * -8;
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
        return(returnValue/3);
    }
}
