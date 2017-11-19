//TeleOpTest.java
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpTest", group = "Teleop")
//@Disabled

//---TeleOp Controls---

//Gamepad 1
//Left Stick Y- Left Side Drive
//Right Stick Y- Right Side Drive
//Left Trigger - Rotate turret left
//Right Trigger - Rotate turret right
//DPad Up - Raise JewelArm
//DPad Down - Lower JewelArm

//Gamepad 2
//Left Stick Button OR DPad Down - Toggle Arm Control Mode
    //(Linear Mode)
    //Left Stick Y - Moves arm along linear projection of Arm.
    //(Free Control Mode)
    //Left Stick X - Move Arm at wrist Servo
    //Left Stick Y - Move Arm at elbow Servo
//A Button - Fully Retract Arm
//Y Button - Fully Extend Arm


public class TeleOpTest extends LinearOpMode{
    RRHardwarePresets robot = new RRHardwarePresets();

    public static int armMode = 0; // 0 = Sweeping Mode, 1 = Agile Mode

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();

        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.setRunMode("RUN_USING_ENCODER");

        while(opModeIsActive()){

            //---GAMEPAD 1---

            //Sets power for DC motors. (TANK DRIVE)
            robot.left1.setPower(speedAdjust(gamepad1.left_stick_y));
            robot.left2.setPower(speedAdjust(gamepad1.left_stick_y));
            robot.right1.setPower(speedAdjust(gamepad1.right_stick_y));
            robot.right2.setPower(speedAdjust(gamepad1.right_stick_y));
            robot.winch.setPower(speedAdjust(gamepad2.right_stick_y));
            //robot.shoulder.setPower(speedAdjust(gamepad2.right_stick_x));

            //DC motor turret controls.
            if(gamepad1.right_trigger > 0.5){
                robot.turretMotor.setPower(0.15);
            }
            else if(gamepad1.left_trigger > 0.5){
                robot.turretMotor.setPower(-0.15);
            }else{
                robot.turretMotor.setPower(0.0);
            }

            //Servo jewelArm controls.
            if(gamepad1.dpad_up){
                robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_UP, 1000, 3000);
            }
            if(gamepad1.dpad_down){
                robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_DOWN, 1000, 3000);
            }

            //---GAMEPAD 2---

            //Swaps armMode between (Linear Mode) and (Free Control Mode)
            if(gamepad2.left_stick_button || gamepad2.dpad_down){
                if (armMode == 0){
                    this.armMode = 1;
                    sleep (500);
                }
                else if(armMode == 1){
                    this.armMode = 0;
                    sleep (500);
                }
            }

<<<<<<< Updated upstream
            //Extends and retracts arm to set positions.
            if(gamepad2.y){
                robot.moveMultipleServo(robot.wrist, robot.elbow, robot.WRIST_UNFOLDED, robot.ELBOW_UNFOLDED, 700, 150);
            }
            if(gamepad2.a){
                robot.moveMultipleServo(robot.wrist, robot.elbow, robot.WRIST_FOLDED, robot.ELBOW_FOLDED, 700, 150);
            }

            //armMode behaviors.
            if(armMode == 0){
                if(gamepad2.left_stick_y != 0){
                    double elbowCurrentPosition = robot.elbow.getPosition(); //Gets current arm position.
                    double wristCurrentPosition = robot.wrist.getPosition();
                    robot.elbow.setPosition(elbowCurrentPosition + (-gamepad2.left_stick_y * .002));//Moves the arm.
                    robot.wrist.setPosition(wristCurrentPosition + (-gamepad2.left_stick_y * .002));//Moves the arm.
                    telemetry.addData("CurrentPosition1", elbowCurrentPosition);
                    telemetry.addData("CurrentPosition2", wristCurrentPosition);
                }else if(gamepad2.right_stick_y != 0) {
                    double currentPosition2 = robot.wrist.getPosition();
                    robot.wrist.setPosition(currentPosition2 + (-gamepad2.left_stick_y * .002));//Moves the arm.
                    telemetry.addData("position", currentPosition2);
                }else if(gamepad2.left_stick_y != 0 && gamepad2.right_stick_y != 0) {
                    double currentPosition2 = robot.wrist.getPosition();
                    robot.wrist.setPosition(currentPosition2);
                }
            }else if(armMode == 1){
                double elbowCurrentPosition = robot.elbow.getPosition();
                double wristCurrentPosition = robot.elbow.getPosition();
                if(Math.abs(gamepad2.right_stick_y) >= 0.05){
                    robot.elbow.setPosition(elbowCurrentPosition + (gamepad2.left_stick_y * 0.00125));
                }else{
                    robot.elbow.setPosition(elbowCurrentPosition);
                }
                if(Math.abs(gamepad2.right_stick_x) >= 0.05){
                    robot.wrist.setPosition(wristCurrentPosition + (gamepad2.left_stick_x * 0.00125));
                }else{
                    robot.wrist.setPosition(wristCurrentPosition);
                }
            }
=======
            //Extends and retracts arm.
//            if(gamepad2.y){
//                robot.moveMultipleServo(robot.wrist, robot.elbow, robot.WRIST_UNFOLDED, robot.ELBOW_UNFOLDED, 700, 150);
//            }
//            if(gamepad2.a){
//                robot.moveMultipleServo(robot.wrist, robot.elbow, robot.WRIST_FOLDED, robot.ELBOW_FOLDED, 700, 150);
//            }
//            //armMode behaviors.
//            if(armMode == 0){
//                if(gamepad2.left_stick_y != 0){
//                    double elbowCurrentPosition = robot.elbow.getPosition(); //Gets current arm position.
//                    double wristCurrentPosition = robot.wrist.getPosition();
//                    robot.elbow.setPosition(elbowCurrentPosition + (-gamepad2.left_stick_y * .002));//Moves the arm.
//                    robot.wrist.setPosition(wristCurrentPosition + (-gamepad2.left_stick_y * .002));//Moves the arm.
//                    //moveWithJoystick(robot.wrist, robot.elbow, gamepad2.left_stick_y, gamepad2.left_stick_y);
//                    telemetry.addData("CurrentPosition1", elbowCurrentPosition);
//                    telemetry.addData("CurrentPosition2", wristCurrentPosition);
//                }else if(gamepad2.right_stick_y != 0) {
//                    double currentPosition2 = robot.wrist.getPosition();
//                    robot.wrist.setPosition(currentPosition2 + (-gamepad2.left_stick_y * .002));//Moves the arm.
//                    //moveWithJoystick(robot.wrist, robot.elbow, gamepad2.left_stick_y, gamepad2.left_stick_y);
//                    telemetry.addData("position", currentPosition2);
//                }else if(gamepad2.left_stick_y != 0 && gamepad2.right_stick_y != 0) {
//                    double currentPosition2 = robot.wrist.getPosition();
//                    robot.wrist.setPosition(currentPosition2);
//                }
//            }else if(armMode == 1){
//                double elbowCurrentPosition = robot.elbow.getPosition();
//                double wristCurrentPosition = robot.elbow.getPosition();
//                if(Math.abs(gamepad2.right_stick_y) >= 0.05){
//                    robot.elbow.setPosition(elbowCurrentPosition + (gamepad2.left_stick_y * 0.00125));
//                }else{
//                    robot.elbow.setPosition(elbowCurrentPosition);
//                }
//                if(Math.abs(gamepad2.right_stick_x) >= 0.05){
//                    robot.wrist.setPosition(wristCurrentPosition + (gamepad2.left_stick_x * 0.00125));
//                }else{
//                    robot.wrist.setPosition(wristCurrentPosition);
//                }
//            }
>>>>>>> Stashed changes

            //---TELEMETRY---
            //Telemetry
            //telemetry.addData("left1 encoder", robot.left1.getCurrentPosition());
            //telemetry.addData("left2 encoder", robot.left2.getCurrentPosition());
            //telemetry.addData("right1 encoder", robot.right1.getCurrentPosition());
            //telemetry.addData("right2 encoder", robot.right2.getCurrentPosition());
            //telemetry.addData("Left1 Power", robot.left1.getPower());
            //telemetry.addData("Left2 Power", robot.left2.getPower());
            //telemetry.addData("Right1 Power", robot.right1.getPower());
            //telemetry.addData("Right2 Power", robot.right2.getPower());
            //telemetry.addData("Claw Position", robot.claw.getPosition());
            //telemetry.addData("JewelArm Position", robot.jewelArm.getPosition());
            //telemetry.addData("Left Stick", gamepad1.left_stick_y);
            //telemetry.addData("Right Stick", gamepad1.right_stick_y);
            //telemetry.addData("TurretMotor", robot.turretMotor.getCurrentPosition());
            //telemetry.addData("Floor Sensor", robot.floorSensor.argb());
            //telemetry.addData("Jewel Sensor", robot.jewelSensor.argb());
            //telemetry.addData("Elbow Position", robot.elbow.getPosition());
            //telemetry.addData("Wrist Position", robot.wrist.getPosition());
            telemetry.addData("Arm Mode", armMode);

            telemetry.update();
            idle();

        } //WhileOpModeIsActive() End.
    }

    //---TELEOP ONLY METHODS BELOW---

    //Used to smooth out acceleration of robot.
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