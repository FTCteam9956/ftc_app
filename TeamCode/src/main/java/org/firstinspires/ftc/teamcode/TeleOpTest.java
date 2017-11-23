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
//Left Stick Button - Toggle Arm Control Mode

    //(Linear Mode)
    //Left Stick Y - Moves arm along linear projection of Arm.

    //(Free Control Mode)
    //Right Stick X - Move Arm at elbow Servo
    //Left Stick X - Move Arm at wrist Servo
    //DPad Left - Move Arm at shoulder
    //DPad Right - Move Arm at shoulder

//DPad Up - Raise with winch
//DPad Down - Lower with winch

//A Button - Fully Retract Arm
//Y Button - Fully Extend Arm


public class TeleOpTest extends LinearOpMode{
    RRHardwarePresets robot = new RRHardwarePresets();

    public static int armMode = 0; // 0 = Sweeping Mode, 1 = Agile Mode
    int clawMode = 0;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();

        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.setRunMode("RUN_USING_ENCODER");

        robot.jewelArm.setPosition(robot.JEWEL_ARM_UP); //Raised
        robot.wrist.setPosition(robot.WRIST_FOLDED); //Folded in
        robot.elbow.setPosition(robot.ELBOW_FOLDED);
        robot.shoulder.setTargetPosition(0);
        sleep(500);


        //Setting this to RUN_TO_POSITION to try and keep it in one place.
        robot.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int currentPos =robot.shoulder.getCurrentPosition();
        while(opModeIsActive()) {

            //---GAMEPAD 1---

            //Sets power for DC motors. (TANK DRIVE)
            robot.left1.setPower(speedAdjust(gamepad1.left_stick_y / 2));
            robot.left2.setPower(speedAdjust(gamepad1.left_stick_y / 2));
            robot.right1.setPower(speedAdjust(gamepad1.right_stick_y / 2));
            robot.right2.setPower(speedAdjust(gamepad1.right_stick_y / 2));

            //DC motor turret controls.
            if (gamepad1.right_trigger > 0.5) {
                robot.turretMotor.setPower(0.15);
            } else if (gamepad1.left_trigger > 0.5) {
                robot.turretMotor.setPower(-0.15);
            } else {
                robot.turretMotor.setPower(0.0);
            }

            //Servo jewelArm controls.
//            if(gamepad1.dpad_up){
//                robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_UP, 1000, 3000);
//            }
//            if(gamepad1.dpad_down){
//                robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_DOWN, 1000, 3000);
//            }


            //---GAMEPAD 2---

            //Controls winchMotor
            if (gamepad2.dpad_up) {
                robot.winchMotor.setPower(-0.15);
            } else if (gamepad2.dpad_down) {
                robot.winchMotor.setPower(0.15);
            } else {
                robot.winchMotor.setPower(0.0);
            }

            //Extends and retracts arm.
            if (gamepad2.y) {
                robot.moveMultipleServo(robot.wrist, robot.elbow, robot.WRIST_UNFOLDED, robot.ELBOW_UNFOLDED, 700, 150);
            }
            if (gamepad2.a) {
                robot.moveMultipleServo(robot.wrist, robot.elbow, robot.WRIST_FOLDED, robot.ELBOW_FOLDED, 700, 150);
            }

            //Swaps armMode between (Linear Mode) and (Free Control Mode)
            if (gamepad2.left_stick_button) {
                if (armMode == 0) {
                    this.armMode = 1;
                    sleep(500);
                } else if (armMode == 1) {
                    this.armMode = 0;
                    sleep(500);
                }
            }

            //armMode behaviors. aka switches between the modes that control the arm
            if (armMode == 0) {
                if (gamepad2.left_stick_y != 0) {
                    double elbowCurrentPosition = robot.elbow.getPosition(); //Gets current arm position.
                    double wristCurrentPosition = robot.wrist.getPosition();
                    robot.elbow.setPosition(elbowCurrentPosition + (-gamepad2.left_stick_y * .002));//Moves the arm.
                    robot.wrist.setPosition(wristCurrentPosition + (-gamepad2.left_stick_y * .002));//Moves the arm.
                    telemetry.addData("CurrentPosition1", elbowCurrentPosition);
                    telemetry.addData("CurrentPosition2", wristCurrentPosition);
                } else if (gamepad2.right_stick_y != 0) {
                    double currentPosition2 = robot.wrist.getPosition();
                    robot.wrist.setPosition(currentPosition2 + (-gamepad2.left_stick_y * .002));//Moves the arm.
                    telemetry.addData("position", currentPosition2);
                } else if (gamepad2.left_stick_y != 0 && gamepad2.right_stick_y != 0) {
                    double currentPosition2 = robot.wrist.getPosition();
                    robot.wrist.setPosition(currentPosition2);

                }
                //Open and close claw
                if (gamepad2.right_bumper) {
                    clawMode = clawMode+1;
                    sleep(500);
                }
                // Different Claw Positions
                if (clawMode >= 3) {
                    clawMode = 0;
                }
                if (clawMode == 0) {
                    robot.claw.setPosition(robot.CLAW_OPENED);
                    telemetry.addData("Status", clawMode);
                }
                if (clawMode == 1) {
                    robot.claw.setPosition(robot.CLAW_KINDAOPEN);
                }
                if (clawMode == 2) {
                    robot.claw.setPosition(robot.CLAW_CLOSED);
                }

                //Twists the claw
                if (gamepad2.right_trigger > 0.5) {
                    robot.clawTwist.setPosition(robot.TWIST_UP);
                }
                if (gamepad2.left_trigger > 0.5) {
                    robot.clawTwist.setPosition(robot.TWIST_DOWN);
                }
            }

            if (armMode == 1) {
                //Controls at shoulder
                //From above this line declares it int currentPos =robot.shoulder.getCurrentPosition();
                if (gamepad2.dpad_left) {
                    robot.shoulder.setTargetPosition(robot.shoulder.getCurrentPosition() + 5);
                    robot.shoulder.setPower(0.2);
                    currentPos = robot.shoulder.getCurrentPosition();//Set the position to lock at;
                } else if (gamepad2.dpad_right) {
                    robot.shoulder.setTargetPosition(robot.shoulder.getCurrentPosition() - 5);
                    robot.shoulder.setPower(0.2);
                    currentPos = robot.shoulder.getCurrentPosition();
                } else {

                    robot.shoulder.setTargetPosition(currentPos);
                    robot.shoulder.setPower(1.00);
                    sleep(10);
                }
                //Controls at elbow
                if (Math.abs(gamepad2.right_stick_x) >= 0.05) {
                    robot.elbow.setPosition(robot.elbow.getPosition() + (-gamepad2.right_stick_x * 0.01));
                } else {
                    robot.elbow.setPosition(robot.elbow.getPosition());
                }
                //Controls at wrist
                if (Math.abs(gamepad2.left_stick_x) >= 0.05) {
                    robot.wrist.setPosition(robot.wrist.getPosition() + (-gamepad2.left_stick_x * 0.01));
                } else {
                    robot.wrist.setPosition(robot.wrist.getPosition());
                }
            }
            //Open and close claw

//            if(gamepad2.right_bumper){
//                robot.claw.setPosition(robot.CLAW_OPENED);
//         }
//            if(gamepad2.left_bumper){
//                robot.claw.setPosition(robot.CLAW_CLOSED);
//            }

            //Twists the claw
            if (gamepad2.right_trigger > 0.5) {
                robot.clawTwist.setPosition(robot.TWIST_UP);
            }
            if (gamepad2.left_trigger > 0.5) {
                robot.clawTwist.setPosition(robot.TWIST_DOWN);
            }

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

        }//WhileOpModeIsActive() End.
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