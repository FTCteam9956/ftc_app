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
    public static int jewelArmMode = 0;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        waitForStart();
        robot.initServoPositions(); //This is where our motors move into their initial positions.
        robot.setRunMode("STOP_AND_RESET_ENCODER"); //Resets all encoders.
        robot.setRunMode("RUN_USING_ENCODER"); //Changes all motors to RUN_USING_ENCODER mode.
        robot.winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(opModeIsActive()){

            //---GAMEPAD 1---

            //TANK DRIVE
            robot.left1.setPower(speedAdjust(gamepad1.left_stick_y));
            robot.left2.setPower(speedAdjust(gamepad1.left_stick_y));
            robot.right1.setPower(speedAdjust(gamepad1.right_stick_y));
            robot.right2.setPower(speedAdjust(gamepad1.right_stick_y));

            //TURRET
            if(gamepad1.right_trigger > 0.5){
                robot.turretMotor.setPower(0.15);
            }else if(gamepad1.left_trigger > 0.5){
                robot.turretMotor.setPower(-0.15);
            }else{
                robot.turretMotor.setPower(0.0);
            }

            //JEWEL ARM
            if(gamepad1.dpad_down && jewelArmMode == 0){
                robot.jewelArm.setPosition(robot.JEWEL_ARM_DOWN);
                this.jewelArmMode = 1;
                sleep(500);
            }
            if(gamepad1.dpad_down && jewelArmMode == 1){
                robot.jewelArm.setPosition(robot.JEWEL_ARM_UP);
                this.jewelArmMode = 0;
                sleep(500);
            }

            //---GAMEPAD 2---

            //WINCH
            if(gamepad2.dpad_up){
                robot.winchMotor.setTargetPosition(robot.winchMotor.getTargetPosition() + 20);
                robot.winchMotor.setPower(0.35);
            }else if(gamepad2.dpad_down){
                robot.winchMotor.setTargetPosition(robot.winchMotor.getTargetPosition() - 20);
                robot.winchMotor.setPower(0.35);
            }else{
                robot.winchMotor.setTargetPosition(robot.winchMotor.getTargetPosition());
                robot.winchMotor.setPower(1.0);
            }

            //ARM POSITION PRESETS
            if(gamepad2.y){
                robot.moveMultipleServo(robot.wrist, robot.elbow, robot.WRIST_UNFOLDED, robot.ELBOW_UNFOLDED, 700, 150);
            }
            if(gamepad2.a){
                robot.moveMultipleServo(robot.wrist, robot.elbow, robot.WRIST_FOLDED, robot.ELBOW_FOLDED, 700, 150);
            }

            //SHOULDER
            if(gamepad2.dpad_left){
                //robot.shoulder.setTargetPosition(robot.shoulder.getCurrentPosition() + 10); //Sets TargetPosition to currentPosition + 1 encoder tick.
                robot.shoulder.setPower(speedAdjust(0.5));
                //while(robot.shoulder.isBusy()){
                    //Waiting for shoulder to reach destination
                //}
                //this.shoulderPosition = robot.shoulder.getCurrentPosition(); //Sets new Shoulder Position
            }else if(gamepad2.dpad_right){
                //robot.shoulder.setTargetPosition(robot.shoulder.getCurrentPosition() - 10); //Sets TargetPosition to currentPosition - 1 encoder tick.
                robot.shoulder.setPower(speedAdjust(-0.5)); //Sets power to run to TargetPosition

                //while(robot.shoulder.isBusy()){
                    //Waiting for shoulder to reach destination
                //}
                //this.shoulderPosition = robot.shoulder.getCurrentPosition(); //Sets new Shoulder Position
            }else{
                //robot.shoulder.setTargetPosition(shoulderPosition); //Sets TargetPosition to the static shoulder position value.
                robot.shoulder.setPower(0.0); //Sets power to run to TargetPosition
            }

            //ELBOW
            if(Math.abs(gamepad2.right_stick_x) >= 0.05){
                robot.elbow.setPosition(robot.elbow.getPosition() + (-gamepad2.right_stick_x * 0.01));
            }else{
                robot.elbow.setPosition(robot.elbow.getPosition());
            }

            //WRIST
            if(Math.abs(gamepad2.left_stick_x) >= 0.05){
                robot.wrist.setPosition(robot.wrist.getPosition() + (-gamepad2.left_stick_x * 0.01));
            }else{
                robot.wrist.setPosition(robot.wrist.getPosition());
            }

            //CLAW
            if(gamepad2.right_bumper && clawMode == 0){
                robot.claw.setPosition(robot.CLAW_OPENED);
                this.clawMode = 1;
                sleep(500);
            }
            if(gamepad2.right_bumper && clawMode == 1){
                robot.claw.setPosition(robot.CLAW_MID);
                this.clawMode = 2;
                sleep(500);
            }
            if(gamepad2.right_bumper && clawMode == 2){
                robot.claw.setPosition(robot.CLAW_CLOSED);
                this.clawMode = 0;
                sleep(500);
            }

            //CLAW TWIST
            if(gamepad2.left_bumper && clawTwistMode == 0){
                robot.clawTwist.setPosition(robot.TWIST_DOWN);
                this.clawTwistMode = 1;
                sleep(500);
            }
            if(gamepad2.left_bumper && clawTwistMode == 1){
                robot.clawTwist.setPosition(robot.TWIST_UP);
                this.clawTwistMode = 0;
                sleep(500);
            }

            //---TELEMETRY---
            telemetry.addData("left1 encoder", robot.left1.getCurrentPosition());
            telemetry.addData("left2 encoder", robot.left2.getCurrentPosition());
            telemetry.addData("right1 encoder", robot.right1.getCurrentPosition());
            telemetry.addData("right2 encoder", robot.right2.getCurrentPosition());
            //telemetry.addData("Left1 Power", robot.left1.getPower());
            //telemetry.addData("Left2 Power", robot.left2.getPower());
            //telemetry.addData("Right1 Power", robot.right1.getPower());
            //telemetry.addData("Right2 Power", robot.right2.getPower());
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

            telemetry.update();
            idle();

        }//WhileOpModeIsActive() End.
    }

    //---TELEOP ONLY METHODS BELOW---

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
