package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import  com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ElapsedTime.Resolution;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;
import java.util.TimerTask;

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
    public static int sliderClawmode = 0;
    public static int shoulderPos = 0;
    public static int endGameMode = 0;
    public static int sliderTwistMode = 0;
    public static int mecanumMode = 3;
    double timeOutLimit = 3000000;
    boolean timeOutFlag = true;
    public double blockTime = 0;
    public double nanoInit = 0;
    //public static int mecanumMode1 = 0;
    public float rightPower;
    public float leftPower;

    public void runOpMode() {

        //Initializing Hardware
        robot.init(hardwareMap);
        robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.winch.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        mRunTime.reset();
        nanoInit = System.nanoTime();

        waitForStart();

        //Set jewelarm and Relic Claw position
        robot.relicClaw.setPosition(robot.RELIC_CLAW_OPENED);
        robot.lowerArm.setPosition(robot.JEWEL_ARM_UP);
        robot.rotateArm.setPosition(0.6);

        while(opModeIsActive()){

            //Allows us to switch modes to have more control
            if(gamepad1.start){
                endGameMode = 1;
            }else if(gamepad2.start){
                endGameMode = 0;
            }

            if(endGameMode == 0) {

                //TANK DRIVE
                robot.left1.setPower(speedAdjust(gamepad1.left_stick_y / 1.5));
                robot.left2.setPower(speedAdjust(gamepad1.left_stick_y / 1.5));
                robot.right1.setPower(speedAdjust(gamepad1.right_stick_y / 1.5));
                robot.right2.setPower(speedAdjust(gamepad1.right_stick_y / 1.5));

                //WINCH CONTROLS
                if (robot.liftlimita.getState() == false && robot.liftlimitb.getState() == false) {//AT Top
                    if (gamepad1.left_trigger < 0.01) { // if not pressing down trigger
                        robot.winch.setPower(0.0);
                    } else { // if pressing down trigger
                        robot.winch.setPower(-0.9);
                    }
                } else if (robot.winchLimit.getState() == false) {// At bot
                    if (gamepad1.right_trigger < 0.01) { // if not pressing up trigger
                        robot.winch.setPower(0.0);
                    } else { //if pressing up trigger
                        robot.winch.setPower(0.9);
                    }
                } else { //mid area
                    if (gamepad1.right_trigger > 0.5) {
                        robot.winch.setPower(0.9);
                    } else if (gamepad1.left_trigger > 0.5) {
                        robot.winch.setPower(-0.9);
                    } else {
                        robot.winch.setPower(0.0);
                    }
                }

                //MECANUM CLAW CONTROLS
                if (gamepad1.a) {
                    mecanumMode = 0;
                }
                if (gamepad1.b) {
                    mecanumMode = 1;
                }
                if (gamepad1.y) {
                    mecanumMode = 2;
                }
                if (gamepad1.x){
                    mecanumMode =3;
                }

                if(mecanumMode == 0){ //FORWARD
                    robot.bottomRight.setPower(-0.9);
                    robot.topRight.setPower(0.9);
                    robot.bottomLeft.setPower(0.9);
                    robot.topLeft.setPower(-0.9);
//                    if (robot.blockFlat.alpha() > 400) {
//                     robot.blockRotate.setPower(0.0);
//                    }
//                    else {
                        robot.blockRotate.setPower(0.53);
//                    }
                }
                else if(mecanumMode == 1){ //STOP
                    robot.bottomRight.setPower(0.0);
                    robot.topRight.setPower(0.0);
                    robot.bottomLeft.setPower(0.0);
                    robot.topLeft.setPower(0.0);
                    robot.blockRotate.setPower(0.0);
                }
                else if(mecanumMode == 2){ //BACKWARDS
                    robot.bottomRight.setPower(0.9);
                    robot.topRight.setPower(-0.9);
                    robot.bottomLeft.setPower(-0.9);
                    robot.topLeft.setPower(0.9);
                    robot.blockRotate.setPower(0.53);
                } else if (mecanumMode == 3){
                    robot.bottomRight.setPower(-0.9);
                    robot.topRight.setPower(0.0);
                    robot.bottomLeft.setPower(0.9);
                    robot.topLeft.setPower(0.0);
                } else{
                    robot.bottomRight.setPower(0.0);
                    robot.topRight.setPower(0.0);
                    robot.bottomLeft.setPower(0.0);
                    robot.topLeft.setPower(0.0);
                    robot.blockRotate.setPower(0.0);
                }
                    if (gamepad1.left_bumper) {
                        robot.clawTop.setPosition(robot.BLOCK_CLAW_OPEN_TOP);
                    }
                    if (gamepad1.right_bumper) {
                        robot.clawTop.setPosition(robot.BLOCK_CLAW_CLOSED_TOP);
                    }
                    if (robot.clawLimit.getState() == true) {
                        if (gamepad1.left_bumper) {
                            robot.clawBottom.setPosition(robot.BLOCK_CLAW_CLOSED_BOTTOM);
                        }
                        if (gamepad1.right_bumper) {
                            robot.clawBottom.setPosition(robot.BLOCK_CLAW_OPEN_BOTTOM);
                        }
                    }
//                    else {
//                            if (gamepad1.left_bumper) {
//                                robot.clawBottom.setPosition(robot.BLOCK_CLAW_CLOSED_BOTTOM);
//                            }
//                            if (gamepad1.right_bumper){
//                                robot.clawBottom.setPosition(robot.BLOCK_CLAW_LIMIT_BOTTOM);
//                            }
//                        }
                    if (robot.clawLimit.getState() == false){
                        if (gamepad1.left_bumper) {
                            robot.clawBottom.setPosition(robot.BLOCK_CLAW_LIMITO_BOTTOM);
                        }
                        if (gamepad1.right_bumper){
                            robot.clawBottom.setPosition(robot.BLOCK_CLAW_LIMIT_BOTTOM);
                        }
                    }

//                    if (robot.topLimit.getState() == true) {

                        if (robot.sensorDistance.getDistance(DistanceUnit.CM) < 6.5) {
//                            if ((System.nanoTime() - nanoInit) > timeOutLimit){
//                                timeOutFlag = false;
//                            }
                            if ((System.nanoTime() - nanoInit) > 3000000) {
                                robot.clawTop.setPosition(0.6);
                                nanoInit = System.nanoTime();
                            }
                            else if ((System.nanoTime() - nanoInit) > 1000000) {
                                robot.clawTop.setPosition(robot.BLOCK_CLAW_CLOSED_BOTTOM);
                            }
                        }

//                    }
//                    else if (robot.topLimit.getState() == false) {
//                        robot.topRight.setPower(0.0);
//                        robot.topLeft.setPower(0.0);
//                        sleep(250);
//                    }

//                    if ( robot.winchLimit.getState() == false) {
//                        if (robot.blockFlat.alpha() > 400) {
//                            robot.clawBottom.setPosition(robot.BLOCK_CLAW_CLOSED_BOTTOM);
//                            robot.blockRotate.setPower(0);
//                        } else {
//                            robot.clawBottom.setPosition(robot.BLOCK_CLAW_LIMIT_BOTTOM);
//                        }
//                    }
//                    telemetry.addData("Jewel Sensor - Red", robot.jewelArm.red());
//                    telemetry.addData("Jewel Sensor - Blue", robot.jewelArm.blue());
//                    telemetry.addData("Alpha Data", robot.glyphSensor.alpha());
//                    //telemetry.addData("Alpha Data Bot", robot.blockFlat.alpha());
//                    telemetry.addData("Mode", mecanumMode);
                    telemetry.addData("Distance (cm)",
                        String.format(Locale.US, "%.02f", robot.sensorDistance.getDistance(DistanceUnit.CM)));
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
                    if (gamepad2.left_bumper && sliderClawmode == 0) {
                        robot.relicClaw.setPosition(robot.RELIC_CLAW_CLOSED);
                        sleep(500);
                        sliderClawmode++;
                    } else if (gamepad2.left_bumper && sliderClawmode == 1) {
                        robot.relicClaw.setPosition(robot.RELIC_CLAW_OPENED);
                        sleep(500);
                        sliderClawmode--;
                    }
                    //Slider Twisting Controls
                    if (gamepad2.right_bumper && sliderTwistMode == 0) {
                        robot.relicTwist.setPosition(robot.RELIC_TWIST_UP);
                        sleep(500);
                        sliderTwistMode++;
                    } else if (gamepad2.right_bumper && sliderTwistMode == 1) {
                        robot.relicTwist.setPosition(robot.RELIC_TWIST_DOWN);
                        sleep(500);
                        sliderTwistMode--;
                    }
                }
//            telemetry.addData("Winch Position", robot.winch.getTargetPosition());
//            telemetry.addData("Winch Power", robot.winch.getPower());
//            telemetry.addData("Endgame Mode", endGameMode);
//            telemetry.addData("RELIC CLAW POS", robot.relicClaw.getPosition());
//            telemetry.addData("SHOULDER POS", shoulderPos);
//            telemetry.addData("Shoulder Encoder", robot.shoulder.getCurrentPosition());
//            telemetry.addData("Jewel Sensor - Red", robot.jewelArm.red());
//            telemetry.addData("Jewel Sensor - Blue", robot.jewelArm.blue());
                telemetry.addData("TOP CLAW", robot.clawTop.getPosition());
                telemetry.addData("Bot Claw", robot.clawBottom.getPosition());
//                if (robot.topLimit.getState() == true) {
//                    telemetry.addData("Digital Touch", "Is Not Pressed");
//                } else {
//                    telemetry.addData("Digital Touch", "Is Pressed");
//                }
                if (robot.clawLimit.getState() == true) {
                    telemetry.addData("Digital Claw Touch", "Is Not Pressed");
                } else {
                    telemetry.addData("Digital Claw Touch", "Is Pressed");
                }
                //telemetry.addData("Glyph Sensor Alpha", robot.glyphSensor.alpha());
                telemetry.update();

                //Sams Attempt
                //initTime = System.nanoTime() //AT TOP OUT OF WHILE
                //cooldown boolean = false;


                //if you see a block
                    //if(system.nanoTime() - initTime) > 3)
                        //init = System.nanoTime()
                    //else
                        //open for 1 second then close


                //if Top Limit isnt pushed in.
                    //if cooldown boolean = false;
                        //if we see a block AND ((System.nanoTime() - initTime) < 3 seconds)
                            //open claw
                            //initTime = System.nanoTime()
                        //else //After 3 seconds
                            //close claw
                            //cooldownBoolean = true


                //if clawState == 1 //Claw is open

                //if clawState == 0 //Claw is closed








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

   public ElapsedTime mRunTime = new ElapsedTime();
    enum State {delay}
    State currentState;
}
