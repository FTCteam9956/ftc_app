package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "NewBlueStraight", group = "Autonomous")
@Disabled

public class NewBlueStraight extends LinearOpMode{
    GrantsTeleopHardware robot = new GrantsTeleopHardware();

    public final int FIRST_DISTANCE = -1200;

    public void runOpMode(){
        robot.init(hardwareMap);//Robot moves during init().
        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.setRunMode("RUN_USING_ENCODER");
        robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while(opModeIsActive()){
            robot.initServoPositions();

            robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_DOWN, 500, 1000);

            sleep(1000);

            int loopBreak = 0;
            while (loopBreak == 0) {
                sleep(500);
                if (robot.jewelArm.red() > 52) {
                    //knockOffBall(0);
                    robot.rotateArm.setPosition(0.45);
                    telemetry.addData("Status", "Confirmed Red Ball!");

                    loopBreak = 1;
                } else if (robot.jewelArm.red() <= 52) {
                    if (robot.jewelArm.blue() > 27) {
                        knockOffBall(1);
                        telemetry.addData("Status", "Confirmed Blue Ball!");
                        sleep(500);
                        loopBreak = 1;
                    } else {
                        telemetry.addData("Status", "Cannot determine color! Double Checking!");
                        robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_UP, 500, 1000);
                        sleep(500);
                        robot.rotateArm.setPosition(0.15);
                        robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_DOWN, 500, 1000);
                        sleep(500);
                        if (robot.jewelArm.red() > 52) {
                            robot.rotateArm.setPosition(0.45);
                            telemetry.addData("Status", "Confirmed Red Ball!");
                            loopBreak = 1;
                        } else if (robot.jewelArm.red() <= 52) {
                            if (robot.jewelArm.blue() > 27) {
                                knockOffBall(1);
                                telemetry.addData("Status", "Confirmed Blue Ball!");
                                sleep(500);
                                loopBreak = 1;
                            } else {
                                telemetry.addData("Status", "Cannot determine color! You screwed up!");
                                loopBreak = 1;
                            }
                        }
                    }
                }
            }
            telemetry.update();
            sleep(500);
            robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_UP, 500, 1000);
            sleep(500);
            robot.rotateArm.setPosition(0.6);
            sleep(200);

            robot.driveForwardSetDistance(-0.2, -FIRST_DISTANCE);
            sleep(250);
            idle();
        }
    }
    public void knockOffBall(int selection){

        if (selection == 0) {
            robot.rotateArm.setPosition(robot.ROTATE_RIGHT);
        }
        if (selection == 1) {
            robot.rotateArm.setPosition(robot.ROTATE_LEFT);
        }
        sleep(100);
        robot.rotateArm.setPosition(robot.ROTATE_MID);
    }
}
