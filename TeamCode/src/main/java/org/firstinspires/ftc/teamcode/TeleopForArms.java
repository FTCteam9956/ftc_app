package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleopForArms", group = "Teleop")

public class TeleopForArms extends LinearOpMode {
    RRHardwarePresets robot = new RRHardwarePresets();

    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        robot.elbow.setPosition(1);
        robot.wrist.setPosition(1);
        boolean mode1 = true;
        while (opModeIsActive()) {

            if(gamepad2.y){
                robot.moveMultipleServo(robot.wrist, robot.elbow, robot.WRIST_UNFOLDED, robot.ELBOW_UNFOLDED, 1000, 500);
            }
            if(gamepad2.a) {
                robot.moveMultipleServo(robot.wrist, robot.elbow, robot.WRIST_FOLDED, robot.ELBOW_FOLDED, 1000, 500);

            }
            if(gamepad2.left_stick_y != 0){
                double currentPosition1 = robot.elbow.getPosition(); //Gets current arm position.
                double currentPosition2 = robot.wrist.getPosition();
                robot.elbow.setPosition(currentPosition1 + (-gamepad2.left_stick_y * .002));//Moves the arm.
                robot.wrist.setPosition(currentPosition2 + (-gamepad2.left_stick_y * .002));//Moves the arm.
               //moveWithJoystick(robot.wrist, robot.elbow, gamepad2.left_stick_y, gamepad2.left_stick_y);
                telemetry.addData("Position", currentPosition1);
                telemetry.addData("posiition",currentPosition2);
            }
            else if(gamepad2.right_stick_y != 0){
                double currentPosition2 = robot.wrist.getPosition();
                robot.wrist.setPosition(currentPosition2 + (-gamepad2.left_stick_y * .002));//Moves the arm.
                //moveWithJoystick(robot.wrist, robot.elbow, gamepad2.left_stick_y, gamepad2.left_stick_y);
                telemetry.addData("posiition",currentPosition2);
            }
            else if(gamepad2.left_stick_y != 0 && gamepad2.right_stick_y != 0){
                double currentPosition2 = robot.wrist.getPosition();
                robot.wrist.setPosition(currentPosition2);
            }
            idle();
        }
    }
    }



