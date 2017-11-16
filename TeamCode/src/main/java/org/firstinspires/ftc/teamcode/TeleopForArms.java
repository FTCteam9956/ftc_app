package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleopForArms", group = "Teleop")

public class TeleopForArms extends LinearOpMode {
    RRHardwarePresets robot = new RRHardwarePresets();

    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();


        while (opModeIsActive()) {

            if(gamepad2.a){
                robot.moveServo(robot.elbow, robot.ELBOW_UNFOLDED, 1000, 2000);
            }
            if(gamepad2.b){
                robot.moveServo(robot.wrist, robot.WRIST_UNFOLDED, 1000, 2000);
            }
            if(gamepad2.y) {
                robot.moveServo(robot.elbow, robot.ELBOW_FOLDED, 1000, 2000);
                robot.moveServo(robot.wrist,  robot.WRIST_FOLDED, 1000, 2000);
            }

            idle();
        }
    }
}

