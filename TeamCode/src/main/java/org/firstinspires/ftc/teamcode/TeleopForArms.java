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
                servoSpeed(0, 1.0, 2, 100, robot.elbow);
                servoSpeed(0, 1.0, 2, 100, robot.wrist);
            }
            if(gamepad2.y) {
                servoSpeed(1.0, 0, 2, 100, robot.elbow);
                servoSpeed(1.0, 0, 2, 100, robot.wrist);
            }


            idle();
        }
    }



    public void servoSpeed(double startingPosition, double finalPosition, int timeLimitMiliseconds, int numberOfSteps, Servo targetServo){
        ElapsedTime timer = new ElapsedTime();
        double distance = 0;
        int stepTime = 0;
        int stepNumber = 0;
        double changeNumber = startingPosition;
        double currentPosition = 1;
        distance = Math.abs(startingPosition - finalPosition) / numberOfSteps;
        stepTime = (timeLimitMiliseconds / numberOfSteps);
        timer.reset();
        while(timer.milliseconds() < (timeLimitMiliseconds * 1000) && stepNumber < numberOfSteps) {
            currentPosition = changeNumber - distance;
            targetServo.setPosition(currentPosition);
            sleep(20);
            changeNumber = changeNumber - distance;
            stepNumber++;
        }
    }
}