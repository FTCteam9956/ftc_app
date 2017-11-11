package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleopForArms", group = "Teleop")

public class TeleopForArms extends LinearOpMode {
    RRHardwarePresets robot = new RRHardwarePresets();

    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {

//            if (gamepad2.a) {
//                servoSpeedElbow(1.0, 0, 2, 100);
//            }
            if (gamepad2.y) {
                servoSpeedWrist(1.0, 0, 2, 100);
            }


            idle();
        }
    }

    public void servoSpeedWrist(double startingPosition, double finalPosition, int timeLimitMiliseconds, int numberOfSteps) {
        ElapsedTime timer = new ElapsedTime();

        double distance = 0;
        int stepTime = 0;
        int stepNumber = 0;
        double changeNumber = startingPosition;
        double currentPosition = 1;

        distance = Math.abs(startingPosition - finalPosition) / numberOfSteps;
        stepTime = (timeLimitMiliseconds / numberOfSteps);

        timer.reset();

        while (timer.milliseconds() < (timeLimitMiliseconds * 1000) && stepNumber < numberOfSteps) {
            currentPosition = changeNumber - distance;
            robot.wrist.setPosition(currentPosition);
            sleep(20);
            changeNumber = changeNumber - distance;
            stepNumber++;
            telemetry.addData("Current Wrist Position", robot.wrist.getPosition());
            telemetry.update();
        }
    }

    public void servoSpeedElbow(double startingPosition, double finalPosition, int timeLimitMiliseconds, int numberOfSteps) {
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
            robot.elbow.setPosition(currentPosition);
            sleep(20);
            changeNumber = changeNumber - distance;
            stepNumber++;
        }
    }
}
