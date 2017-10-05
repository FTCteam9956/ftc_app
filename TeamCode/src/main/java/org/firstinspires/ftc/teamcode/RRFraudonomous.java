package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

//ports 0 and 2 encoder results
//drive forward
//turn
//infinite donuts
@Autonomous(name = "FraudAutonomous", group = "Autonomous")
public class RRFraudonomous extends LinearOpMode
{
    DcMotor left1;
    DcMotor left2;
    DcMotor right1;
    DcMotor right2;

    final static int DRIVE_STRAIGHT = 2000;
    final static int TURN = 720;

    public void runOpMode()
    {
        //Drive Straight
        left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left1.setTargetPosition(DRIVE_STRAIGHT);
        right1.setTargetPosition(DRIVE_STRAIGHT);
        left1.setPower(0.3);
        left2.setPower(0.3);
        right1.setPower(0.3);
        right2.setPower(0.3);

        left1.setPower(0);
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);
        sleep(1000);
        resetEncoders();

        //Turn
        left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left1.setTargetPosition(TURN);
        right1.setTargetPosition(-TURN);
        left1.setPower(0.15);
        left2.setPower(0.15);
        right1.setPower(0.15);
        right2.setPower(0.15);

        left1.setPower(0);
        left2.setPower(0);
        right1.setPower(0);
        right2.setPower(0);
        sleep(1000);
        resetEncoders();

        }

    void resetEncoders() {
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
}