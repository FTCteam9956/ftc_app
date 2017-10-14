
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name = "AutonomousTest", group = "Autonomous")

public class AutonomousTest extends LinearOpMode
{
    DcMotor left1;
    DcMotor left2;
    DcMotor right1;
    DcMotor right2;

    public void runOpMode()
    {
        //Remember to label the motors the same as how we name them in the configuration on the FTC app. - Sam C.
        left1 = hardwareMap.dcMotor.get("left1"); //port 0
        left2 = hardwareMap.dcMotor.get("left2"); //port 1
        right1 = hardwareMap.dcMotor.get("right1"); //port 2
        right2 = hardwareMap.dcMotor.get("right2"); //port 3

        int DRIVE_STRAIGHT = 2000;
        //int TURN = 720;

        left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Tells us our speed.
        left1.setPower(0.3);
        left2.setPower(0.3);
        right1.setPower(0.3);
        right2.setPower(0.3);

        //Drive Straight
        left1.setTargetPosition(DRIVE_STRAIGHT);
        left2.setTargetPosition(DRIVE_STRAIGHT);
        right1.setTargetPosition(DRIVE_STRAIGHT);
        right2.setTargetPosition(DRIVE_STRAIGHT);

        telemetry.addData("ENCODER: (left1)", left1.getCurrentPosition());
        telemetry.addData("ENCODER: (left2)", left2.getCurrentPosition());
        telemetry.addData("ENCODER: (right1)", right1.getCurrentPosition());
        telemetry.addData("ENCODER: (right2)", right2.getCurrentPosition());

        //left1.setPower(0);
        //left2.setPower(0);
        //right1.setPower(0);
        //right2.setPower(0);
        sleep(1000);
        resetEncoders();

        //Turn
        //left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //left1.setTargetPosition(TURN);
        //right1.setTargetPosition(-TURN);
        //left1.setPower(0.15);
        //left2.setPower(0.15);
        //right1.setPower(0.15);
        //right2.setPower(0.15);

        //left1.setPower(0);
        //left2.setPower(0);
        //right1.setPower(0);
        //right2.setPower(0);
        //sleep(1000);
        //resetEncoders();

    }

    void resetEncoders() {
        left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}