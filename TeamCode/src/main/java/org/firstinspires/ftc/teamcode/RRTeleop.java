package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "RRTeleop", group = "Teleop")
public class RRTeleop extends LinearOpMode
{
    DcMotor left1;
    DcMotor left2;
    DcMotor right1;
    DcMotor right2;

    @Override
    public void runOpMode()
    {
        left1 =hardwareMap.dcMotor.get("left1");
        left2 =hardwareMap.dcMotor.get("left2");
        right1 =hardwareMap.dcMotor.get("right1");
        right2 =hardwareMap.dcMotor.get("right2");

        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left2.setDirection(DcMotorSimple.Direction.REVERSE);

        right1.setDirection(DcMotorSimple.Direction.FORWARD);
        right2.setDirection(DcMotorSimple.Direction.FORWARD);

        left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive())
        {
            left1.setPower(gamepad1.left_stick_y);
            left2.setPower(gamepad1.left_stick_y);
            right1.setPower(gamepad1.right_stick_y);
            right2.setPower(gamepad1.right_stick_y);

            idle();
        }
    }
}