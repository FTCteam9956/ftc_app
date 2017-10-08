package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOpTest", group = "Teleop")
public class TeleOpTest extends LinearOpMode
{
    DcMotor left1;

    @Override
    public void runOpMode()
    {
        left1 = hardwareMap.dcMotor.get("left1");
        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive())
        {
            left1.setPower(gamepad1.left_stick_y);
            idle();
        }
    }
}