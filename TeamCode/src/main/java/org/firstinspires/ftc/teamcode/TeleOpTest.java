//TeleOpTest.java
//Goal of this program is just to spin 1 motor named left1.

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@Disabled
@TeleOp(name = "TeleOpTest", group = "Teleop")

public class TeleOpTest extends LinearOpMode
{
    DcMotor left1;
    DcMotor left2;
    DcMotor right1;
    DcMotor right2;

    @Override
    public void runOpMode()
    {
        //Remember to label the motors the same as how we name them in the configuration on the FTC app. - Sam C.
        left1 = hardwareMap.dcMotor.get("left1"); //port 0
        left2 = hardwareMap.dcMotor.get("left2"); //port 1
        right1 = hardwareMap.dcMotor.get("right1"); //port 2
        right2 = hardwareMap.dcMotor.get("right2"); //port 3

        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive())
        {
            left1.setPower(gamepad1.left_stick_y);
            left2.setPower(gamepad1.left_stick_y);
            right1.setPower(gamepad1.right_stick_y);
            right2.setPower(gamepad1.right_stick_y);

            telemetry.addData("ENCODER: (left1)", left1.getCurrentPosition());
            telemetry.addData("ENCODER: (left2)", left2.getCurrentPosition());
            telemetry.addData("ENCODER: (right1)", right1.getCurrentPosition());
            telemetry.addData("ENCODER: (right2)", right2.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }
}