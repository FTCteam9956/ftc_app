//TeleOpTest.java
//Goal of this program is just to spin 1 motor named left1.

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//This is where we define what the op mode will be called on the FTC app.
@TeleOp(name = "TeleOpTest", group = "Teleop")

public class TeleOpTest extends LinearOpMode
{
    DcMotor left1;

    @Override
    public void runOpMode()
    {
        //Remember to label the motors the same as how we name them in the configuration on the FTC app. - Sam C.
        left1 = hardwareMap.dcMotor.get("left1");
        left1.setDirection(DcMotorSimple.Direction.REVERSE);
        left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while(opModeIsActive())
        {
            left1.setPower(gamepad1.left_stick_y);
            idle();

            telemetry.addData("ENCODER: ", left1.getCurrentPosition());
            telemetry.update();
        }
    }
}