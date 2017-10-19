//TeleOpTest.java

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
        //Initializing Motors.
        left1 = hardwareMap.dcMotor.get("left1"); //port 0
        left2 = hardwareMap.dcMotor.get("left2"); //port 1 //Encoder
        right1 = hardwareMap.dcMotor.get("right1"); //port 2
        right2 = hardwareMap.dcMotor.get("right2"); //port 3 //Encoder

        //Stops robot from coasting.
        left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //No Encoders
        left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Encoders
        left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Reverse because motors are across from each other.
        right1.setDirection(DcMotorSimple.Direction.REVERSE);
        right2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive())
        {
            //Sets power for leading motors.
            left2.setPower(speedAdjust(gamepad1.left_stick_y));
            right2.setPower(speedAdjust(gamepad1.right_stick_y));

            //Use this if we are only using 2 encoders in order to "clone" power level.
            left1.setPower(left2.getPower());
            right1.setPower(right2.getPower());

            //Telemetry
            //telemetry.addData("ENCODER: (left1)", left1.getCurrentPosition());
            //telemetry.addData("ENCODER: (left2)", left2.getCurrentPosition());
            //telemetry.addData("ENCODER: (right1)", right1.getCurrentPosition());
            //telemetry.addData("ENCODER: (right2)", right2.getCurrentPosition());
            telemetry.addData("Left1 Power", left1.getPower());
            telemetry.addData("Left2 Power", left2.getPower());
            telemetry.addData("Right1 Power", right1.getPower());
            telemetry.addData("Right2 Power", right2.getPower());
            telemetry.addData("Left Stick", gamepad1.left_stick_y);
            telemetry.addData("Right Stick", gamepad1.right_stick_y);

            telemetry.update();

            idle();
        }
    }

    public static float speedAdjust(float stickInput){
        if(stickInput > 0){
            return(stickInput * stickInput);
        }else if(stickInput < 0){
            return(-1 * (stickInput * stickInput));
        }else{
            return(stickInput);
        }
    }
}