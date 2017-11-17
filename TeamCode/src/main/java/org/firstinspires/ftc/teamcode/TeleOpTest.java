//TeleOpTest.java

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpTest", group = "Teleop")
//@Disabled

public class TeleOpTest extends LinearOpMode{
    RRHardwarePresets robot = new RRHardwarePresets();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.setRunMode("RUN_USING_ENCODER");

        waitForStart();
        while(opModeIsActive()){

            //Sets power for DC motors.
            robot.left1.setPower(speedAdjust(gamepad1.left_stick_y));
            robot.left2.setPower(speedAdjust(gamepad1.left_stick_y));
            robot.right1.setPower(speedAdjust(gamepad1.right_stick_y));
            robot.right2.setPower(speedAdjust(gamepad1.right_stick_y));

            //---GAMEPAD 1---

            //DC motor turret controls.
            if(gamepad1.right_trigger > 0.5){
                robot.turretMotor.setPower(0.15);
            }
            else if(gamepad1.left_trigger > 0.5){
                robot.turretMotor.setPower(-0.15);
            }else{
                robot.turretMotor.setPower(0.0);
            }

            //Servo jewelArm controls.
            if(gamepad1.dpad_up){
                robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_UP, 1000, 3000);
            }
            if(gamepad1.dpad_down){
                robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_DOWN, 1000, 3000);
            }

            //Arm Tests, currently these servos do not run concurrently.
            if(gamepad1.dpad_left){
                robot.moveServo(robot.elbow, robot.ELBOW_UNFOLDED, 1000, 3000);
                robot.moveServo(robot.wrist, robot.WRIST_UNFOLDED, 1000, 3000);
            }
            if(gamepad1.dpad_right){
                robot.moveServo(robot.wrist, robot.WRIST_FOLDED, 1000, 3000);
                robot.moveServo(robot.elbow, robot.ELBOW_FOLDED, 1000, 3000);
            }

            //---GAMEPAD 2---

            //TeleOp control over arm.
            if(gamepad2.left_stick_y != 0){
                double currentPosition1 = robot.elbow.getPosition(); //Gets current arm position.
                double currentPosition2 = robot.wrist.getPosition();
                robot.elbow.setPosition(currentPosition1 + (-gamepad2.left_stick_y * .002));//Moves the arm.
                robot.wrist.setPosition(currentPosition2 + (-gamepad2.left_stick_y * .002));//Moves the arm.
                //moveWithJoystick(robot.wrist, robot.elbow, gamepad2.left_stick_y, gamepad2.left_stick_y);
                telemetry.addData("CurrentPosition1", currentPosition1);
                telemetry.addData("CurrentPosition2",currentPosition2);
            }
            else if(gamepad2.right_stick_y != 0){
                double currentPosition2 = robot.wrist.getPosition();
                robot.wrist.setPosition(currentPosition2 + (-gamepad2.left_stick_y * .002));//Moves the arm.
                //moveWithJoystick(robot.wrist, robot.elbow, gamepad2.left_stick_y, gamepad2.left_stick_y);
                telemetry.addData("posiition",currentPosition2);
            }
            else if(gamepad2.left_stick_y != 0 && gamepad2.right_stick_y != 0){
                double currentPosition2 = robot.wrist.getPosition();
                robot.wrist.setPosition(currentPosition2);
            }

            //---TELEMETRY---

            //Telemetry
            //telemetry.addData("left1 encoder", robot.left1.getCurrentPosition());
            //telemetry.addData("left2 encoder", robot.left2.getCurrentPosition());
            //telemetry.addData("right1 encoder", robot.right1.getCurrentPosition());
            //telemetry.addData("right2 encoder", robot.right2.getCurrentPosition());
            //telemetry.addData("Left1 Power", robot.left1.getPower());
            //telemetry.addData("Left2 Power", robot.left2.getPower());
            //telemetry.addData("Right1 Power", robot.right1.getPower());
            //telemetry.addData("Right2 Power", robot.right2.getPower());
            //telemetry.addData("Claw Position", robot.claw.getPosition());
            //telemetry.addData("JewelArm Position", robot.jewelArm.getPosition());
            //telemetry.addData("Left Stick", gamepad1.left_stick_y);
            //telemetry.addData("Right Stick", gamepad1.right_stick_y);
            //telemetry.addData("TurretMotor", robot.turretMotor.getCurrentPosition());
            //telemetry.addData("Floor Sensor", robot.floorSensor.argb());
            //telemetry.addData("Jewel Sensor", robot.jewelSensor.argb());
            //telemetry.addData("Elbow Position", robot.elbow.getPosition());
            //telemetry.addData("Wrist Position", robot.wrist.getPosition());

            telemetry.update();
            idle();
        }
    }

    //---TELEOP ONLY METHODS BELOW---

    //Used to smooth out acceleration of robot.
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