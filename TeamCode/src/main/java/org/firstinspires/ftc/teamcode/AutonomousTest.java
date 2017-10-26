//AutonomousTest.java

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutonomousTest", group = "Autonomous")
//@Disabled
public class AutonomousTest extends LinearOpMode{
    RRHardwarePresets robot = new RRHardwarePresets(1);

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        robot.resetEncoders();
        waitForStart();

        //Set smacker position down.
        robot.smacker.setPosition(robot.SMACKER_DOWN);
        sleep(1000);

        //Read the joule with the sensor.
        if(robot.joule.blue() < robot.joule.red()){
            telemetry.addData("Color", "Red");
        }
        else{
            telemetry.addData("Color", "Blue");
        }

        //Spin the turret

        //Lift Arm
        robot.smacker.setPosition(robot.SMACKER_UP);
        sleep(500);

        //Spin the turret back.

        //Read Picture

        //Drive Straight
        robot.left1.setTargetPosition(robot.DRIVE_STRAIGHT);
        robot.left2.setTargetPosition(robot.DRIVE_STRAIGHT);
        robot.right1.setTargetPosition(robot.DRIVE_STRAIGHT);
        robot.right2.setTargetPosition(robot.DRIVE_STRAIGHT);

        robot.left1.setPower(0.95);
        robot.left2.setPower(0.95);
        robot.right1.setPower(0.95);
        robot.right2.setPower(0.95);
        while((motorBusy(robot.left1)) || motorBusy(robot.right1) || motorBusy(robot.right2) || motorBusy(robot.left2) && opModeIsActive()){
            telemetry.addData("Status", "Driving Straight");
            telemetry.addData("Target", robot.DRIVE_STRAIGHT);
            telemetry.addData("ENCODER: (left1)", robot.left1.getCurrentPosition());
            telemetry.addData("ENCODER: (left2)", robot.left2.getCurrentPosition());
            telemetry.addData("ENCODER: (right1)", robot.right1.getCurrentPosition());
            telemetry.addData("ENCODER: (right2)", robot.right2.getCurrentPosition());
            updateTelemetry(telemetry);
        }

        robot.left1.setPower(0);
        robot.left2.setPower(0);
        robot.right1.setPower(0);
        robot.right2.setPower(0);
        sleep(500);
        robot.resetEncoders();

        //Spin Turret

        //Release Block
        robot.claw.setPosition(robot.CLAW_OPEN);
    }

    boolean motorBusy(DcMotor m){
        return m.getCurrentPosition() < m.getTargetPosition() - 10 || m.getCurrentPosition() > m.getTargetPosition() +10;
    }
}