//AutonomousTest.java

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;

@Autonomous(name = "AutonomousTest", group = "Autonomous")
//@Disabled
public class AutonomousTest extends LinearOpMode{
    RRHardwarePresets robot = new RRHardwarePresets(1);
    //Because we are giving 1 as a parameter the drive mode will be set to RUN_TO_POSITION

    //Constants.
    final int DRIVE_STRAIGHT = 5000;
    final int DRIVE_OFF = 1000;
    final int TRIANGLE_DRIVE = 500;
    final double SMACKER_UP = 0.28;
    final double SMACKER_DOWN = 1;

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);

        //robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.resetEncoders();

        waitForStart();
        robot.floor.enableLed(false);

        //Lower arm.
        robot.smacker.setPosition(robot.SMACKER_DOWN);
        sleep(500);

        //Read the jewel with the sensor.

        //Spin the turret based on jewel color.

        //Lift Arm
        robot.smacker.setPosition(robot.SMACKER_UP);
        sleep(500);

        //Spin the turret back.

        //Read picture.

        //Drive off of balance stone.
        driveStraight(DRIVE_OFF);
        sleep(500);

        //while(motorBusy(robot.left1) && motorBusy(robot.left2) && motorBusy(robot.right1) && motorBusy(robot.right2) && opModeIsActive()){
        //    telemetry.addData("Status", "Driving Straight");
        //    telemetry.addData("Target", DRIVE_OFF);
        //    telemetry.addData("ENCODER: (left1)", robot.left1.getCurrentPosition());
        //    telemetry.addData("ENCODER: (left2)", robot.left2.getCurrentPosition());
        //    telemetry.addData("ENCODER: (right1)", robot.right1.getCurrentPosition());
        //    telemetry.addData("ENCODER: (right2)", robot.right2.getCurrentPosition());
        //    updateTelemetry(telemetry);
        //}

        while(opModeIsActive()){
            updateTelemetry(telemetry);
        }

        robot.resetEncoders();
        robot.floor.enableLed(true);
        sleep(500);

        //Drive straight to the Triangle
        telemetry.addData("Red", robot.floor.red());
        telemetry.addData("Blue", robot.floor.blue());
        updateTelemetry(telemetry);
        sleep(1000);

        while (robot.floor.red() < 150 && opModeIsActive()){

            driveStraight(DRIVE_STRAIGHT);
            setMotorsPower(0.35);
            telemetry.addData("Red", robot.floor.red());
            telemetry.addData("Blue", robot.floor.blue());
            updateTelemetry(telemetry);
        }
            setMotorsPower(0.0);
            sleep(500);

            robot.floor.enableLed(false);
            robot.resetEncoders();
    }

    public boolean motorBusy(DcMotor m){
        return m.getCurrentPosition() < m.getTargetPosition() - 10 || m.getCurrentPosition() > m.getTargetPosition() +10;
    }

    public void driveStraight(int distance){
        telemetry.addData("status", "driveStraight Flag");
        robot.left1.setTargetPosition(distance);
        robot.left2.setTargetPosition(distance);
        robot.right1.setTargetPosition(distance);
        robot.right2.setTargetPosition(distance);
    }

    public void setMotorsPower(double inputPower){
        telemetry.addData("status", "setMotorsPower Flag");
        robot.left1.setPower(inputPower);
        robot.left2.setPower(inputPower);
        robot.right1.setPower(inputPower);
        robot.right2.setPower(inputPower);
    }
}