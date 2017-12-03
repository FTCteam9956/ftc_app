package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "TestPose", group = "Teleop")
//@Disabled

public class TestPose extends LinearOpMode{

    //Instantiation of RRHardwarePresets
    RRHardwarePresets robot = new RRHardwarePresets();

    public static double elbowPosition = 1.0;
    public static double wristPosition = 0.0;

    public void runOpMode(){
        robot.init(hardwareMap);
        robot.shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        waitForStart();
        while(opModeIsActive()){

            if(gamepad1.dpad_right){
                this.elbowPosition = elbowPosition + 0.01;
                sleep(50);
            }
            if(gamepad1.dpad_left){
                this.elbowPosition = elbowPosition - 0.01;
                sleep(50);
            }
            if(gamepad1.dpad_up){
                this.wristPosition = wristPosition + 0.01;
                sleep(50);
            }
            if(gamepad1.dpad_down){
                this.wristPosition = wristPosition - 0.01;
                sleep(50);
            }

            robot.elbow.setPosition(elbowPosition);
            robot.wrist.setPosition(wristPosition);

            telemetry.addData("Turret Position", robot.turretMotor.getCurrentPosition());
            telemetry.addData("Shoulder Position", robot.shoulder.getCurrentPosition());
            telemetry.addData("Elbow Position", robot.elbow.getPosition());
            telemetry.addData("Wrist Position", robot.wrist.getPosition());
            telemetry.update();
        }
    }
}
