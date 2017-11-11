//TeleOpTest.java

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "TeleOpTest", group = "Teleop")
//@Disabled

public class TeleOpTest extends LinearOpMode{
    RRHardwarePresets robot = new RRHardwarePresets();

    @Override
    public void runOpMode(){
        robot.init(hardwareMap);
        setRunMode("STOP_AND_RESET_ENCODER");
        setRunMode("RUN_USING_ENCODER");
        robot.jewelArm.setPosition(0.7);
        waitForStart();
        while(opModeIsActive()){

            //Sets power for DC motors.
            robot.left1.setPower(speedAdjust(gamepad1.left_stick_y));
            robot.left2.setPower(speedAdjust(gamepad1.left_stick_y));
            robot.right1.setPower(speedAdjust(gamepad1.right_stick_y));
            robot.right2.setPower(speedAdjust(gamepad1.right_stick_y));

            //turret controls.
            if(gamepad1.right_trigger > 0.5){
                robot.turretMotor.setPower(0.15);
            }
            else if(gamepad1.left_trigger > 0.5){
                robot.claw.setPosition(0);
                robot.turretMotor.setPower(-0.15);
            }else{
                robot.turretMotor.setPower(0.0);
            }

            //jewelArm controls.
            if(gamepad1.dpad_up){
                robot.jewelArm.setPosition(robot.JEWEL_ARM_UP);
            }
            if(gamepad1.dpad_down){
                servoSpeed(0.7, 0.0, 2, 100); //start position, ending position, time to move
            }


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
            telemetry.addData("IMU angular orientation", robot.imu.getAngularOrientation());
            


            telemetry.update();

            idle();
        }
    }

    //Sets the run mode of all DC motors.
    public void setRunMode(String input){
        if(input.equals("STOP_AND_RESET_ENCODER")) {
            robot.left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        if(input.equals("RUN_WITHOUT_ENCODER")) {
            robot.left1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.left2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.right2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if(input.equals("RUN_USING_ENCODER")) {
            robot.left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(input.equals("RUN_TO_POSITION")) {
            robot.left1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.left2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

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
    public void servoSpeed(double startingPosition, double finalPosition, int timeLimitMiliseconds, int numberOfSteps) {
        ElapsedTime timer = new ElapsedTime();

        double distance = 0;
        int stepTime = 0;
        int stepNumber = 0;
        double changeNumber = startingPosition;
        double currentPosition = 1;

        distance = Math.abs(startingPosition - finalPosition) / numberOfSteps;
        stepTime = (timeLimitMiliseconds / numberOfSteps);

        timer.reset();

        while(timer.milliseconds() < (timeLimitMiliseconds * 1000) && stepNumber < numberOfSteps) {
            currentPosition = changeNumber - distance;
            robot.jewelArm.setPosition(currentPosition);
            sleep(20);
            changeNumber = changeNumber - distance;
            stepNumber++;
        }

//        double slope = 0;
//        double currentPosition = 0;
//
//        slope = (finalPosition - startingPosition) / timeLimitSeconds;
//        currentPosition = startingPosition;
//
//        timer.reset();
//
//        while (timer.milliseconds() < 2000) {
//            currentPosition = ((Math.abs(slope)) * timer.seconds()) + startingPosition;
//            robot.jewelArm.setPosition(currentPosition);
//            sleep(3);
//        }
    }
    public void servoSpeedWrist(double startingPosition, double finalPosition, int timeLimitMiliseconds, int numberOfSteps) {
        ElapsedTime timer = new ElapsedTime();

        double distance = 0;
        int stepTime = 0;
        int stepNumber = 0;
        double changeNumber = startingPosition;
        double currentPosition = 1;

        distance = Math.abs(startingPosition - finalPosition) / numberOfSteps;
        stepTime = (timeLimitMiliseconds / numberOfSteps);

        timer.reset();

        while(timer.milliseconds() < (timeLimitMiliseconds * 1000) && stepNumber < numberOfSteps) {
            currentPosition = changeNumber - distance;
            robot.jewelArm.setPosition(currentPosition);
            sleep(20);
            changeNumber = changeNumber - distance;
            stepNumber++;
        }
    }
    public void servoSpeedElbow(double startingPosition, double finalPosition, int timeLimitMiliseconds, int numberOfSteps) {
        ElapsedTime timer = new ElapsedTime();

        double distance = 0;
        int stepTime = 0;
        int stepNumber = 0;
        double changeNumber = startingPosition;
        double currentPosition = 1;

        distance = Math.abs(startingPosition - finalPosition) / numberOfSteps;
        stepTime = (timeLimitMiliseconds / numberOfSteps);

        timer.reset();

        while(timer.milliseconds() < (timeLimitMiliseconds * 1000) && stepNumber < numberOfSteps) {
            currentPosition = changeNumber - distance;
            robot.elbow.setPosition(currentPosition);
            sleep(20);
            changeNumber = changeNumber - distance;
            stepNumber++;
        }
    }
}