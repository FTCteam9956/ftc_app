//RedTurn.java
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "RedTurn", group = "Autonomous")
//@Disabled
public class RedTurn extends LinearOpMode{
    public RRHardwarePresets robot = new RRHardwarePresets();

    public void runOpMode(){
        robot.init(hardwareMap); //Robot moves during init().

        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode("RUN_USING_ENCODER");
        robot.turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.winchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.winchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        robot.initServoPositions();
        robot.relicTrackables.activate(); // Activate Vuforia

        //--AUTO SCRIPT START--

        //Finds out what VuMark we are looking at and returns corresponding int.
        int targetPosition = 0;
        long initTime = (System.nanoTime()/1000000); //Converting Nanoseconds to Milliseconds.
        long timeOutTime = 3000; //In Milliseconds.
        while(targetPosition == 0){
            targetPosition = robot.lookForVuMark(robot.relicTemplate); //1 - LEFT, 2 - RIGHT, 3 - CENTER, 0 - NOT VISIBLE, 4 - TIMEOUT
            if(((System.nanoTime()/1000000) - initTime) > timeOutTime){
                targetPosition = 4;
            }
        }

        //Lowers jewel arm into JEWEL_ARM_DOWN position with 1000 steps over 2 seconds.
        robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_DOWN, 500, 1000);

        //Reads color of ball and calls knockOffBall(0), knockOffBall(1) or does nothing.
        int loopBreak = 0;
        while (loopBreak == 0){
            sleep(1000);
            if(robot.jewelSensor.red() > 52){
                robot.knockOffBall(0);
                telemetry.addData("Status", "Confirmed Red Ball!");
                loopBreak = 1;
            }else if(robot.jewelSensor.red() <= 52){
                if (robot.jewelSensor.blue() > 20){
                    robot.knockOffBall(1);
                    telemetry.addData("Status", "Confirmed Blue Ball!");
                    sleep(300);
                    loopBreak = 1;
                }else{
                    telemetry.addData("Status", "Cannot determine color!");
                    loopBreak = 1;
                }
            }
            telemetry.addData("Jewel Sensor - Red", robot.jewelSensor.red());
            telemetry.addData("Jewel Sensor - Blue", robot.jewelSensor.blue());
            telemetry.addData("CPosition:", robot.turretMotor.getCurrentPosition());
            telemetry.update();
        }
        sleep(500);

        //Raises jewel arm into JEWEL_ARM_UP position with 750 steps over 1 second.
        robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_UP, 500, 660);
        sleep(500);

        //Drive backwards off of the balancing stone to place the block.
        robot.driveForwardSetDistance(0.15, robot.DRIVE_OFF_STONE);
        sleep(500);

        //Drive into the balancing stone to give us a known position
        robot.driveForwardSetDistance(0.15, robot.DRIVE_INTO_STONE);
        sleep(500);

        //Turn Turret X amount degrees
        robot.rotateTurret(0.3, 1835, "CW");
        sleep(500);

        //Raises Winch
        robot.winchMotor.setTargetPosition(300);
        robot.winchMotor.setPower(0.35);

        //1 - LEFT, 2 - RIGHT, 3 - CENTER, 0 - NOT VISIBLE, 4 - TIMEOUT
        if(targetPosition == 1){
            robot.redTurnLeft.execute();
        }
        else if(targetPosition == 2){
            robot.redTurnRight.execute();
        }
        else if(targetPosition == 3){
            robot.redTurnCenter.execute();
        }
        else if(targetPosition == 4){

        }
    }
}
