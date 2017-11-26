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

    VuforiaLocalizer vuforia;

    public void runOpMode(){
        robot.init(hardwareMap); //Robot moves during init().

        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.setRunMode("RUN_USING_ENCODER");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AU0kxmH/////AAAAGV4QPVzzlk6Hl969cSL2pmM4F6TuzhWZS/dKbY45MEzS31OYJxLbKewdt1CSFrmpvrpPnIYZyBJt3kFRJQCtEXet0LHd2KtBB5NsDTuBADfgIsQk+7TSWSTFDjSi8SpKaXtAjZPKePwGDaIKf5VK6mRBYaWxqTHpZFBlelejLHxib8qweOFrJjKTsbgsb2pwVNFhDeJabbI5aed8JSI8LxHs0368ezQfnCz3UK9u8pC1DkKgcwdgoJ0OXBKChXB4v2lEnIrQf7ROYcPtVuRJJ5/prBoyfR11pvp69iCA25Cttz9xVsdZ9VliuQJ4UO37Hzhz1dB2SPnxTQQmCJMDoDKqe3wpiCFu8ThQ4pmS05ka";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // Use FRONT Camera (Change to BACK if you want to use that one)
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES; // Display Axes

        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        waitForStart();

        robot.initServoPositions();
        relicTrackables.activate(); // Activate Vuforia

        //--AUTO SCRIPT START--

        //Finds out what VuMark we are looking at and returns corresponding int.
        int targetPosition = 0;
        while(targetPosition == 0){
            targetPosition = lookForVuMark(relicTemplate); //1 - LEFT, 2 - RIGHT, 3 - CENTER, 0 - NOT VISIBLE
            sleep(500);
        }

        //Lowers jewel arm into JEWEL_ARM_DOWN position with 1000 steps over 2 seconds.
        robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_DOWN, 1000, 2000);

        //Reads color of ball and calls knockOffBall(0), knockOffBall(1) or does nothing.
        int loopBreak = 0;
        while (loopBreak == 0){
            sleep(1000);
            if(robot.jewelSensor.red() > 52){
                knockOffBall(0);
                telemetry.addData("Status", "Confirmed Red Ball!");
                loopBreak = 1;
            }else if(robot.jewelSensor.red() <= 52){
                if (robot.jewelSensor.blue() > 20){
                    knockOffBall(1);
                    telemetry.addData("Status", "Confirmed Blue Ball!");
                    loopBreak = 1;
                }else{
                    telemetry.addData("Status", "Cannot determine color!");
                    loopBreak = 1;
                }
            }
            telemetry.addData("Jewel Sensor - Red", robot.jewelSensor.red());
            telemetry.addData("Jewel Sensor - Blue", robot.jewelSensor.blue());
            telemetry.update();
        }
        sleep(500);

        //Raises jewel arm into JEWEL_ARM_UP position with 750 steps over 1 second.
        robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_UP, 750, 1000);
        sleep(500);

        //Drive backwards off of the balancing stone to place the block.
        robot.driveForwardSetDistance(0.10, robot.DRIVE_OFF_STONE);
        sleep(500);

        //Drive into the balancing stone to give us a known position
        robot.driveForwardSetDistance(0.10, robot.DRIVE_INTO_STONE);
        sleep(500);

//        //Set position of the turret
//        robot.turretMotor.setTargetPosition(robot.TURRET_FOR_WALL);
//
//        //Opens claw to drop block
//        robot.claw.setPosition(robot.CLAW_OPENED);
//
//        //Move the arm and turret to
//        robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_FOLDED, robot.WRIST_FOLDED, 1000, 2000);
//        robot.turretMotor.setTargetPosition(robot.TURRET_FOR_RELIC);
//
//        robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_RELIC, robot.WRIST_RELIC, 1000, 2000);
    }

    //Looks for VuMark and positions arm accordingly. Returns int based on what it saw for debugging purposes
    public int lookForVuMark(VuforiaTrackable rTemplate){
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(rTemplate);
        int returnValue = -1;
        if(vuMark != RelicRecoveryVuMark.UNKNOWN){
            if(vuMark == RelicRecoveryVuMark.LEFT){ // Test to see if Image is the "LEFT" image and display value.
                telemetry.addData("VuMark is", "Left");
                //robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_LEFT, robot.WRIST_LEFT, 1000, 2000);
                //robot.shoulder.setTargetPosition(0);
                returnValue = 1;
            }else if(vuMark == RelicRecoveryVuMark.RIGHT){ // Test to see if Image is the "RIGHT" image and display values.
                telemetry.addData("VuMark is", "Right");
                //robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_RIGHT, robot.WRIST_RIGHT, 1000, 2000);
                //robot.shoulder.setTargetPosition(0);
                returnValue = 2;
            }else if(vuMark == RelicRecoveryVuMark.CENTER){ // Test to see if Image is the "CENTER" image and display values.
                telemetry.addData("VuMark is", "Center");
                //robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_CENTER, robot.WRIST_CENTER, 1000, 2000);
                //robot.shoulder.setTargetPosition(0);
                returnValue = 3;
            }
        }else{
            telemetry.addData("VuMark", "not visible");
            //robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_CENTER, robot.WRIST_CENTER, 1000, 2000);
            //robot.shoulder.setTargetPosition(0);
            returnValue = 0;
        }
        telemetry.update();
        return(returnValue);
    }
    public void knockOffBall(int selection){
        //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.setRunMode("RUN_TO_POSITION");
        if (selection == 0) {
            robot.turretMotor.setTargetPosition(200);
        }
        if (selection == 1) {
            robot.turretMotor.setTargetPosition(-200);
        }
        robot.turretMotor.setPower(0.15);
        while(robot.turretMotor.isBusy()){
            //Waiting while turret turns.
        }
        sleep(500);
        robot.turretMotor.setPower(0.0);
        robot.setRunMode("RUN_USING_ENCODER");
    }
    String format(OpenGLMatrix transformationMatrix){
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
