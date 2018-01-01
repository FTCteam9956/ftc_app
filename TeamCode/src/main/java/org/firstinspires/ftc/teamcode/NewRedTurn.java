package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@TeleOp(name = "NewRedTurn", group = "Autonomous")
//@Disabled

public class NewRedTurn extends LinearOpMode{
    public GrantsTeleopHardware robot = new GrantsTeleopHardware();

    VuforiaLocalizer vuforia;

    public final static int FIRST_DISTANCE =  1000;
    public final static int SECOND_DISTANCE = 2000;
    public final static int BACKUP = -500;

    public final static int SHOULDER_POS1 = 80;
    public final static int SHOULDER_POS2 = 76;
    public final static int SHOULDER_POS3 = 72;

    public final static int TURN1 = 1200;
    public final static int TURN2 = 1100;
    public final static int TURN3 = 1000;

    public void runOpMode(){
        robot.init(hardwareMap); //Robot moves during init().
        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.winch.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode("RUN_USING_ENCODER");
        robot.shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.winch.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        relicTrackables.activate();// Activate Vuforia

        int targetPosition = 0;
        long initTime = (System.nanoTime() / 1000000); //Converting Nanoseconds to Milliseconds.
        long timeOutTime = 3000; //In Milliseconds.
        while (targetPosition == 0) {
            sleep(500);
            targetPosition = lookForVuMark(relicTemplate);//1 - LEFT, 2 - RIGHT, 3 - CENTER, 0 - NOT VISIBLE, 4 - TIMEOUT
            sleep(500);
            if (((System.nanoTime() / 1000000) - initTime) > timeOutTime) {
                targetPosition = 4;
            }
        }

        //robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_DOWN, 500, 1000);

        int loopBreak = 0;
        while (loopBreak == 0) {
            sleep(1000);
            if (robot.jewelArm.red() > 52) {
                knockOffBall(0);
                telemetry.addData("Status", "Confirmed Red Ball!");
                loopBreak = 1;
            } else if (robot.jewelArm.red() <= 52) {
                if (robot.jewelArm.blue() > 20) {
                    knockOffBall(1);
                    telemetry.addData("Status", "Confirmed Blue Ball!");
                    loopBreak = 1;
                } else {
                    telemetry.addData("Status", "Cannot determine color!");
                    loopBreak = 1;
                }
            }
            telemetry.addData("Jewel Sensor - Red", robot.jewelArm.red());
            telemetry.addData("Jewel Sensor - Blue", robot.jewelArm.blue());
            telemetry.addData("CPosition:", robot.rotateArm);
            telemetry.update();
        }
        sleep(500);

        //robot.moveServo(robot.lowerArm, robot.JEWEL_ARM_UP, 500, 1000);
        sleep(500);

        robot.driveForwardSetDistance(0.2, FIRST_DISTANCE);
        sleep(5000);

        //LEFT = 1: RIGHT = 2: CENTER = 3: NOT READ = 4
        if (targetPosition == 1){
            robot.shoulder.setTargetPosition(SHOULDER_POS1);
            sleep(5000);
            robot.turnDirection(0.2, TURN1, "CCW");
            sleep(5000);

        } else if(targetPosition == 2){
            robot.shoulder.setTargetPosition(SHOULDER_POS2);
            sleep(5000);
            robot.turnDirection(0.2, TURN2, "CCW");
            sleep(5000);

        } else if (targetPosition == 3) {
            robot.shoulder.setTargetPosition(SHOULDER_POS3);
            sleep(5000);
            robot.turnDirection(0.2, TURN3, "CCW");
            sleep(5000);

        } else if (targetPosition == 4) {
            robot.shoulder.setTargetPosition(SHOULDER_POS3);
            sleep(5000);
            robot.turnDirection(0.2, TURN3, "CCW");
            sleep(5000);
        }

        robot.driveForwardSetDistance(0.3, SECOND_DISTANCE);
        sleep(5000);
        robot.clawBottom.setPosition(robot.BLOCK_CLAW_OPEN_BOTTOM);
        robot.clawTop.setPosition(robot.BLOCK_CLAW_OPEN_TOP);
        sleep(5000);
        robot.driveForwardSetDistance(0.3, BACKUP);
        sleep(500);
    }

    public int lookForVuMark(VuforiaTrackable rTemplate){
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(rTemplate);
        int returnValue = 0;
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
            returnValue = 4;
        }
        telemetry.update();
        return(returnValue);
    }
    public void knockOffBall(int selection){

        if (selection == 0) {
            robot.rotateArm.setPosition(robot.ROTATE_RIGHT);
        }
        if (selection == 1) {
           robot.rotateArm.setPosition(robot.ROTATE_LEFT);
        }
        sleep(100);
        robot.rotateArm.setPosition(robot.ROTATE_MID);
    }
    String format(OpenGLMatrix transformationMatrix){
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
