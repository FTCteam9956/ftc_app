package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

@Autonomous(name = "RedTurn", group = "Autonomous")
//@Disabled

public class RedTurn extends LinearOpMode {
    public RRHardwarePresets robot = new RRHardwarePresets();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap); //Robot moves during init().

        robot.setRunMode("STOP_AND_RESET_ENCODER");
        robot.setRunMode("RUN_USING_ENCODER");

        //Vuforia Trackables.
        VuforiaTrackables relicTrackables = robot.vuforia.loadTrackablesFromAsset("RelicVuMark"); //I believe this loads VuMark data from the assets folder in FtcRobotController.
        VuforiaTrackable relicTemplate = relicTrackables.get(0);

        waitForStart();

        //Relic Trackables
        relicTrackables.activate();

        boolean testArea = true; //CHANGE THIS BOOLEAN TO RUN TEST AREA. PUT IN SO WE DON'T HAVE TO RUN ENTIRE SCRIPT TO TEST.

        if (testArea == true) {

            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

            while (opModeIsActive()) {
                if (vuMark == RelicRecoveryVuMark.UNKNOWN) { //Test to see if image is visible
                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));

                    telemetry.addData("VuMark", "%s visible", vuMark);


                    //Break apart the pose into transitive and rotational componenets
                    if (pose != null) {
                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        //Extract XYZ values of the offset of the target in relation to our robot
                        robot.tX = trans.get(0);
                        robot.tY = trans.get(0);
                        robot.tZ = trans.get(0);
                        //Extract rotational components of the target in relation to our robot
                        robot.rX = rot.firstAngle;
                        robot.rY = rot.secondAngle;
                        robot.rZ = rot.thirdAngle;
                    }

                    //Waiting for turret to stop moving.
                    if (vuMark == RelicRecoveryVuMark.LEFT) {//Left seen.
                        telemetry.addData("vuMark", "% visible", vuMark);
                        telemetry.addData("X =", robot.tX);
                        telemetry.addData("Y =", robot.tY);
                        telemetry.addData("Z =", robot.tZ);
                    }
                    if (vuMark == RelicRecoveryVuMark.CENTER) { //Center seen.
                        telemetry.addData("vuMark", "% visible", vuMark);
                        telemetry.addData("X =", robot.tX);
                        telemetry.addData("Y =", robot.tY);
                        telemetry.addData("Z =", robot.tZ);
                    }
                    if(vuMark == RelicRecoveryVuMark.RIGHT){ //Right seen.
                        telemetry.addData("vuMark", "% visible", vuMark);
                        telemetry.addData("X =", robot.tX);
                        telemetry.addData("Y =", robot.tY);
                        telemetry.addData("Z =", robot.tZ);
                    }
                }else{
                        telemetry.addData("vuMark is", "not visible");
                    }
                    telemetry.update();

            }
        }
                else {
                    //--AUTO SCRIPT START--

                    //Lowers jewel arm into JEWEL_ARM_DOWN position with 1000 steps over 2 seconds.
                    robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_DOWN, 1000, 2000);

                    //Reads color of ball and calls knockOffBall(0), knockOffBall(1) or does nothing.
                    int loopBreak = 0;
                    while (loopBreak == 0) {
                        sleep(1000);
                        if (robot.jewelSensor.red() > 52) {
                            knockOffBall(1);
                            telemetry.addData("Status", "Confirmed Red Ball!");
                            loopBreak = 1;
                        } else if (robot.jewelSensor.red() <= 52) {
                            if (robot.jewelSensor.blue() > 20) {
                                knockOffBall(0);
                                telemetry.addData("Status", "Confirmed Blue Ball!");
                                loopBreak = 1;
                            } else {
                                telemetry.addData("Status", "Cannot determine color!");
                                loopBreak = 1;
                            }
                        }
                        telemetry.addData("Jewel Sensor - Red", robot.jewelSensor.red());
                        telemetry.addData("Jewel Sensor - Blue", robot.jewelSensor.blue());
                        telemetry.update();
                    }
                    sleep(500);

                    //Raises jewel arm into JEWEL_ARM_UP position with 1000 steps over 2 seconds.
                    robot.moveServo(robot.jewelArm, robot.JEWEL_ARM_UP, 1000, 2000);
                    sleep(500);

                    //Search for and confirm VuMark.relicTemplate);
                    String targetPosition = scanForVuMark(relicTemplate);

                    //Drive backwards off of the balancing stone to place the block.
                    robot.driveForwardSetDistance(0.15, robot.DRIVE_OFF_STONE);
                    sleep(500);

                    //Drive into the balancing stone to give us a known position
                    robot.driveForwardSetDistance(0.15, robot.DRIVE_INTO_STONE);
                    sleep(500);

                    //Set position of the turret
                    robot.turretMotor.setTargetPosition(robot.TURRET_FOR_WALL);

                    //Place the block according to the vuMark results.
                    if (targetPosition.equals("left")) {
                        robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_LEFT, robot.WRIST_LEFT, 1000, 2000);
                        robot.shoulder.setTargetPosition(0);
                    }
                    if (targetPosition.equals("right")) {
                        robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_RIGHT, robot.WRIST_RIGHT, 1000, 2000);
                        robot.shoulder.setTargetPosition(0);
                    }
                    if (targetPosition.equals("center")) {
                        robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_CENTER, robot.WRIST_CENTER, 1000, 2000);
                        robot.shoulder.setTargetPosition(0);
                    }
                    if (targetPosition.equals("gone forever")) {
                        robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_CENTER, robot.WRIST_CENTER, 1000, 2000);
                        robot.shoulder.setTargetPosition(0);
                    }
                    robot.claw.setPosition(robot.CLAW_OPENED);

                    //Move the arm and turret to
                    robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_FOLDED, robot.WRIST_FOLDED, 1000, 2000);
                    robot.turretMotor.setTargetPosition(robot.TURRET_FOR_RELIC);

                    robot.moveMultipleServo(robot.elbow, robot.wrist, robot.ELBOW_RELIC, robot.WRIST_RELIC, 1000, 2000);
                }
            }

            //---AUTONOMOUS ONLY METHODS BELOW---

            //Moves the jewelArm depending on what the input is.
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

            //NEEDS TO BE TESTED.
            //Scans for VuForia Target. returns a string of either "none", "right", "left", or "center"
            //power sets scanning speed, distance sets range of "scan", relicTemplate is the VuforiaTrackable we are looking for.
            public String scanForVuMark(VuforiaTrackable relicTemp) {
                //Resets encoders by setting to STOP_AND_RESET_ENCODER mode.
                robot.setRunMode("STOP_AND_RESET_ENCODER");
                robot.setRunMode("RUN_TO_POSITION");
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemp);
                String decidingMark = "none"; //Used to return String

                while (opModeIsActive()) {
                    if (vuMark == RelicRecoveryVuMark.UNKNOWN) { //Test to see if image is visible
                        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemp.getListener()).getPose();
                        //telemetry.addData("Pose", format(pose));

                        //Break apart the pose into transitive and rotational componenets
                        if (pose != null) {
                            VectorF trans = pose.getTranslation();
                            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                            //Extract XYZ values of the offset of the target in relation to our robot
                            robot.tX = trans.get(0);
                            robot.tY = trans.get(0);
                            robot.tZ = trans.get(0);
                            //Extract rotational components of the target in relation to our robot
                            robot.rX = rot.firstAngle;
                            robot.rY = rot.secondAngle;
                            robot.rZ = rot.thirdAngle;
                        }
                    }
                    //Waiting for turret to stop moving.
                        if(vuMark == RelicRecoveryVuMark.LEFT) {//Left seen.
                            telemetry.addData("vuMark if", "Left");
                            telemetry.addData("X =", robot.tX);
                            telemetry.addData("Y =", robot.tY);
                            telemetry.addData("Z =", robot.tZ);
                            decidingMark = "left";
                        }if(vuMark == RelicRecoveryVuMark.CENTER) { //Center seen.
                            telemetry.addData("vuMark is", "Center");
                            telemetry.addData("X =", robot.tX);
                            telemetry.addData("Y =", robot.tY);
                            telemetry.addData("Z =", robot.tZ);
                            decidingMark = "center";
                        }if(vuMark == RelicRecoveryVuMark.RIGHT) { //Right seen.
                            telemetry.addData("vuMark is", "Right");
                            telemetry.addData("X =", robot.tX);
                            telemetry.addData("Y =", robot.tY);
                            telemetry.addData("Z =", robot.tZ);
                            decidingMark = "right";
                        }else{
                            telemetry.addData("vuMark is", "not visible");
                            decidingMark = "gone forever";
                        }
                        telemetry.addData("Status", decidingMark);
                        telemetry.update();
                    }
                return(decidingMark);
            }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
