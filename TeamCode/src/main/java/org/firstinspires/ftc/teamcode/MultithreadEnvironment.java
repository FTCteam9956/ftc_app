package org.firstinspires.ftc.teamcode;
//Test
import com.qualcomm.robotcore.hardware.Servo;
import java.lang.reflect.Type;
import java.util.ArrayList;

public class MultithreadEnvironment implements Runnable{

    public static RRHardwarePresets robot; //Used to gain access to RRHardwarePresets methods.
    public static String functionToExecute;
    public static ArrayList<Type> genericArgs = new ArrayList<>();

    //Constructor that takes in the name of the function we wish to run on a thread.
    public MultithreadEnvironment(String functionName, ArrayList<Type> Args){
        this.functionToExecute = functionName;
        //this.servoArgs = Args;
        this.genericArgs = Args;
    }

    //Runs when we call start() on the thread we create else where.
    public void run(){
        //Names of functions in this class we want to execute.
        if(functionToExecute.equals("theadedExtendArm")){
            threadedExtendArm();
        }
        if(functionToExecute.equals("")){}
        if(functionToExecute.equals("")){}
        if(functionToExecute.equals("")){}
    }

    //Parameters are passed via servoArgs ArrayList, index 0 should be wrist, index 1 should be elbow.
    public void threadedExtendArm(){
        Thread thread = Thread.currentThread();
        if(thread.getName().equals("thread1")){
            System.out.println("thread: " + thread.getName());
            robot.moveServo((Servo)genericArgs.get(0), robot.WRIST_UNFOLDED, 1000, 2000);
        }
        if(thread.getName().equals("thread2")){
            System.out.println("thread: " + thread.getName());
            robot.moveServo((Servo)genericArgs.get(0), robot.ELBOW_UNFOLDED, 1000, 2000);
        }
    }
}
