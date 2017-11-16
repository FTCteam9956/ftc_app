package org.firstinspires.ftc.teamcode;

public class MultithreadEnviroment implements Runnable{

    //Used to gain access to RRHardwarePresets methods.
    public static RRHardwarePresets robot;
    public String functionToExecute;

    //Constructor that takes in the name of the function we wish to run on a thread.
    public MultithreadEnviroment(String functionName){
        this.functionToExecute = functionName;
    }

    //Runs when we call start() on the thread we create else where.
    public void run(){
        //Names of functions in this class we want to execute.
        if(functionToExecute.equals("")){}
        if(functionToExecute.equals("")){}
        if(functionToExecute.equals("")){}
        if(functionToExecute.equals("")){}
    }
}
