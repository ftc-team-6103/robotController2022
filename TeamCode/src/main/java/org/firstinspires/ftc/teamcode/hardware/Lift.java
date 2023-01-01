package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.checkerframework.checker.units.qual.A;

public class Lift {

    public static int LIFT_CONE_STACK = -300;

    public static int POSITION_GROUND = 0;
    public static int POSITION_LOW_TERMINAL = -1800;
    public static int POSITION_MID_TERMINAL = -3000;
    public static int POSITION_HIGH_TERMINAL = -4200;

    private static final int ADJUSTMENT = 200;

    private static final double AUTONOMOUS_POWER = 1.0;

    private DcMotor liftMotor;
    private DigitalChannel liftButtonSensor;
    private int currentPosition = POSITION_GROUND;

    public Lift (DcMotor liftMotor){
        this.liftMotor = liftMotor;

        this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Lift (DcMotor liftMotor, DigitalChannel liftButtonSensor){
        this.liftMotor = liftMotor;
        this.liftButtonSensor = liftButtonSensor;
        this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Lift (DcMotor liftMotor, DigitalChannel liftButtonSensor, boolean encoderBased){
        this.liftMotor = liftMotor;
        this.liftButtonSensor = liftButtonSensor;
        if (encoderBased){
            this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void stop(){
        liftMotor.setPower(0);
    }

    public void raise(double power){
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setPower(Math.abs(power));
    }

    public void lower (double power){
        if (!liftButtonSensor.getState()){
            stop();
            return;
        }

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setPower(Math.abs(power));
    }

    public void moveToPosition(int targetPosition){
        liftMotor.setTargetPosition(targetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(AUTONOMOUS_POWER);

        while (liftMotor.isBusy()){
            //wait for lift motor to move to position
        }
    }

    public void togglePositionUp(){
        if (currentPosition == POSITION_GROUND){
            currentPosition = POSITION_LOW_TERMINAL;
        }
        else if (currentPosition == POSITION_LOW_TERMINAL){
            currentPosition = POSITION_MID_TERMINAL;
        }
        else{
            currentPosition = POSITION_HIGH_TERMINAL;
        }

        moveToPositionAsync(currentPosition);
    }

    public void togglePositionDown(){
        if (currentPosition == POSITION_HIGH_TERMINAL){
            currentPosition = POSITION_MID_TERMINAL;
        }
        else if (currentPosition == POSITION_MID_TERMINAL){
            currentPosition = POSITION_LOW_TERMINAL;
        }
        else{
            currentPosition = POSITION_GROUND;
        }

        moveToPositionAsync(currentPosition);
    }

    /**
     * Make Fine Tune Movement Down by ADJUSTMENT amount
     */
    public void adjustDownAsync(){
        int encoderValue = liftMotor.getCurrentPosition();
        if (currentPosition != POSITION_GROUND && encoderValue + ADJUSTMENT < POSITION_GROUND){
            moveToPositionAsync(encoderValue + ADJUSTMENT, 0.25);
        }
    }

    public void adjustDown(){
        int encoderValue = liftMotor.getCurrentPosition();
        if (currentPosition != POSITION_GROUND && encoderValue + ADJUSTMENT < POSITION_GROUND){
            moveToPositionAsync(encoderValue + ADJUSTMENT, 0.25);
        }

        while (liftMotor.isBusy()){
            //wait for lift motor to move to position
        }
    }

    /**
     * Make Fine Tune Movement Up by ADJUSTMENT amount
     */
    public void adjustUpAsync(){
        int encoderValue = liftMotor.getCurrentPosition();
        if (currentPosition != POSITION_HIGH_TERMINAL && encoderValue - ADJUSTMENT > POSITION_HIGH_TERMINAL){
            moveToPositionAsync(encoderValue - ADJUSTMENT, 0.25);
        }
    }

    public void adjustUp(){
        int encoderValue = liftMotor.getCurrentPosition();
        if (currentPosition != POSITION_HIGH_TERMINAL && encoderValue - ADJUSTMENT > POSITION_HIGH_TERMINAL){
            moveToPositionAsync(encoderValue - ADJUSTMENT, 0.25);
        }

        while (liftMotor.isBusy()){
            //wait for lift motor to move to position
        }
    }

    private void calibrateGround(){
        lower(0.1);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveToPositionAsync(int targetPosition){
        liftMotor.setTargetPosition(targetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(AUTONOMOUS_POWER);
    }

    private void moveToPositionAsync(int targetPosition, double power){
        liftMotor.setTargetPosition(targetPosition);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);
    }

    public String getCurrentPosition(){
        if (currentPosition == POSITION_GROUND){
            return "GROUND " + currentPosition;
        }
        else if (currentPosition == POSITION_LOW_TERMINAL){
            return "LOW TERMINAL " + currentPosition;
        }
        else if (currentPosition == POSITION_MID_TERMINAL){
            return "MID TERMINAL " + currentPosition;
        }
        else{
            return "HIGH TERMINAL " + currentPosition;
        }
    }
}
