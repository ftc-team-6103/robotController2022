package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

public class Lift {

    public static int MAX_LIFT_MOTOR_POSITION = 12000;
    public static int MIN_LIFT_MOTOR_POSITION = 0;
    public static int MIN_LIFT_ROTATE_POSITION = 1000;

    public static int LIFT_GROUND_POSITION = -100;
    public static int LIFT_DRIVE_POSITION = -500;
    public static int LIFT_LOW_TERMINAL = -1000;
    public static int LIFT_MID_TERMINAL = -7500;
    public static int LIFT_MID_TERMINAL_RELEASE = -6000;
    public static int LIFT_HIGH_TERMINAL = -3000;

    private static final double AUTONOMOUS_POWER = 1.0;

    private DcMotor liftMotor;
    private DigitalChannel liftButtonSensor;

    public Lift (DcMotor liftMotor){
        this.liftMotor = liftMotor;

        this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public Lift (DcMotor liftMotor, DigitalChannel liftButtonSensor){
        this.liftMotor = liftMotor;
        this.liftButtonSensor = liftButtonSensor;
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
}
