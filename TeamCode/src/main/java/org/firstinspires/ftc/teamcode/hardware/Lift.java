package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Lift {

    public static int MAX_LIFT_MOTOR_POSITION = 12000;
    public static int MIN_LIFT_MOTOR_POSITION = 0;
    public static int MIN_LIFT_ROTATE_POSITION = 1000;

    private DcMotor liftMotor;

    public Lift (DcMotor liftMotor){
        this.liftMotor = liftMotor;
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
        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        liftMotor.setPower(Math.abs(power));
    }

}
