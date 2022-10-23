package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private double ARM_POSITION_FORWARD = 0.0;
    private double ARM_POSITION_REAR = 1.0;

    private Servo armServo;

    public Arm(Servo armServo){
        this.armServo = armServo;
    }

    public void rotateForward(){
        armServo.setPosition(ARM_POSITION_FORWARD);
    }

    public void rotateRear(){
        armServo.setPosition(ARM_POSITION_REAR);
    }
}
