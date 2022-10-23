package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class Claw{

    private double CLOSED_SERVO_POSITION = 0.0;
    private double OPEN_SERVO_POSITION = 0.5;

    private Servo clawServo = null;

    public Claw (Servo clawServo) {
        this.clawServo = clawServo;
    }

    public void close(){
        clawServo.setPosition(CLOSED_SERVO_POSITION);
    }

    public void open(){
        clawServo.setPosition(OPEN_SERVO_POSITION);
    }
}