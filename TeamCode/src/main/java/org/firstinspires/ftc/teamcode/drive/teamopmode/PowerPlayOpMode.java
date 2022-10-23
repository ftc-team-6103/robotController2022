package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class PowerPlayOpMode extends LinearOpMode {

    Claw claw;
    Servo clawServo;
    Arm arm;
    Servo armServo;
    Lift lift;
    DcMotor liftMotor;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        claw = new Claw(clawServo);
        armServo = hardwareMap.get(Servo.class, "gate_servo");
        arm = new Arm(armServo);
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        lift = new Lift(liftMotor);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            if (gamepad2.a){
                claw.open();
            }
            else if (gamepad2.b){
                claw.close();
            }

            if (gamepad2.x && liftMotor.getCurrentPosition() > Lift.MIN_LIFT_ROTATE_POSITION){
                arm.rotateForward();
            }
            else if (gamepad2.y && liftMotor.getCurrentPosition() > Lift.MIN_LIFT_ROTATE_POSITION){
                arm.rotateRear();
            }

            if(gamepad2.right_stick_y < 0 && liftMotor.getCurrentPosition() < Lift.MAX_LIFT_MOTOR_POSITION){
                lift.raise(gamepad2.right_stick_y);
            }
            else if (gamepad2.right_stick_y > 0 && Math.abs(liftMotor.getCurrentPosition()) > Lift.MIN_LIFT_MOTOR_POSITION){
                lift.lower(gamepad2.right_stick_y);
            }
            else{
                lift.stop();
            }

            telemetry.addData("Right Stick Y: ", gamepad2.right_stick_y);
            telemetry.addData("Lift Motor Position: ", liftMotor.getCurrentPosition());
            telemetry.update();
        }

    }
}
