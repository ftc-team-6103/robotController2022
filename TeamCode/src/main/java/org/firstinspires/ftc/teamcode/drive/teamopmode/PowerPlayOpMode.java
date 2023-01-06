package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class PowerPlayOpMode extends LinearOpMode {

    private Claw claw;
    private Servo clawServo;
    private Arm arm;
    private Servo armServo;
    private Lift lift;
    private DcMotor liftMotor;
    private DigitalChannel liftButtonSensor;
    private long toggleTimer = System.currentTimeMillis();

    private static final double THROTTLE = 0.5;
    private static final double STRAFE_THROTTLE = 0.5;
    private static final double ROTATION_THROTTLE = 0.5;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        claw = new Claw(clawServo);
        armServo = hardwareMap.get(Servo.class, "gate_servo");
        arm = new Arm(armServo);
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        liftButtonSensor = hardwareMap.get(DigitalChannel.class, "lift_button_sensor");
        lift = new Lift(liftMotor, liftButtonSensor, false);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()){
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * THROTTLE,
                            -gamepad1.left_stick_x * STRAFE_THROTTLE,
                            -gamepad1.right_stick_x * ROTATION_THROTTLE
                    )
            );

            drive.update();

            if (gamepad2.a){
                claw.close();
            }
            else if (gamepad2.b){
                claw.open();
            }

            if (gamepad2.x){
                arm.rotateForward();
            }
            else if (gamepad2.y){
                arm.rotateRear();
            }

            if(gamepad2.left_stick_y < 0){
                lift.raise(gamepad2.left_stick_y);
            }
            else if (gamepad2.left_stick_y > 0){
                lift.lower(gamepad2.left_stick_y);
            }
            else{
                lift.stop();
            }

            telemetry.addData("Lift Motor Position: ", liftMotor.getCurrentPosition());
            telemetry.addData("Lift Position: ", lift.getCurrentPosition());
            telemetry.addData("Lift Button Sensor: ", liftButtonSensor.getState());
            telemetry.update();
        }
    }
}
