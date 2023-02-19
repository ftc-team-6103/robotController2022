package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class FieldCentricTeleOp extends LinearOpMode {

    private Claw claw;
    private Servo clawServo;
    private Arm arm;
    private Servo armServo;
    private Lift lift;
    private DcMotor liftMotor;
    private DigitalChannel liftButtonSensor;
    private long toggleTimer = System.currentTimeMillis();

    private static final double THROTTLE = 0.75;
    private static final double STRAFE_THROTTLE = 0.9;
    private static final double ROTATION_THROTTLE = 0.75            ;

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

        Pose2d startPose = new Pose2d(0, 0, 4.71);
        drive.setPoseEstimate(startPose);

        waitForStart();

        while (!isStopRequested()){

            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * THROTTLE,
                            input.getY() * STRAFE_THROTTLE,
                            -gamepad1.right_stick_x * ROTATION_THROTTLE
                    )
            );

            drive.update();

            if (gamepad1.y){
                drive.setPoseEstimate(new Pose2d());
            }

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
            telemetry.addData("Heading", drive.getPoseEstimate().getHeading());
            telemetry.update();
        }
    }
}
