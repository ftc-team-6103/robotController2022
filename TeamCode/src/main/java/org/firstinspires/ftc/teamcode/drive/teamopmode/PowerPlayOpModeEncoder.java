package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
@Disabled
public class PowerPlayOpModeEncoder extends LinearOpMode {

    private Claw claw;
    private Servo clawServo;
    private Arm arm;
    private Servo armServo;
    private Lift lift;
    private DcMotor liftMotor;
    private DigitalChannel liftButtonSensor;
    private long toggleTimer = System.currentTimeMillis();

    private static final double THROTTLE = 0.4;
    private static final double STRAFE_THROTTLE = 0.4;
    private static final double ROTATION_THROTTLE = 0.4;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        claw = new Claw(clawServo);
        armServo = hardwareMap.get(Servo.class, "gate_servo");
        arm = new Arm(armServo);
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        liftButtonSensor = hardwareMap.get(DigitalChannel.class, "lift_button_sensor");
        lift = new Lift(liftMotor, liftButtonSensor);

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

            if (gamepad2.dpad_up && canToggle()){
                lift.togglePositionUp();
            }
            else if (gamepad2.dpad_down && canToggle()){
                lift.togglePositionDown();
                if (!liftButtonSensor.getState()){
                    liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }

            if (gamepad2.left_trigger > 0.25 && canToggle()){
                lift.adjustDownAsync();
            }

            if (gamepad2.left_bumper && canToggle()){
                lift.adjustUpAsync();
            }

            if (gamepad2.right_trigger > 0.5 && canToggle()){
                calibrateGroundManual();
            }

//            calibrateGround();

            telemetry.addData("Lift Motor Position: ", liftMotor.getCurrentPosition());
            telemetry.addData("Lift Position: ", lift.getCurrentPosition());
            telemetry.addData("Lift Button Sensor: ", liftButtonSensor.getState());
            telemetry.update();
        }

    }

    /**
     * Ensure a minimum amount of time between consecutive button presses
     */
    private boolean canToggle(){
        long time = System.currentTimeMillis();

        if (time - toggleTimer > 250){
            toggleTimer = time;
            return true;
        }

        return false;
    }

    /**
     * If the button sensor is not pressed and the arm thinks it's all the way lower,
     * manually lower it until the button sensor is pressed and then reset encoder
     */
    private void calibrateGround(){
//        if (liftButtonSensor.getState() && lift.getCurrentPosition().equals("GROUND 0") && Math.abs(liftMotor.getCurrentPosition()) <= 5){
//            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            long start = System.currentTimeMillis();
//            long current = start;
//            while (liftButtonSensor.getState() && current - start < 250){
//                current = System.currentTimeMillis();
//                lift.lower(0.2);
//            }
//            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
    }

    /**
     * If the button sensor is not pressed and the arm thinks it's all the way lower,
     * manually lower it until the button sensor is pressed and then reset encoder
     */
    private void calibrateGroundManual(){
        if (liftButtonSensor.getState() && lift.getCurrentPosition().equals("GROUND 0") && Math.abs(liftMotor.getCurrentPosition()) <= 5){
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            long start = System.currentTimeMillis();
            long current = start;
            while (liftButtonSensor.getState() && current - start < 500){
                current = System.currentTimeMillis();
                lift.lower(0.2);
            }
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
