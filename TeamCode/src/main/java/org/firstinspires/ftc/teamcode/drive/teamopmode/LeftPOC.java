package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "drive")
public class LeftPOC extends Left {

    public static final TrajectoryAccelerationConstraint ACCELERATION_CONSTRAINT = SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2);
    public static final TrajectoryVelocityConstraint VELOCITY_CONSTRAINT = SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);

    private Claw claw;
    private Servo clawServo;
    private Arm arm;
    private Servo armServo;
    private Lift lift;
    private DcMotor liftMotor;


    private String randomization = "";

    @Override
    public void runOpMode() throws InterruptedException{

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        claw = new Claw(clawServo);
        armServo = hardwareMap.get(Servo.class, "gate_servo");
        arm = new Arm(armServo);
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        lift = new Lift(liftMotor);

        initCustomTensorFlow();

        waitForStart();

//        double start = this.getRuntime();
//
//        while (this.getRuntime() < start + 0.5){
//            telemetry.addData("runtime", this.getRuntime());
//            telemetry.update();
//            randomization = detectSignal();
//        }
//
//        telemetry.addData("randomization", "detected: " + randomization);
//        telemetry.update();
//
//        claw.close();
//        sleep(250);
//        lift.adjustUp();

        claw.close();
        sleep(500);
        arm.rotateForward();
        lift.moveToPositionAsync(Lift.POSITION_MID_TERMINAL);


        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(40,
                        VELOCITY_CONSTRAINT,
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .forward(5.25,
                        VELOCITY_CONSTRAINT,
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .build();

        drive.followTrajectorySequence(trajectory);

        claw.open();
        sleep(250);
        arm.rotateRear();

        drive.setPoseEstimate(new Pose2d());

        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(new Pose2d())
                .back(5.25,
                        VELOCITY_CONSTRAINT,
                        ACCELERATION_CONSTRAINT)
                .strafeLeft(8,
                        VELOCITY_CONSTRAINT,
                        ACCELERATION_CONSTRAINT)
                .build();

        drive.followTrajectorySequence(trajectory2);

        lift.moveToPositionAsync(Lift.LIFT_CONE_STACK);

        drive.setPoseEstimate(new Pose2d());

        TrajectorySequence trajectory3 = drive.trajectorySequenceBuilder(new Pose2d())
                .back(17.75,
                        VELOCITY_CONSTRAINT,
                        ACCELERATION_CONSTRAINT)
                .build();

        drive.followTrajectorySequence(trajectory3);

        claw.close();
        sleep(250);
        lift.moveToPositionAsync(Lift.POSITION_MID_TERMINAL);
        sleep(500);
        arm.rotateForward();

        drive.setPoseEstimate(new Pose2d());

        TrajectorySequence trajectory4 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(16.75,
                        VELOCITY_CONSTRAINT,
                        ACCELERATION_CONSTRAINT)
                .strafeRight(9,
                        VELOCITY_CONSTRAINT,
                        ACCELERATION_CONSTRAINT)
                .forward(5.25,
                        VELOCITY_CONSTRAINT,
                        ACCELERATION_CONSTRAINT)
                .build();

        drive.followTrajectorySequence(trajectory4);

        lift.adjustDown();
        claw.open();
    }
}
