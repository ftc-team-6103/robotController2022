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
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Config
@Autonomous(group = "drive")
public class Right_SQ extends Left {

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

        double start = this.getRuntime();

        while (this.getRuntime() < start + 0.5){
            telemetry.addData("runtime", this.getRuntime());
            telemetry.update();
            randomization = detectSignal();
        }

        telemetry.addData("randomization", "detected: " + randomization);
        telemetry.update();

        claw.close();
        sleep(500);
        arm.rotateForward();
        lift.moveToPositionAsync(Lift.POSITION_MID_TERMINAL);


        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(39,
                        VELOCITY_CONSTRAINT,
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .build();

        drive.followTrajectorySequence(trajectory);
        arm.rotateRear();
        sleep(1500);
        drive.setPoseEstimate(new Pose2d());
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d())
                .forward(2.5)
                .build());
        lift.moveToPosition(Lift.POSITION_MID_TERMINAL + 500);
        sleep(500);
        claw.open();
        lift.adjustUp();
        sleep(500);
        arm.rotateForward();
        drive.setPoseEstimate(new Pose2d());
        drive.followTrajectorySequence(drive.trajectorySequenceBuilder(new Pose2d())
                .back(2.5)
                .build());
        lift.moveToPositionAsync(0);
        drive.setPoseEstimate(new Pose2d());

        TrajectorySequence trajectory2 = drive.trajectorySequenceBuilder(new Pose2d())
                .strafeLeft(11.5,
                        VELOCITY_CONSTRAINT,
                        ACCELERATION_CONSTRAINT)
                .build();

        drive.followTrajectorySequence(trajectory2);

        drive.setPoseEstimate(new Pose2d());

        sleep(500);
        driveToParkingPosition(drive);
    }

    private void driveToParkingPosition(SampleMecanumDrive drive){

        drive.setPoseEstimate(new Pose2d());

        TrajectorySequence parking1 = drive.trajectorySequenceBuilder(new Pose2d())
                .back(24,
                        VELOCITY_CONSTRAINT,
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL / 2))
                .build();

//        TrajectorySequence parking2 = drive.trajectorySequenceBuilder(new Pose2d())
//                .back(22.5,
//                        VELOCITY_CONSTRAINT,
//                        ACCELERATION_CONSTRAINT)
//                .build();

        TrajectorySequence parking3 = drive.trajectorySequenceBuilder(new Pose2d())
                .forward(24,
                        VELOCITY_CONSTRAINT,
                        ACCELERATION_CONSTRAINT)
                .build();

        if (randomization.equals("Lightning")){
            //Do nothing
        }
        else if (randomization.equals("Laser")){
            drive.followTrajectorySequence(parking1);
        }
        else{
            drive.followTrajectorySequence(parking3);
        }

    }
}
