package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@Config
@Autonomous(group = "drive")
public class RightAutonomous extends LinearOpMode {

    private Claw claw;
    private Servo clawServo;
    private Arm arm;
    private Servo armServo;
    private Lift lift;
    private DcMotor liftMotor;

    private Pose2d poseHome = new Pose2d(0,0,0);
    private Pose2d poseBackup = new Pose2d(-33.15,0,0);
    private Pose2d poseMediumPole = new Pose2d(-25.79,-2.77, 0.65);
    private Pose2d poseParking = new Pose2d(-25.79, 0, 3.14);


    private Trajectory trajectoryHomeToBackUp = null;
    private Trajectory trajectoryBackUpToPole = null;
    private Trajectory trajectoryPoleToParkingPosition = null;



    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        claw = new Claw(clawServo);
        armServo = hardwareMap.get(Servo.class, "gate_servo");
        arm = new Arm(armServo);
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        lift = new Lift(liftMotor);

        waitForStart();

        claw.close();
        sleep(1000);
        lift.moveToPosition(Lift.LIFT_DRIVE_POSITION);
        sleep(1000);
        driveToPole(drive, lift);
        sleep(1000);
        arm.rotateRear();
        sleep(2000);
        lift.moveToPosition(Lift.LIFT_MID_TERMINAL_RELEASE);
        claw.open();
        sleep(1000);
        arm.rotateForward();
        lift.moveToPosition(Lift.LIFT_DRIVE_POSITION);
        driveToParkingPosition(drive);

    }

    private void driveToParkingPosition(SampleMecanumDrive drive){
        trajectoryPoleToParkingPosition = drive.trajectoryBuilder(poseMediumPole)
                .lineToLinearHeading(poseParking)
                .build();

        drive.followTrajectory(trajectoryPoleToParkingPosition);
    }

    private void driveToPole(SampleMecanumDrive drive, Lift lift){
        trajectoryHomeToBackUp = drive.trajectoryBuilder(poseHome)
                .lineToLinearHeading(poseBackup)
                .build();

        trajectoryBackUpToPole = drive.trajectoryBuilder(poseBackup)
                .lineToLinearHeading(poseMediumPole)
                .build();

        drive.followTrajectory(trajectoryHomeToBackUp);
        lift.moveToPosition(Lift.LIFT_MID_TERMINAL);
        drive.followTrajectory(trajectoryBackUpToPole);
    }

}
