package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@Config
@Disabled
@Autonomous(group = "drive")
public class LeftAutonomousTest extends LinearOpMode {

    private Claw claw;
    private Servo clawServo;
    private Arm arm;
    private Servo armServo;
    private Lift lift;
    private DcMotor liftMotor;

    private Pose2d poseHome = new Pose2d(0,0,0);
    private Pose2d poseBackup = new Pose2d(-55,0,0);
    private Pose2d poseMediumPole = new Pose2d(-43,5, 0.93);
    private Pose2d poseMediumPole2 = new Pose2d(-43,4, 0.93);
    private Pose2d poseConeStack = new Pose2d(-48, -19, 1.6);
//    private Pose2d poseParking = new Pose2d(-29.29, 0, 3.14);


    private Trajectory trajectoryHomeToBackUp = null;
    private Trajectory trajectoryBackUpToPole = null;
    private Trajectory trajectoryPoleToConeStack = null;
    private Trajectory trajectoryConeStackToPole2 = null;

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
        sleep(500);
        lift.adjustUp();
        sleep(500);

        trajectoryHomeToBackUp = drive.trajectoryBuilder(poseHome)
                .lineToLinearHeading(poseBackup)
                .build();

        drive.followTrajectory(trajectoryHomeToBackUp);

        lift.moveToPosition(Lift.POSITION_MID_TERMINAL);

        trajectoryBackUpToPole = drive.trajectoryBuilder(poseBackup)
                .lineToLinearHeading(poseMediumPole)
                .build();

        drive.followTrajectory(trajectoryBackUpToPole);

        sleep(500);

        lift.moveToPosition(Lift.POSITION_MID_TERMINAL);

        sleep(500);

        lift.adjustDown();

        claw.open();

        sleep(500);

        trajectoryPoleToConeStack = drive.trajectoryBuilder(poseMediumPole)
                .lineToLinearHeading(poseConeStack)
                .build();

        drive.followTrajectory(trajectoryPoleToConeStack);

        arm.rotateRear();

        lift.moveToPosition(Lift.LIFT_CONE_STACK);

        sleep(500);

        claw.close();

        sleep(500);

        lift.moveToPosition(Lift.POSITION_MID_TERMINAL);

        arm.rotateForward();

        sleep(500);

        trajectoryConeStackToPole2 = drive.trajectoryBuilder(poseConeStack)
                .lineToLinearHeading(poseMediumPole2)
                .build();

        drive.followTrajectory(trajectoryConeStackToPole2);

        sleep(500);
        lift.adjustDown();
        claw.open();
    }
}
