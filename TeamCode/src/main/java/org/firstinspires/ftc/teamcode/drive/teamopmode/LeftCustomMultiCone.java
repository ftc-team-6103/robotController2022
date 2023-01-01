package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@Config
@Disabled
@Autonomous(group = "drive")
public class LeftCustomMultiCone extends Left {

    private Claw claw;
    private Servo clawServo;
    private Arm arm;
    private Servo armServo;
    private Lift lift;
    private DcMotor liftMotor;

    private Pose2d poseHome = new Pose2d(0,0,0);
    private Pose2d poseBackup = new Pose2d(-55,0,0);
    private Pose2d poseMediumPole = new Pose2d(-43,5, 0.93);
    private Pose2d poseConeStack = new Pose2d(-46, -20, 1.6);

    private Pose2d poseParking1 = new Pose2d(-31.29, -26, 0);
    private Pose2d poseParking2 = new Pose2d(-29.29, 0, 0);
    private Pose2d poseParking3 = new Pose2d(-29.29, 26, 0);

    private Trajectory trajectoryHomeToBackUp = null;
    private Trajectory trajectoryBackUpToPole = null;
    private Trajectory trajectoryPoleToConeStack = null;
    private Trajectory trajectoryConeStackToPole = null;

    private Trajectory trajectoryPoleToParkingPosition1 = null;
    private Trajectory trajectoryPoleToParkingPosition2 = null;
    private Trajectory trajectoryPoleToParkingPosition3 = null;

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

        while (this.getRuntime() < start + 2){
            telemetry.addData("runtime", this.getRuntime());
            telemetry.update();
            randomization = detectSignal();
        }

        telemetry.addData("randomization", "detected: " + randomization);
        telemetry.update();

        claw.close();
        sleep(1000);
        lift.adjustUp();

        trajectoryHomeToBackUp = drive.trajectoryBuilder(poseHome)
                .lineToLinearHeading(poseBackup)
                .build();

        drive.followTrajectory(trajectoryHomeToBackUp);

        arm.rotateForward();

        lift.moveToPosition(Lift.POSITION_MID_TERMINAL);

        trajectoryBackUpToPole = drive.trajectoryBuilder(poseBackup)
                .lineToLinearHeading(poseMediumPole)
                .build();

        drive.followTrajectory(trajectoryBackUpToPole);
        sleep(1000);
        lift.moveToPosition(Lift.POSITION_MID_TERMINAL + 350);
        sleep(1000);
        claw.open();
        sleep(1000);
        trajectoryPoleToConeStack = drive.trajectoryBuilder(poseMediumPole)
                .lineToLinearHeading(poseConeStack)
                .build();

        drive.followTrajectory(trajectoryPoleToConeStack);

        arm.rotateRear();

        sleep(500);

        lift.moveToPosition(Lift.LIFT_CONE_STACK);

        sleep(500);
        claw.close();
        sleep(500);
        lift.moveToPositionAsync(Lift.POSITION_MID_TERMINAL);

        trajectoryConeStackToPole = drive.trajectoryBuilder(poseConeStack)
                .lineToLinearHeading(poseMediumPole)
                .build();
        drive.followTrajectory(trajectoryConeStackToPole);

        arm.rotateForward();
        sleep(1000);
        lift.moveToPosition(Lift.POSITION_MID_TERMINAL + 350);
        claw.open();



//        driveToParkingPosition(drive);
//        lift.moveToPosition(Lift.POSITION_GROUND);
    }

    private void driveToParkingPosition(SampleMecanumDrive drive){
        trajectoryPoleToParkingPosition1 = drive.trajectoryBuilder(poseMediumPole)
                .lineToLinearHeading(poseParking1)
                .build();
        trajectoryPoleToParkingPosition2 = drive.trajectoryBuilder(poseMediumPole)
                .lineToLinearHeading(poseParking2)
                .build();
        trajectoryPoleToParkingPosition3 = drive.trajectoryBuilder(poseMediumPole)
                .lineToLinearHeading(poseParking3)
                .build();

        if (randomization.equals("Lightning")){
            drive.followTrajectory(trajectoryPoleToParkingPosition2);
        }
        else if (randomization.equals("Laser")){
            drive.followTrajectory(trajectoryPoleToParkingPosition1);
        }
        else{
            drive.followTrajectory(trajectoryPoleToParkingPosition3);
        }

    }
}
