package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Lift;

@Config
@Autonomous(group = "drive")
public class RightCustom extends Right {

    private Claw claw;
    private Servo clawServo;
    private Arm arm;
    private Servo armServo;
    private Lift lift;
    private DcMotor liftMotor;

    private Pose2d poseHome = new Pose2d(0,0,0);
    private Pose2d poseBackup = new Pose2d(-33.15,0,0);
    private Pose2d poseMediumPole = new Pose2d(-25.79,-2.77, 0.65);

    private Pose2d poseParking1 = new Pose2d(-25.79, -24, 0);
    private Pose2d poseParking2 = new Pose2d(-25.79, 0, 0);
    private Pose2d poseParking3 = new Pose2d(-25.79, 24, 0);

    private Trajectory trajectoryHomeToBackUp = null;
    private Trajectory trajectoryBackUpToPole = null;
    private Trajectory trajectoryPoleToParkingPosition1 = null;
    private Trajectory trajectoryPoleToParkingPosition2 = null;
    private Trajectory trajectoryPoleToParkingPosition3 = null;

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

        lift.moveToPosition(Lift.POSITION_MID_TERMINAL);

        trajectoryBackUpToPole = drive.trajectoryBuilder(poseBackup)
                .lineToLinearHeading(poseMediumPole)
                .build();

        drive.followTrajectory(trajectoryBackUpToPole);

        sleep(1000);
        arm.rotateRear();
        sleep(2000);
        lift.moveToPosition(Lift.POSITION_MID_TERMINAL + 350);
        sleep(1000);
        claw.open();
        sleep(1000);
        arm.rotateForward();
        sleep(1000);
        driveToParkingPosition(drive);
        lift.moveToPosition(Lift.POSITION_GROUND);
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
