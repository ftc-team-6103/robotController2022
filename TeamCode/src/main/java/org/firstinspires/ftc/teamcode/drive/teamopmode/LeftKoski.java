package org.firstinspires.ftc.teamcode.drive.teamopmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Arm;
import org.firstinspires.ftc.teamcode.hardware.Claw;
import org.firstinspires.ftc.teamcode.hardware.Lift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Disabled
@Autonomous(group = "drive")
public class LeftKoski extends TensorFlowOpMode {

    private Claw claw;
    private Servo clawServo;
    private Arm arm;
    private Servo armServo;
    private Lift lift;
    private DcMotor liftMotor;

    private Pose2d poseHome = new Pose2d(0,0,Math.toRadians(0));

    private Pose2d poseBackup = new Pose2d(-24.87, 0, 0); //2.47
    private Pose2d poseRotateToPole = new Pose2d(-28.86, 3.53, 2.47); //2.47
    private Pose2d poseInitialPole = new Pose2d(-35.28, 7.5, 2.47); //2.47

    private Pose2d poseRotateAroundPole = new Pose2d(-35.28, 7.5, 1.57);
    private Pose2d poseBackAwayFromPole = new Pose2d(-35.28, 0, 1.57);
    private Pose2d poseStrafeConeOutOfWay = new Pose2d(-45.28, 0, 1.57);


    private Pose2d poseParking1 = new Pose2d(-31.29, -26, 0);
    private Pose2d poseParking2 = new Pose2d(-29.29, 0, 0);
    private Pose2d poseParking3 = new Pose2d(-29.29, 26, 0);

    private TrajectorySequence sequencePlaceInitialCone = null;
    private TrajectorySequence sequenceMoveAwayFromPole = null;

    private Trajectory trajectoryPoleToParkingPosition1 = null;
    private Trajectory trajectoryPoleToParkingPosition2 = null;
    private Trajectory trajectoryPoleToParkingPosition3 = null;

    private String randomization = "";

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        claw = new Claw(clawServo);
        armServo = hardwareMap.get(Servo.class, "gate_servo");
        arm = new Arm(armServo);
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
        lift = new Lift(liftMotor);

        initTensorFlow();

        waitForStart();

//        double start = this.getRuntime();
//
//        while (this.getRuntime() < start + 2) {
//            telemetry.addData("runtime", this.getRuntime());
//            telemetry.update();
//            randomization = detectSignal();
//        }
//
//        telemetry.addData("randomization", "detected: " + randomization);
//        telemetry.update();
//
//
//        claw.close();
//        sleep(250);
//        lift.adjustUp();

        drive.setPoseEstimate(poseHome);

        claw.close();
        sleep(500);
        arm.rotateForward();
        lift.moveToPositionAsync(Lift.POSITION_MID_TERMINAL);

        sequencePlaceInitialCone = drive.trajectorySequenceBuilder(poseHome)
                .lineToLinearHeading(poseBackup)
                .lineToLinearHeading(poseRotateToPole)
                .lineToLinearHeading(poseInitialPole)
                .build();

        drive.followTrajectorySequence(sequencePlaceInitialCone);

        claw.open();

        sequenceMoveAwayFromPole = drive.trajectorySequenceBuilder(poseInitialPole)
                .lineToLinearHeading(poseRotateAroundPole)
                .lineToLinearHeading(poseBackAwayFromPole)
                .lineToLinearHeading(poseStrafeConeOutOfWay)
                .build();

        drive.followTrajectorySequence(sequenceMoveAwayFromPole);
    }

//    private void driveToParkingPosition(SampleMecanumDrive drive){
//        trajectoryPoleToParkingPosition1 = drive.trajectoryBuilder(poseMediumPole)
//                .lineToLinearHeading(poseParking1)
//                .build();
//        trajectoryPoleToParkingPosition2 = drive.trajectoryBuilder(poseMediumPole)
//                .lineToLinearHeading(poseParking2)
//                .build();
//        trajectoryPoleToParkingPosition3 = drive.trajectoryBuilder(poseMediumPole)
//                .lineToLinearHeading(poseParking3)
//                .build();
//
//        int parking = 3;
//
//        if (randomization != null && randomization.trim().length() > 0){
//            parking = Integer.parseInt(randomization.substring(0,1));
//        }
//
//        if (parking == 1){
//            drive.followTrajectory(trajectoryPoleToParkingPosition1);
//        }
//        else if (parking == 2){
//            drive.followTrajectory(trajectoryPoleToParkingPosition2);
//        }
//        else{
//            drive.followTrajectory(trajectoryPoleToParkingPosition3);
//        }
//    }
}
