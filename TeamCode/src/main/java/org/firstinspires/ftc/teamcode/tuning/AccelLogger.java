package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Rotation2;
import com.acmerobotics.roadrunner.Transform2;
import com.acmerobotics.roadrunner.Vector2;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.MidpointTimer;

import java.util.ArrayList;
import java.util.List;

// FIXME: which group, if any?
@TeleOp(group = "drive")
public final class AccelLogger extends LinearOpMode {
    private static final double POWER = 0.8;

    @Override
    public void runOpMode() throws InterruptedException {
        DriveView view = new DriveView(new MecanumDrive(hardwareMap,
                new Transform2(new Vector2(0, 0), Rotation2.exp(0))));

        // FIXME: log current
        class Data {
            final List<List<Double>> powerTimes = new ArrayList<>();
            final List<List<Double>> powers = new ArrayList<>();

            final List<Double> voltageTimes = new ArrayList<>();
            final List<Double> voltages = new ArrayList<>();

            final List<Double> encTimes = new ArrayList<>();
            final List<List<Integer>> forwardEncPositions = new ArrayList<>();
            final List<List<Integer>> forwardEncVels = new ArrayList<>();
        }

        Data data = new Data();
        for (DcMotorEx m : view.motors) {
            data.powerTimes.add(new ArrayList<>());
            data.powers.add(new ArrayList<>());
        }
        for (Encoder e : view.forwardEncs) {
            data.forwardEncPositions.add(new ArrayList<>());
            data.forwardEncVels.add(new ArrayList<>());
        }

        waitForStart();

        MidpointTimer t = new MidpointTimer();
        for (DcMotorEx m : view.motors) {
            m.setPower(POWER);
        }

        while (opModeIsActive()) {
            for (int i = 0; i < view.motors.size(); i++) {
                data.powerTimes.get(i).add(t.addSplit());
                data.powers.get(i).add(POWER);
            }

            data.voltages.add(view.voltageSensor.getVoltage());
            data.voltageTimes.add(t.addSplit());

            for (int i = 0; i < view.forwardEncs.size(); i++) {
                Encoder.PositionVelocityPair p = view.forwardEncs.get(i).getPositionAndVelocity();
                data.forwardEncPositions.get(i).add(p.position);
                data.forwardEncVels.get(i).add(p.velocity);
            }
            data.encTimes.add(t.addSplit());
        }

        for (DcMotorEx m : view.motors) {
            m.setPower(0);
        }

        TuningFiles.save(TuningFiles.FileType.ACCEL, data);
    }
}
