package org.firstinspires.ftc.teamcode.rr.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.rr.ThreeDeadWheelLocalizer;

@Config
public final class ManualLateralFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 20;

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.PI/2));
        if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
            throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
        }

        waitForStart();

        while (opModeIsActive()) {
            Actions.runBlocking(
                    drive.actionBuilder(new Pose2d(0, 0, Math.PI/2))
                            .setTangent(0.0)
                            .lineToXConstantHeading(DISTANCE)
                            .lineToXConstantHeading(0)
                            .build());
        }
    }
}