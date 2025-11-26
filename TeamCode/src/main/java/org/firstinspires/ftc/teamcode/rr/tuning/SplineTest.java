package org.firstinspires.ftc.teamcode.rr.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

@TeleOp(name = "Spline Test")
@Config
public final class SplineTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));

        waitForStart();

        Action motion = drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(20.0, 20.0), Math.PI/2)
                .splineTo(new Vector2d(20, 40.0), Math.PI / 2, new TranslationalVelConstraint(15.0))
                .build();
        Actions.runBlocking(motion);

        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .splineTo(new Vector2d(20, 40.0), Math.PI / 2, new TranslationalVelConstraint(15.0))
                .build());
    }
}
