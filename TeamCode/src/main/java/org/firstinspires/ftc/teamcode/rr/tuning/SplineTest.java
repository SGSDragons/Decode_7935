package org.firstinspires.ftc.teamcode.rr.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

@TeleOp(name = "Spline Test")
@Config
public final class SplineTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-48.0, 0.0, Math.PI / 2.0));

        waitForStart();

        while (opModeIsActive()) {
            Action motion = drive.actionBuilder(drive.localizer.getPose())
                    .setTangent(Math.PI / 2)
                    .splineTo(new Vector2d(0.0, 48.0), 0.0)
                    .splineTo(new Vector2d(48.0, 0.0), -Math.PI / 2)
                    .splineTo(new Vector2d(0.0, -48.0), -Math.PI)
                    .splineTo(new Vector2d(-48.0, 0.0), Math.PI / 2)
                    .build();
            Actions.runBlocking(motion);
        }
    }
}
