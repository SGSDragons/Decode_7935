package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.rr.Localizer;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

@TeleOp(name = "RoadRunner")
public class RoadRunnerDemo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d init = new Pose2d(-52.0, 50.0, Math.toRadians(135));
        final MecanumDrive drive = new MecanumDrive(hardwareMap, init);

        waitForStart();

        Action stage1 = drive.actionBuilder(init)
                .lineToX(-27)
                .splineToLinearHeading(new Pose2d(-10, 25, Math.PI/2), 0)
                .setTangent(Math.PI/2)
                .lineToYConstantHeading(60.0, new TranslationalVelConstraint(15))
                .lineToYLinearHeading(15.0, Math.toRadians(105))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-30, 29, Math.toRadians(135)), Math.toRadians(45))

                .turnTo(Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(12, 25, 0), 0)
                .turnTo(Math.PI/2)
                .lineToYConstantHeading(60.0, new TranslationalVelConstraint(15))
                .splineTo(new Vector2d(-5, 15), Math.toRadians(45))
                .turnTo(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-30, 29, Math.toRadians(135)), Math.toRadians(45))
                .build();


        Actions.runBlocking(stage1);

        while(opModeIsActive()) {
            // Stall
        }
    }
}
