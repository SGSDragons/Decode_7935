package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.rr.Localizer;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

@TeleOp(name = "RoadRunner")
public class RoadRunnerDemo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        final MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0.0, 0.0, 0.0));
        final Localizer localizer = drive.localizer;

        waitForStart();

        while (opModeIsActive()) {
            localizer.update();

            TelemetryPacket p = new TelemetryPacket();
            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));
            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }
}
