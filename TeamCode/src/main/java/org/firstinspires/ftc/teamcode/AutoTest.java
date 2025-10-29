package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoTest")
//@Config
public class AutoTest extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    DriveSubsystem driveSubsystem;

    @Override
    public void runOpMode() {

        waitForStart();

        driveSubsystem = new DriveSubsystem(hardwareMap);
        driveSubsystem.setTargetPosition(50,20);
        driveSubsystem.setTargetHeading(driveSubsystem.getHeading());

        while (!driveSubsystem.reachedPosition()) {
            double[] translate = driveSubsystem.gotoPosition();
            double turn = driveSubsystem.gotoHeading();
            driveSubsystem.setMotion(translate[0],translate[1],turn);

            telemetry.addData("Current heading", driveSubsystem.getHeading());
            telemetry.addData("Target heading", driveSubsystem.targetheading);
            telemetry.addData("Drive Power", translate[0]);
            telemetry.addData("Strafe Power", translate[1]);
            telemetry.addData("Turn Power", turn);
//            telemetry.addData("Error", driveSubsystem.error);
            telemetry.update();
        }
    }
}
