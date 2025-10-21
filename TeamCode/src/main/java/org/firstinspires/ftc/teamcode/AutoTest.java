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
        driveSubsystem.setTargetPosition(30,30);
        driveSubsystem.setTargetHeading(-44);

        while (!driveSubsystem.reachedPosition()) {
            driveSubsystem.gotoPosition();
            driveSubsystem.gotoHeading();
            telemetry.addData("Current heading", driveSubsystem.getHeading());
            telemetry.addData("Target heading", driveSubsystem.targetheading);
            telemetry.addData("Error", driveSubsystem.error);
            telemetry.addData("Drive Power", driveSubsystem.Drivepower);
            telemetry.addData("Strafe Power", driveSubsystem.Strafepower);
            telemetry.addData("Turn Power", driveSubsystem.Turnpower);
            telemetry.update();
        }
    }
}
