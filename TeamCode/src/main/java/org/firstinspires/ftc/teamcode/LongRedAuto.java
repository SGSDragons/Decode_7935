package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "LongRedAuto")
@Config
public class LongRedAuto extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    public static int drive1 = 5;
    public static int strafe1 = 0;
    public static int turn1 = -17;


    @Override
    public void runOpMode() {

        waitForStart();
        runtime.reset();

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        driveSubsystem = new DriveSubsystem(hardwareMap);
        driveSubsystem.resetYaw();

        AutoCommands commands = new AutoCommands(driveSubsystem, intakeSubsystem, shooterSubsystem);

        commands.shootball(3);
        commands.move(drive1,strafe1);
    }
}
