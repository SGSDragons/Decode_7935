package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "RedAuto")
@Config
public class RedAuto extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    public static int drive1 = -53;
    public static int strafe1 = 0;

    public static int drive2 = 40;
    public static int strafe2 = 0;
    public static int turn1 = -45;

    public static int drive3 = -30;


    @Override
    public void runOpMode() {

        waitForStart();
        runtime.reset();

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        driveSubsystem = new DriveSubsystem(hardwareMap);
        driveSubsystem.resetYaw();

        AutoCommands commands = new AutoCommands(driveSubsystem, intakeSubsystem, shooterSubsystem);

        commands.move(drive1,strafe1);
        commands.shootball(5);

        commands.turn(turn1);

        commands.intakeball(drive2,0);
        commands.intakeball(drive3,0);

        commands.turn(0);

        commands.shootball(5);
    }
}
