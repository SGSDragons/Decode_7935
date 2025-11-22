package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BlueAuto")
@Config
public class BlueAuto extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    public static int drive1 = -60;
    public static int strafe1 = 0;

    public static int drive2 = 15;
    public static int strafe2 = 4;
    public static int turn1 = 45;

    public static int drive3 = 15;


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
        commands.shootball();

        commands.turn(turn1);
        commands.move(drive2,strafe2);

        commands.intakeball(drive3,0);
        commands.intakeball(-drive3,0);

        commands.move(-drive2,-strafe2);
        commands.turn(0);

        commands.shootball();
    }
}
