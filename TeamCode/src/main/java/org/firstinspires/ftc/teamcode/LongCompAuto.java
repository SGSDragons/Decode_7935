package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

@Autonomous(name = "LongCompAuto")
@Config
public class LongCompAuto extends LinearOpMode {

    public static int xfiring = 52;
    public static int yfiring = -12;
    public static int turnfiring = -157;
    public static double turnoffset = 23;


    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d init = new Pose2d(60.0, -12.0, Math.toRadians(-90));
        final MecanumDrive drive = new MecanumDrive(hardwareMap, init);
        AutoShootIntake mechanisms = new AutoShootIntake(hardwareMap);

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        DriveSubsystem driving = new DriveSubsystem(hardwareMap);

        AutoCommands commands = new AutoCommands(driving,intake,shooter);
        Action turnToGoal = (p) -> {commands.turn(turnoffset); return !driving.reachedHeading(); };

        waitForStart();
        mechanisms.shooter.setTargetSpeed(2);
//        mechanisms.shooter.enableShooter();
        driving.resetYaw();

        Pose2d firingPoint = new Pose2d(xfiring, yfiring, Math.toRadians(turnfiring));
        Pose2d parkPos = new Pose2d(63, -36, Math.toRadians(-90));

        // Backup and take the first shot
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .lineToYConstantHeading(-36.0, new TranslationalVelConstraint(15))
                .build());

        while(opModeIsActive()) {
            // Stall for assessment
        }
    }
}
