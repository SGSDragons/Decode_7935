//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//@Autonomous(name = "RedAuto")
//@Config
//public class RedAuto extends LinearOpMode{
//
//    ElapsedTime runtime = new ElapsedTime();
//    DriveSubsystem driveSubsystem;
//    IntakeSubsystem intakeSubsystem;
//    ShooterSubsystem shooterSubsystem;
//
//    public static int drive1 = -53;
//    public static int strafe1 = 0;
//
//    public static int drive2 = 40;
//    public static int strafe2 = 0;
//    public static int turn1 = -45;
//
//    public static int drive3 = -35;
//
//
//    @Override
//    public void runOpMode() {
//
//        waitForStart();
//        runtime.reset();
//
//        intakeSubsystem = new IntakeSubsystem(hardwareMap);
//        shooterSubsystem = new ShooterSubsystem(hardwareMap);
//        driveSubsystem = new DriveSubsystem(hardwareMap);
//        driveSubsystem.resetYaw();
//
//        AutoCommands commands = new AutoCommands(driveSubsystem, intakeSubsystem, shooterSubsystem);
//
//        commands.move(drive1,strafe1);
//        commands.shootball(5);
//
//        commands.turn(turn1);
//
//        commands.intakeball(drive2,0);
//        commands.intakeball(drive3,0);
//
//        commands.turn(0);
//
//        commands.shootball(5);
//        commands.move(0,10);
//    }
//}

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rr.Localizer;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

@TeleOp(name = "RedAuto")
public class RedAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d init = new Pose2d(-52.0, 50.0, Math.toRadians(135));
        final MecanumDrive drive = new MecanumDrive(hardwareMap, init);
        AutoShootIntake autohelpers = new AutoShootIntake(hardwareMap);

//        this.intake = new IntakeSubsystem(hardwareMap);
//        this.shooter = new ShooterSubsystem(hardwareMap);

        waitForStart();
        autohelpers.shooter.setTargetSpeed(1);
        autohelpers.shooter.enableShooter();

        Pose2d firingPoint = new Pose2d(-30, 29, Math.toRadians(135));
        Pose2d firstBallRow = new Pose2d(-9, 25, Math.toRadians(90));

        TrajectoryActionBuilder autoMode = drive.actionBuilder(init);

        // Backup and take the first shot
        autoMode = autoMode
                .setReversed(true)
                .splineToLinearHeading(firingPoint, Math.toRadians(180.0)) // Tangent points backwards along the route
                .stopAndAdd(autohelpers.new ShootThree());

        // Move to the first row of balls and grab them
        autoMode = autoMode
                .splineToLinearHeading(firstBallRow, 0)
                .stopAndAdd(new RaceAction(
                        autohelpers.runIntake,
                        drive.actionBuilder(firstBallRow)
                                .setTangent(Math.PI/2)
                                .lineToYConstantHeading(60.0, new TranslationalVelConstraint(15))
                                .build()
                ))
                .stopAndAdd(autohelpers.stopIntake);

        // Return to the shooting position and shoot again.
        autoMode = autoMode
                .lineToYLinearHeading(15.0, Math.toRadians(105))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(firingPoint, Math.toRadians(45))
                .stopAndAdd(autohelpers.new ShootThree());

        // Move to the second row of balls and grab them
        autoMode = autoMode
                .turnTo(Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(14, 25, 0), 0)
                .turnTo(Math.PI/2)

                .stopAndAdd(new RaceAction(
                        autohelpers.runIntake,
                        drive.actionBuilder(firstBallRow)
                                .setTangent(Math.PI/2)
                                .lineToYConstantHeading(60.0, new TranslationalVelConstraint(15))
                                .build()
                ))
                .stopAndAdd(autohelpers.stopIntake);

        // Return to the shooting position and shoot again
        autoMode = autoMode
                .setReversed(true)
                .splineTo(new Vector2d(-5, 15), Math.toRadians(45))
                .turnTo(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-30, 29, Math.toRadians(135)), Math.toRadians(45))
                .stopAndAdd(autohelpers.new ShootThree());

        Actions.runBlocking(autoMode.build());

        while(opModeIsActive()) {
            // Stall for assessment
        }
    }
}
