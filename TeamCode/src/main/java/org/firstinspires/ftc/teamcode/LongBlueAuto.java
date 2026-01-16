package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.RaceAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

@Autonomous(name = "LongBlueAuto")
@Config
public class LongBlueAuto extends LinearOpMode {

    public static int xfiring = 52;
    public static int yfiring = -12;
    public static int turnfiring = -157;
    public static double turnoffset = 23;


    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d init = new Pose2d(60.0, -12.0, Math.toRadians(180));
        final MecanumDrive drive = new MecanumDrive(hardwareMap, init);
        AutoShootIntake mechanisms = new AutoShootIntake(hardwareMap);

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        DriveSubsystem driving = new DriveSubsystem(hardwareMap);

        AutoCommands commands = new AutoCommands(driving,intake,shooter);
        Action turnToGoal = (p) -> {commands.turn(turnoffset); return !driving.reachedHeading(); };

        waitForStart();
        mechanisms.shooter.setTargetSpeed(2);
        mechanisms.shooter.enableShooter();
        driving.resetYaw();

        Pose2d firingPoint = new Pose2d(xfiring, yfiring, Math.toRadians(turnfiring));
        Pose2d firstBallRow = new Pose2d(36, -25, Math.toRadians(-90));
        Pose2d secondBallRow = new Pose2d(14, -25, Math.toRadians(-90));

        // Backup and take the first shot
        Actions.runBlocking(drive.actionBuilder(init)
                .setReversed(true)
                .splineToLinearHeading(firingPoint, Math.toRadians(0)) // Tangent points backwards along the route
                .stopAndAdd(mechanisms.new ShootThree())
                .splineToLinearHeading(firstBallRow, 0)
                .build());

        // Move to the first row of balls and grab them
        Actions.runBlocking(
                new RaceAction(
                        mechanisms.runIntake,
                        drive.actionBuilder(drive.localizer.getPose())
                                .setTangent(Math.PI/2)
                                .turnTo(-Math.PI/2)
                                .lineToYConstantHeading(-60.0, new TranslationalVelConstraint(15))
                                .build()
                ));

        // Return to the shooting position and shoot again.
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .stopAndAdd(mechanisms.stopIntake)
                .setReversed(true)
                .lineToYConstantHeading(-35.0, new TranslationalVelConstraint(15))
                .splineToLinearHeading(firingPoint, Math.toRadians(180))
                .stopAndAdd(turnToGoal)
                .stopAndAdd(mechanisms.new ShootThree())
                .build());

        // Move to the second row of balls and grab them
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .splineToLinearHeading(secondBallRow, 0)
                        .turnTo(-Math.PI/2)
                        .build());

        Actions.runBlocking(new RaceAction(
                mechanisms.runIntake,
                drive.actionBuilder(drive.localizer.getPose())
                        .setTangent(Math.PI/2)
                        .turnTo(-Math.PI/2)
                        .lineToYConstantHeading(-60.0, new TranslationalVelConstraint(15))
                        .build()
        ));

        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .stopAndAdd(mechanisms.stopIntake)
                        .setReversed(true)
                        .lineToYConstantHeading(-45.0, new TranslationalVelConstraint(15))
                        .build());

        // Return to the shooting position and shoot again
//        Actions.runBlocking(
//                drive.actionBuilder(drive.localizer.getPose())
//                        .stopAndAdd(mechanisms.stopIntake)
//                        .setReversed(true)
//                        .lineToYConstantHeading(45.0, new TranslationalVelConstraint(15))
//                        .splineToLinearHeading(firingPoint, Math.PI)
//                        .stopAndAdd(turnToGoal)
//                        .stopAndAdd(mechanisms.new ShootThree())
//                        .build());

        while(opModeIsActive()) {
            // Stall for assessment
        }
    }
}
