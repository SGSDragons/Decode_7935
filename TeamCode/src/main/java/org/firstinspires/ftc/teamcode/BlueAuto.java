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

@Autonomous(name = "BlueAuto")
@Config
public class BlueAuto extends LinearOpMode {

    public static int xfiring = -18;
    public static int yfiring = -17;   // 🔵 flipped for blue
    public static int turnfiring = -135;
    public static double turnoffset = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // 🔵 Blue Alliance Mirror Start Pose
        Pose2d init = new Pose2d(-52.0, -50.0, Math.toRadians(-135));

        final MecanumDrive drive = new MecanumDrive(hardwareMap, init);
        AutoShootIntake mechanisms = new AutoShootIntake(hardwareMap);

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        DriveSubsystem driving = new DriveSubsystem(hardwareMap);
        AutoCommands commands = new AutoCommands(driving,intake,shooter);

//        Action turnToGoal = (p) -> {commands.turn(turnoffset); return !driving.reachedHeading(); };

        waitForStart();
        mechanisms.shooter.setTargetSpeed(2);
        mechanisms.shooter.enableShooter();
        driving.resetYaw();

        // 🔵 MIRRORED TARGET POSES
        Pose2d firingPoint = new Pose2d(xfiring, yfiring, Math.toRadians(turnfiring));
        Pose2d firstBallRow = new Pose2d(-12, -25, Math.toRadians(-90));
        Pose2d secondBallRow = new Pose2d(12, -25, Math.toRadians(-90));

        // FIRST SHOT
        Actions.runBlocking(drive.actionBuilder(init)
                .setReversed(true)
                .splineToLinearHeading(firingPoint, Math.toRadians(-135))
                .stopAndAdd(mechanisms.new ShootThreeClose())
                .splineToLinearHeading(firstBallRow, 0)
                .build()
        );

        // INTAKE FIRST ROW
        Actions.runBlocking(
                new RaceAction(
                        mechanisms.runIntake,
                        drive.actionBuilder(drive.localizer.getPose())
                                .setTangent(-Math.PI/2)
                                .turnTo(Math.toRadians(-90))     // 🔵 mirrored
                                .lineToYConstantHeading(-60.0, new TranslationalVelConstraint(15))
                                .build()
                )
        );

        // Open gate
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .stopAndAdd(mechanisms.stopIntake)
                .setReversed(true)
                .lineToYConstantHeading(-45.0, new TranslationalVelConstraint(15))
                .lineToXConstantHeading(-5.0, new TranslationalVelConstraint(15))
                .setReversed(true)
                .lineToYConstantHeading(-60.0, new TranslationalVelConstraint(15))
                .build()
        );

        // RETURN + SHOOT AGAIN
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .setReversed(true)
                .splineToLinearHeading(firingPoint, Math.toRadians(-160))
                .stopAndAdd(mechanisms.new ShootThreeClose())
                .build()
        );

        // SECOND BALL ROW
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
//                        .splineToLinearHeading(secondBallRow, 0)
                        .splineToConstantHeading(secondBallRow.component1(),-Math.PI/2)
                        .turnTo(Math.toRadians(-90))
                        .build()
        );

        // INTAKE PASS #2
        Actions.runBlocking(new RaceAction(
                mechanisms.runIntake,
                drive.actionBuilder(drive.localizer.getPose())
                        .setTangent(-Math.PI/2)
                        .turnTo(Math.toRadians(-90))
                        .lineToYConstantHeading(-60.0, new TranslationalVelConstraint(15))
                        .build()
        ));

                Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .stopAndAdd(mechanisms.stopIntake)
                        .setReversed(true)
                        .lineToYConstantHeading(-45.0, new TranslationalVelConstraint(15))
                        .build());

        // FINAL RETURN + SHOOT
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .stopAndAdd(mechanisms.stopIntake)
                .setReversed(true)
                .lineToYConstantHeading(-45.0, new TranslationalVelConstraint(15))
                .splineToLinearHeading(firingPoint, Math.PI)
                .stopAndAdd(mechanisms.new ShootThreeClose())
                .build());

        while(opModeIsActive()) {}
    }
}
