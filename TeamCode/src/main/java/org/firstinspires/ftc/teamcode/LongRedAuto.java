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
public class LongRedAuto extends LinearOpMode {

    public static int xfiring = 58;
    public static int yfiring = -6;   // 🔵 flipped for blue
    public static double turnoffset = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        // 🔵 Blue Alliance Mirror Start Pose
        Pose2d init = new Pose2d(60, -6, Math.toRadians(180));

        final MecanumDrive drive = new MecanumDrive(hardwareMap, init);
        AutoShootIntake mechanisms = new AutoShootIntake(hardwareMap);

        IntakeSubsystem intake = new IntakeSubsystem(hardwareMap);
        ShooterSubsystem shooter = new ShooterSubsystem(hardwareMap);
        DriveSubsystem driving = new DriveSubsystem(hardwareMap);
        AutoCommands commands = new AutoCommands(driving,intake,shooter);

        Action turnToGoal = (p) -> {commands.turn(turnoffset); return !driving.reachedHeading(); };

        waitForStart();
        mechanisms.shooter.setTargetSpeed(1);
        mechanisms.shooter.enableShooter();
        driving.resetYaw();

        // 🔵 MIRRORED TARGET POSES
        Pose2d firingPoint = new Pose2d(xfiring, yfiring, Math.toRadians(-135));
        Pose2d firstBallRow = new Pose2d(-9, -25, Math.toRadians(-90));
        Pose2d secondBallRow = new Pose2d(14, -25, Math.toRadians(-90));

        // FIRST SHOT
        Actions.runBlocking(drive.actionBuilder(init)
                .setReversed(true)
                .splineToLinearHeading(firingPoint, Math.toRadians(-135))
                .stopAndAdd(turnToGoal)
                .stopAndAdd(mechanisms.new ShootThree())
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

        // RETURN + SHOOT AGAIN
        Actions.runBlocking(drive.actionBuilder(drive.localizer.getPose())
                .stopAndAdd(mechanisms.stopIntake)
                .setReversed(true)
                .splineToLinearHeading(firingPoint, Math.toRadians(-160))
                .stopAndAdd(turnToGoal)
                .stopAndAdd(mechanisms.new ShootThree())
                .build()
        );

        // SECOND BALL ROW
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .turnTo(Math.toRadians(30))           // 🔵 mirrored
                        .splineToLinearHeading(secondBallRow, 0)
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

        // FINAL RETURN + SHOOT
        Actions.runBlocking(
                drive.actionBuilder(drive.localizer.getPose())
                        .stopAndAdd(mechanisms.stopIntake)
                        .setReversed(true)
                        .lineToYConstantHeading(-45.0, new TranslationalVelConstraint(15))
                        .splineToLinearHeading(firingPoint, Math.PI)
                        .stopAndAdd(turnToGoal)
                        .stopAndAdd(mechanisms.new ShootThree())
                        .build()
        );

        while(opModeIsActive()) {

        }
    }
}
