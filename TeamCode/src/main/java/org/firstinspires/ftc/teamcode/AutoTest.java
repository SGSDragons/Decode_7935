package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoTest")
@Config
public class AutoTest extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    public static int drive1 = -60;
    public static int strafe1 = 0;

    public static int drive2 = 15;
    public static int strafe2 = -4;
    public static int turn2 = -45;

    @Override
    public void runOpMode() {

        waitForStart();
        runtime.reset();

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        driveSubsystem = new DriveSubsystem(hardwareMap);
        driveSubsystem.resetYaw();

        move(drive1,strafe1);
        shootball();

        turn(turn2);
        move(drive2,strafe2);
    }

    public void move(int drive, int strafe){
        driveSubsystem.setTargetPosition(drive,strafe);
        driveSubsystem.setTargetHeading(driveSubsystem.getHeading());

        while (!driveSubsystem.reachedPosition() || !driveSubsystem.reachedHeading()) {
            double[] translation = driveSubsystem.gotoPosition();
            double turn = driveSubsystem.reachedPosition()? driveSubsystem.gotoHeading(true) : driveSubsystem.gotoHeading(false);
            driveSubsystem.setMotion(translation[0], translation[1], turn);

            driveSubsystem.updateTelemetry();
        }
        driveSubsystem.stop();
    }

    public void turn(double degrees){
        driveSubsystem.setTargetHeading(degrees);

        while (!driveSubsystem.reachedHeading()) {
            double[] translation = driveSubsystem.gotoPosition();
            double turn = driveSubsystem.gotoHeading(true);
            driveSubsystem.setMotion(0, 0, turn);

            driveSubsystem.updateTelemetry();
        }
        driveSubsystem.stop();
    }

    public void shootball(){
        while (runtime.milliseconds() <= 10000) {
            shooterSubsystem.setTargetSpeed(1);
            shooterSubsystem.enableShooter();

            if (shooterSubsystem.atTargetVelocity()) {
                intakeSubsystem.setPower(-0.7,-0.7, false);
            } else {
                intakeSubsystem.setPower(-0.7,0, true);
            }

            intakeSubsystem.updateTelemetry();
            shooterSubsystem.updateTelemetry();
        }
    }
}
