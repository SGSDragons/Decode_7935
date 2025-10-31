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
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    public static int drive1 = -50;
    public static int index1 = -10;

    @Override
    public void runOpMode() {

        waitForStart();

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        driveSubsystem = new DriveSubsystem(hardwareMap);
        driveSubsystem.resetYaw();

        drive1();
        shootball();
    }

    public void drive1(){
        driveSubsystem.setTargetPosition(drive1,0);
        driveSubsystem.setTargetHeading(driveSubsystem.getHeading());

        while (!driveSubsystem.reachedPosition()) {
            double[] translation = driveSubsystem.gotoPosition();
            double turn = driveSubsystem.gotoHeading();
            driveSubsystem.setMotion(translation[0], translation[1],turn);

            updateTelemetry(translation,turn);
        }
    }

    public void shootball(){
        while (!intakeSubsystem.reachedIndexPosition()) {
            shooterSubsystem.runShooter(1);
            if (shooterSubsystem.flywheel.getPower() >= 0.9){
                intakeSubsystem.moveIndexer(index1);
            }
        }
    }

    public void updateTelemetry(double[] translation, double turn){
        telemetry.addData("Current heading", driveSubsystem.getHeading());
        telemetry.addData("Target heading", driveSubsystem.targetheading);
        telemetry.addData("Drive Power", translation[0]);
        telemetry.addData("Strafe Power", translation[1]);
        telemetry.addData("Turn Power", turn);
//        telemetry.addData("Error", driveSubsystem.error);
        telemetry.update();
    }
}
