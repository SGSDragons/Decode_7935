package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "AutoTest")
@Config
//@Config
public class AutoTest extends LinearOpMode {

    ElapsedTime runtime = new ElapsedTime();
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    public static int drive1 = 0;
    public static int strafe1 = -40;

    @Override
    public void runOpMode() {

        waitForStart();
        runtime.reset();

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        driveSubsystem = new DriveSubsystem(hardwareMap);
        driveSubsystem.resetYaw();

        drive1();
//        shootball();
    }

    public void drive1(){
        driveSubsystem.setTargetPosition(drive1,strafe1);
        driveSubsystem.setTargetHeading(driveSubsystem.getHeading());

        while (!driveSubsystem.reachedPosition() || !driveSubsystem.reachedHeading()) {
            double[] translation = driveSubsystem.gotoPosition();
            double turn = driveSubsystem.gotoHeading();
            driveSubsystem.setMotion(translation[0], translation[1], turn);

            updateTelemetry(translation,turn);
        }
        driveSubsystem.stop();
    }

    public void shootball(){
        while (runtime.milliseconds() <= 10000) {
            shooterSubsystem.setTargetSpeed(1);
            shooterSubsystem.enableShooter();
            intakeSubsystem.setPower(-0.7,0);
        }
    }

    public void updateTelemetry(double[] translation, double turn){
        telemetry.addData("Current heading", driveSubsystem.getHeading());
        telemetry.addData("Target heading", driveSubsystem.targetheading);
        telemetry.addData("Drive Power", translation[0]);
        telemetry.addData("Strafe Power", translation[1]);
        telemetry.addData("Turn Power", turn);
        telemetry.addData("Error", driveSubsystem.strafetarget - driveSubsystem.getStrafePositions());
        telemetry.update();
    }
}
