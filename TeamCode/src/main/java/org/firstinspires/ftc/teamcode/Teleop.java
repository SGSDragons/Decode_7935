package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Teleop", group="Linear OpMode")
@Config
public class  Teleop extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    DcMotor shooter;

    public static double minpos = 0.68;
    public static double maxpos = 1;
    public static boolean runwheel = false;

    @Override
    public void runOpMode(){

        driveSubsystem = new DriveSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);

        driveSubsystem.setTargetHeading(0);
        driveSubsystem.resetYaw();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()){

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            // set target heading to goal and disable the joystick
            if (gamepad1.right_bumper) {
                driveSubsystem.setTargetHeading(-45);
                turn = driveSubsystem.gotoHeading(true);
                driveSubsystem.pointAtGoal = true;
            } else if (gamepad1.left_bumper) {
                driveSubsystem.setTargetHeading(45);
                turn = driveSubsystem.gotoHeading(true);
                driveSubsystem.pointAtGoal = true;
            } else {
                driveSubsystem.pointAtGoal = false;
            }
            if (gamepad1.a) {
                driveSubsystem.resetYaw();
            }

            driveSubsystem.setMotion(drive, strafe, turn);
//            driveSubsystem.feildOriented(drive, strafe, turn);

            if (gamepad2.a) {
                shooterSubsystem.setTargetSpeed(1);
                shooterSubsystem.enableShooter();
            } else if (gamepad2.x) {
                shooterSubsystem.setTargetSpeed(2);
                shooterSubsystem.enableShooter();
            }  else if (gamepad2.y) {
                shooterSubsystem.setTargetSpeed(3);
                shooterSubsystem.enableShooter();
            } else {
//                double shooterpower = -gamepad2.right_stick_y;
//                shooterSubsystem.runShooter(shooterpower);
                shooterSubsystem.setTargetSpeed(0);
                shooterSubsystem.enableShooter();
            }

            double intakepower = gamepad2.right_bumper ? 0 : gamepad2.left_stick_y;
            double indexpower = gamepad2.left_bumper ? 0 : gamepad2.left_stick_y;
            intakeSubsystem.setPower(intakepower, indexpower, true);

            // make sure the limit switch doesn't stop the indexer right as the ball hits the flywheel
//            if (shooterSubsystem.atTargetVelocity() && shooterSubsystem.targetflywheelspeed != shooterSubsystem.defalt_speed) {
//                intakeSubsystem.setPower(intakepower, 0.8, false);
//            } else {
//                intakeSubsystem.setPower(intakepower, indexpower, true);
//            }

            driveSubsystem.updateTelemetry();
            intakeSubsystem.updateTelemetry();
            shooterSubsystem.updateTelemetry();
            telemetry.update();
        }
    }
}
