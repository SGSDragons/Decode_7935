package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Teleop", group="Linear OpMode")
public class  Teleop extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();
    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;
    DcMotor shooter;

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

            if (gamepad1.right_bumper && !driveSubsystem.reachedHeading()) {
                driveSubsystem.setTargetHeading(-45);
                turn = driveSubsystem.gotoHeading();
            }
            if (gamepad1.left_bumper && !driveSubsystem.reachedHeading()) {
                driveSubsystem.setTargetHeading(45);
                turn = driveSubsystem.gotoHeading();
            }
            if (gamepad1.a) {
                driveSubsystem.resetYaw();
            }

//            driveSubsystem.setMotion(drive, strafe, turn);
            driveSubsystem.feildOriented(drive, strafe, turn);

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
                double shooterpower = -gamepad2.right_stick_y;
                shooterSubsystem.runShooter(shooterpower);
            }

            double intakepower = gamepad2.left_stick_y;
            if (gamepad2.right_bumper){
                intakeSubsystem.setPower(0, intakepower);
            } else{
                intakeSubsystem.setPower(intakepower, intakepower);
            }

            telemetry.addData("Heading", driveSubsystem.getHeading());
            telemetry.update();
            driveSubsystem.updateTelemetry();
        }
    }
}
