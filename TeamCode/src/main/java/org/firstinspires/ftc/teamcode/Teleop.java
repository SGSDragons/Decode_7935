package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Teleop", group="Linear OpMode")
public class Teleop extends LinearOpMode{

    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    @Override
    public void runOpMode(){

        driveSubsystem = new DriveSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double intakepower = gamepad2.left_stick_y;
            double shooterpower = gamepad2.left_trigger;
            shooterSubsystem.runShooter(shooterpower);

            driveSubsystem.setMotion(drive, strafe, turn);
            intakeSubsystem.setPower(intakepower, intakepower);

            if (Math.abs(shooterpower) < 0.1 && Math.abs(intakepower) > 0.1) {
                shooterSubsystem.runShooter(-0.4);
            }
            else {
                shooterSubsystem.runShooter(shooterpower);
            }
        }
    }
}
