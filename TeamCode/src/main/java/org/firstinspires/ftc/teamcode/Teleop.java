package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Teleop", group="Linear OpMode")
public class  Teleop extends LinearOpMode{

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

        while (opModeIsActive()){

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            if (gamepad1.right_bumper && !driveSubsystem.reachedHeading()) {
                driveSubsystem.setTargetHeading(-45);
                turn = driveSubsystem.gotoHeading();
            }

            double intakepower = gamepad2.left_stick_y;
            double shooterpower = gamepad2.right_stick_y;

            driveSubsystem.setMotion(drive, strafe, turn);
//            driveSubsystem.feildOriented(drive,strafe,turn);
            intakeSubsystem.setPower(intakepower, intakepower);
            shooterSubsystem.runShooter(shooterpower);


            telemetry.addData("Heading", driveSubsystem.getHeading());
            telemetry.update();
        }
    }
}
