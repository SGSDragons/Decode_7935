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

        shooter = hardwareMap.get(DcMotor.class,"flywheel");

        waitForStart();

        while (opModeIsActive()){

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double intakepower = gamepad2.left_stick_y;
            double shooterpower = gamepad2.left_trigger;
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setPower(shooterpower);

            telemetry.addData("Shooter Power", shooter.getPower());
            telemetry.update();

            driveSubsystem.setMotion(drive, strafe, turn);
            intakeSubsystem.setPower(intakepower, intakepower);

//            if (Math.abs(shooterpower) < 0.1 && Math.abs(intakepower) > 0.1) {
//                shooterSubsystem.runShooter(-0.4);
//            }
//            else {
//                shooterSubsystem.runShooter(shooterpower);
//            }
            telemetry.update();
        }
    }
}
