package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Teleop", group="Linear OpMode")
public class Teleop extends LinearOpMode{

    DriveSubsystem driveSubsystem;

    @Override
    public void runOpMode(){

        driveSubsystem = new DriveSubsystem(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            driveSubsystem.setMotion(drive, strafe, turn);

        }
    }
}
