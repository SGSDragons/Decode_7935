package org.firstinspires.ftc.teamcode;

//import android.util.Range;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


public class DriveSubsystem {

    int ticksperrotation = 28;
    double circumfrence = 3.1416*3;
    int gearratio = 3;
    double ticksperinch = (ticksperrotation*gearratio) / circumfrence;

    double driveGain = 0.001;
    double turnGain = -0.001;
    double minDrivePower = 0.2;
    double minStrafePower = 0.3;
    double minTurnPower = 0.3;

    DcMotor frontleftmotor;
    DcMotor frontrightmotor;
    DcMotor backleftmotor;
    DcMotor backrightmotor;
    IMU imu;

    int drivetarget = 0;
    int strafetarget = 0;
    double targetheading = 0;


    public DriveSubsystem(HardwareMap hardwareMap) {
        frontleftmotor = hardwareMap.get(DcMotor.class, "frontleft");
        frontrightmotor = hardwareMap.get(DcMotor.class, "frontright");
        backleftmotor = hardwareMap.get(DcMotor.class, "backleft");
        backrightmotor = hardwareMap.get(DcMotor.class, "backright");
        imu = hardwareMap.get(IMU.class,"imu");

        frontleftmotor.setDirection(DcMotor.Direction.FORWARD);
        frontrightmotor.setDirection(DcMotor.Direction.FORWARD);
        backleftmotor.setDirection(DcMotor.Direction.FORWARD);
        backrightmotor.setDirection(DcMotor.Direction.REVERSE);

        imu.resetYaw();
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD))
        );
    }

    public double[] gotoPosition() {
        int currentdrive = getDrivePositions();
        int currentstrafe = getStrafePositions();
        int driveError = drivetarget - currentdrive;
        int strafeError = strafetarget - currentstrafe;
        double distance = Math.sqrt(driveError*driveError + strafeError*strafeError);

        double drivepower = Range.clip(driveError * driveGain, -1.0, 1.0) * Math.abs(driveError/distance);
        double strafepower = Range.clip(strafeError * driveGain, -1.0, 1.0) * Math.abs(strafeError/distance);

        double adjust = minDrivePower / Math.abs(drivepower);
        if (adjust > 1.0) {
            drivepower *= adjust;
        }
        adjust = minStrafePower / Math.abs(strafepower);
        if (adjust > 1.0) {
            strafepower *= adjust;
        }

        if (Math.abs(driveError) <= 3) drivepower = 0;
        if (Math.abs(strafeError) <= 3) strafepower = 0;

//        setMotion(drivepower,strafepower,0);
        return new double[] {drivepower, strafepower};
    }

    public double gotoHeading() {
        double currentheading = getHeading();
        double turnerror = targetheading - currentheading;
        if (Math.abs(turnerror) > 180) {
            double sign = -turnerror/Math.abs(turnerror);
            turnerror = sign * (360 - Math.abs(turnerror));
        }

        double turnpower = turnerror * turnGain;
        double adjust = minTurnPower / Math.abs(turnpower);
        if (adjust > 1.0) {
            turnpower *= adjust;
        }

//        setMotion(0,0,turnpower);
        return turnpower;
    }

    public void setTargetPosition(int drive, int strafe) {
        drivetarget = getDrivePositions() + (int)(drive * ticksperinch);
        strafetarget = getStrafePositions() + (int)(strafe * ticksperinch);
    }

    public void setTargetHeading(double degrees) {
        targetheading = degrees;
    }

    public void setMotion(double drive, double strafe, double turn) {
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;

//        double frontLeftPower = drive + strafe + turn;
//        double frontRightPower = drive - strafe - turn;
//        double backLeftPower = drive - strafe + turn;
//        double backRightPower = drive + strafe - turn;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        frontleftmotor.setPower(frontLeftPower);
        frontrightmotor.setPower(frontRightPower);
        backleftmotor.setPower(backLeftPower);
        backrightmotor.setPower(backRightPower);
    }

    public void feildOriented(double drive, double strafe, double turn){
        double currentheading = getHeading() * Math.PI / 180;
        double drivepower = drive * Math.cos(-currentheading) - strafe * Math.sin(-currentheading);
        double strafepower = drive * Math.sin(-currentheading) + strafe * Math.cos(-currentheading);
    }

    public boolean reachedPosition() {
        return (Math.abs(drivetarget - getDrivePositions()) <= 3);
    }

    public boolean reachedHeading() {
        return (Math.abs(targetheading - getHeading()) <= 2);
    }

    public int getDrivePositions() {
        int sum = frontleftmotor.getCurrentPosition() +
                frontrightmotor.getCurrentPosition() +
                backleftmotor.getCurrentPosition() +
                backrightmotor.getCurrentPosition();
        return (int) (sum/4);
    }

    public int getStrafePositions() {
        int sum = frontleftmotor.getCurrentPosition() +
                -frontrightmotor.getCurrentPosition() +
                -backleftmotor.getCurrentPosition() +
                backrightmotor.getCurrentPosition();
        return (int) (sum/4);
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}


