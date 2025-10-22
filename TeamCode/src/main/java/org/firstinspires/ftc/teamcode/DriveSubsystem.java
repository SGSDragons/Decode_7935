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

    double error = 0;
    double Turnpower = 0;
    double Drivepower = 0;
    double Strafepower = 0;

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
        frontrightmotor.setDirection(DcMotor.Direction.REVERSE);
        backleftmotor.setDirection(DcMotor.Direction.FORWARD);
        backrightmotor.setDirection(DcMotor.Direction.REVERSE);

        imu.resetYaw();
        imu.initialize(
                new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD))
        );
    }

//    public void translate(int drive, int strafe) {
//
//        double distance = Math.sqrt(drive*drive + strafe*strafe);
//        double direction = getHeading();
//
//        int driveError = (int)(drive * ticksperinch);
//        int strafeError = (int)(strafe * ticksperinch);
//
//        int currentdrive = getDrivePositions();
//        int currentstrafe = getStrafePositions();
//
//        int drivetarget = currentdrive + driveError;
//        int strafetarget = currentstrafe + strafeError;
//
//        while (driveError > 3 || strafeError > 3) {
//
//            double drivepower = Range.clip(driveError * driveGain, -1.0, 1.0) * (drive/distance);
//            double strafepower = Range.clip(strafeError * driveGain, -1.0, 1.0) * (strafe/distance);
//
//            double adjust = minDrivePower / drivepower;
//            if (adjust > 1.0) {
//                drivepower *= adjust;
//            }
//            adjust = minStrafePower / strafepower;
//            if (adjust > 1.0) {
//                strafepower *= adjust;
//            }
//
//            if (driveError <= 3) drivepower = 0;
//            if (strafeError <= 3) strafepower = 0;
//
//            double turnadjust = turnCorrection(direction);
//            setMotion(drivepower,strafepower,turnadjust);
//
//            currentdrive = getDrivePositions();
//            currentstrafe = getStrafePositions();
//            driveError = drivetarget - currentdrive;
//            strafeError = strafetarget - currentstrafe;
//        }
//        setMotion(0,0,0);
//    }
//
//    public void turn(double degrees) {
//        double currentheading = getHeading();
//        double error = degrees - currentheading;
//
//        while (error > 1) {
//            double turnpower = error * turnGain;
//
//            double adjust = minDrivePower / turnpower;
//            if (adjust > 1.0) {
//                turnpower *= adjust;
//            }
//
//            setMotion(0,0,turnpower);
//            currentheading = getHeading();
//            error = degrees - currentheading;
//        }
//        setMotion(0,0,0);
//    }

    public void gotoPosition() {
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

        error = driveError;
        Drivepower = drivepower;
        Strafepower = strafepower;

        if (Math.abs(driveError) <= 3) drivepower = 0;
        if (Math.abs(strafeError) <= 3) strafepower = 0;

        setMotion(drivepower,strafepower,0);
    }

    public void gotoHeading() {
        double currentheading = getHeading();
        double turnerror = targetheading - currentheading;
        if (Math.abs(turnerror) > 180) {
            double sign = -turnerror/Math.abs(turnerror);
            turnerror = sign * (360 - Math.abs(turnerror));
        }

        double turnpower = turnerror * turnGain;
//        double adjust = minTurnPower / Math.abs(turnpower);
//        if (adjust > 1.0) {
//            turnpower *= adjust;
//        }

        error = turnerror;
        Turnpower = turnpower;

        setMotion(0,0,turnpower);
    }

    public void setTargetPosition(int drive, int strafe) {
        drivetarget = getDrivePositions() + (int)(drive * ticksperinch);
        strafetarget = getStrafePositions() + (int)(strafe * ticksperinch);
    }

    public void setTargetHeading(double degrees) {
        targetheading = degrees;
    }

    public void setMotion(double drive, double strafe, double turn) {
        double frontLeftPower = Range.clip(drive + strafe + turn, -1.0, 1.0);
        double frontRightPower = Range.clip(drive - strafe - turn, -1.0, 1.0);
        double backLeftPower = Range.clip(drive - strafe + turn, -1.0, 1.0);
        double backRightPower = Range.clip(drive + strafe - turn, -1.0, 1.0);

//        double frontLeftPower = Range.clip(drive + strafe + turn, -0.2, 0.2);
//        double frontRightPower = Range.clip(drive - strafe - turn, -0.2, 0.2);
//        double backLeftPower = Range.clip(drive - strafe + turn, -0.2, 0.2);
//        double backRightPower = Range.clip(drive + strafe - turn, -0.2, 0.2);

        frontleftmotor.setPower(frontLeftPower);
        frontrightmotor.setPower(frontRightPower);
        backleftmotor.setPower(backLeftPower);
        backrightmotor.setPower(backRightPower);
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


