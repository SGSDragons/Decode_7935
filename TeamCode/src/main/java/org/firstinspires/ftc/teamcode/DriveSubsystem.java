package org.firstinspires.ftc.teamcode;

//import android.util.Range;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Config
public class DriveSubsystem {

    int ticksperrotation = 2000;
    double circumfrence = 3.1416*2;
    double ticksperinch = (ticksperrotation) / circumfrence;

    public static double turnsensitivity = 60;
    public static double driveGain = -0.00002;
    public static double strafeGain = 0.000002;
    public static double turnGain = -0.0075;
    public static double minDrivePower = 0.275;
    public static double minStrafePower = 0.5;
    public static double minTurnPower = 0.35;
    public static double minTurnPower2 = 0.1;
    public static double tolerance = 100;

    DcMotorEx frontleftmotor;
    DcMotorEx frontrightmotor;
    DcMotorEx backleftmotor;
    DcMotorEx backrightmotor;

    DcMotorEx leftwheel;
    DcMotorEx rightwheel;
    DcMotorEx strafewheel;
    IMU imu;

    double error;
    double power;

    int drivetarget = 0;
    int strafetarget = 0;
    double targetheading = 0;
    boolean pointAtGoal;


    public DriveSubsystem(HardwareMap hardwareMap) {
        frontleftmotor = hardwareMap.get(DcMotorEx.class, "frontleft");
        frontrightmotor = hardwareMap.get(DcMotorEx.class, "frontright");
        backleftmotor = hardwareMap.get(DcMotorEx.class, "backleft");
        backrightmotor = hardwareMap.get(DcMotorEx.class, "backright");
        imu = hardwareMap.get(IMU.class,"imu");

        leftwheel = hardwareMap.get(DcMotorEx.class, "leftwheel");
        rightwheel = hardwareMap.get(DcMotorEx.class, "indexer");
        strafewheel = hardwareMap.get(DcMotorEx.class, "intake");

        frontleftmotor.setDirection(DcMotor.Direction.FORWARD);
        frontrightmotor.setDirection(DcMotor.Direction.FORWARD);
        backleftmotor.setDirection(DcMotor.Direction.FORWARD);
        backrightmotor.setDirection(DcMotor.Direction.REVERSE);

        frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        double distance = Math.sqrt(Math.abs(driveError*driveError + strafeError*strafeError));

        double drivepower = Range.clip(driveError * driveGain, -1.0, 1.0) * Math.abs(driveError/distance);
        double strafepower = Range.clip(strafeError * strafeGain, -1.0, 1.0) * Math.abs(strafeError/distance);

        double adjust = minDrivePower / Math.abs(drivepower);
        if (adjust > 1.0 || Math.abs(drivepower) == 0) {
            drivepower *= adjust;
        }
        adjust = minStrafePower / Math.abs(strafepower);
        if (adjust > 1.0 || Math.abs(drivepower) == 0) {
            strafepower *= adjust;
        }

        if (Math.abs(driveError) <= tolerance){
            drivepower = 0;
        }
        if (Math.abs(strafeError) <= tolerance) {
            strafepower = 0;
        }

        error = driveError;
        power = drivepower;

        return new double[] {drivepower, strafepower};
    }

    // set minPower lower to get rid of wobble
    public double gotoHeading(boolean minPower) {
        double currentheading = getHeading();
        double turnerror = targetheading - currentheading;
        if (Math.abs(turnerror) > 180) {
            double sign = -turnerror/Math.abs(turnerror);
            turnerror = sign * (360 - Math.abs(turnerror));
        }

        double turnpower = turnerror * turnGain;

        double adjust = minPower ? minTurnPower / Math.abs(turnpower) : minTurnPower2 / Math.abs(turnpower) ;
        if (adjust > 1.0) {
            turnpower *= adjust;
        }
        if (Math.abs(turnerror) <= 2) turnpower = 0;

        return turnpower;
    }

    public void setTargetPosition(int drive, int strafe) {
        drivetarget = getDrivePositions() + (int)(-drive * ticksperinch);
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
        double drivepower = drive * Math.cos(-currentheading) + strafe * Math.sin(-currentheading);
        double strafepower = -drive * Math.sin(-currentheading) + strafe * Math.cos(-currentheading);

        // disable turnadjust if pointAtGoal is true
        double turnadjust = Math.pow(-turn,3) * turnsensitivity;
        if (Math.abs(turn) > 0.1 && !pointAtGoal) {
            setTargetHeading(getHeading()+turnadjust);
        }
        double turnpower = reachedHeading() ? 0 : gotoHeading(true);

        setMotion(drivepower,strafepower,turnpower);
    }

    public void stop() {
        setMotion(0,0,0);
    }

    public boolean reachedPosition() {
        boolean drive = (Math.abs(drivetarget - getDrivePositions()) <= tolerance);
        boolean strafe = (Math.abs(strafetarget - getStrafePositions()) <= tolerance);
        return (drive && strafe);
    }

    public boolean reachedHeading() {
        return (Math.abs(targetheading - getHeading()) <= 2);
    }

    public int getDrivePositions() {
//        int sum = frontleftmotor.getCurrentPosition() +
//                frontrightmotor.getCurrentPosition() +
//                backleftmotor.getCurrentPosition() +
//                backrightmotor.getCurrentPosition();
//        return (int) (sum/4);
        return (int) (leftwheel.getCurrentPosition() + rightwheel.getCurrentPosition())/2;
    }

    public int getStrafePositions() {
//        int sum = frontleftmotor.getCurrentPosition() +
//                -frontrightmotor.getCurrentPosition() +
//                -backleftmotor.getCurrentPosition() +
//                backrightmotor.getCurrentPosition();
//        return (int) (sum/4);
        return strafewheel.getCurrentPosition();
    }

    public double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetYaw(){
        imu.resetYaw();
    }

    public void updateTelemetry() {
        TelemetryPacket telemetry = new TelemetryPacket();
        telemetry.put("Current Heading", getHeading());
        telemetry.put("Target Heading", targetheading);
        telemetry.put("Current Drive", getDrivePositions());
        telemetry.put("Target Drive", drivetarget);
        telemetry.put("Current Strafe", getStrafePositions());
        telemetry.put("Target Strafe", strafetarget);
        telemetry.put("Error", error);
        telemetry.put("Power", power);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetry);
    }
}


