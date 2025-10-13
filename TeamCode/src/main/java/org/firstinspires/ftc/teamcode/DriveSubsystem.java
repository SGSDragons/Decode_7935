package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class DriveSubsystem {
    PIDCoefficients pidCoefficients = new PIDCoefficients();

    DcMotor frontleftmotor;
    DcMotor frontrightmotor;
    DcMotor backleftmotor;
    DcMotor backrightmotor;

    int tickstoinches = 100;
    int tickstodegrees = 100;
    int sdfsdfa;

    DriveSubsystem(HardwareMap hardwareMap){
        frontleftmotor = hardwareMap.get(DcMotor.class, "frontleft");
        frontrightmotor = hardwareMap.get(DcMotor.class, "frontright");
        backleftmotor = hardwareMap.get(DcMotor.class, "backleft");
        backrightmotor = hardwareMap.get(DcMotor.class, "backright");

        frontleftmotor.setDirection(DcMotor.Direction.REVERSE);
        frontrightmotor.setDirection(DcMotor.Direction.REVERSE);
        backleftmotor.setDirection(DcMotor.Direction.FORWARD);
        backrightmotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void drive(double distance) {
        frontleftmotor.setTargetPosition((int)(distance*tickstoinches));
        frontrightmotor.setTargetPosition((int)(distance*tickstoinches));
        backleftmotor.setTargetPosition((int)(distance*tickstoinches));
        backrightmotor.setTargetPosition((int)(distance*tickstoinches));
    }

    public void strafe(double distance) {
        frontleftmotor.setTargetPosition((int)(-distance*tickstoinches));
        frontrightmotor.setTargetPosition((int)(distance*tickstoinches));
        backleftmotor.setTargetPosition((int)(distance*tickstoinches));
        backrightmotor.setTargetPosition((int)(-distance*tickstoinches));
    }

    public void turn(double degrees) {
        frontleftmotor.setTargetPosition((int)(-degrees*tickstodegrees));
        frontrightmotor.setTargetPosition((int)(degrees*tickstodegrees));
        backleftmotor.setTargetPosition((int)(-degrees*tickstodegrees));
        backrightmotor.setTargetPosition((int)(degrees*tickstodegrees));
    }

    public void runtoPosition() {
        frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runwithEncoder() {
        frontleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double drive, double strafe, double turn) {
        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontleftmotor.setPower(frontLeftPower);
        frontrightmotor.setPower(frontRightPower);
        backleftmotor.setPower(backLeftPower);
        backrightmotor.setPower(backRightPower);
    }
}
