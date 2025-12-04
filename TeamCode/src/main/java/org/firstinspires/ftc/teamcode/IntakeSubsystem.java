package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    DcMotorEx intakemotor;
    DcMotorEx indexmotor;
    DigitalChannel limit;

//    public static double indexpower = 0.7;
//    public static double intakepower = 0.7;

    public IntakeSubsystem(HardwareMap hardwareMap){
        intakemotor = hardwareMap.get(DcMotorEx.class,"intake");
        indexmotor = hardwareMap.get(DcMotorEx.class,"indexer");

        intakemotor.setDirection(DcMotor.Direction.FORWARD);
        indexmotor.setDirection(DcMotor.Direction.FORWARD);

        limit = hardwareMap.get(DigitalChannel.class, "limit");
        limit.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isLoaded() {
        // getState == true -> A ball is pressing the switch
        return !limit.getState();
    }

    public void runIntake() {
        // Running the intake means grabbing balls from the floor and running
        // the indexer wheel to make room, but only if the limit switch isn't down
        intakemotor.setPower(-1.0);
        if (!isLoaded()) {
            indexmotor.setPower(-0.5);
        } else {
            indexmotor.setPower(0.0);
        }
    }

    public void stopIntake() {
        intakemotor.setPower(0.0);
        indexmotor.setPower(0.0);
    }

    public void runIndexer(boolean flywheelReady) {
        // Running the indexer means we're shooting. Don't waste energy on the
        // intake motor and feed as fast as possible if the flywheel is ready.
        intakemotor.setPower(0.0);
        if (flywheelReady) {
            indexmotor.setPower(-1.0);
        } else {
            indexmotor.setPower(0.0);
        }
    }

    public void setIntakePower(double power){
        intakemotor.setPower(power);
    }

    // Only stop the index motor when the ball hits the limit if enableLimit is true
    public void setIndexPower(double power, boolean enableLimit){
        if (enableLimit && isLoaded()) {
            indexmotor.setPower(0);
        } else {
            indexmotor.setPower(power);
        }
    }

    public void setPower(double intakepower, double indexpower, boolean enableLimit){
        setIntakePower(intakepower);
        setIndexPower(indexpower, enableLimit);
    }

    public void updateTelemetry() {
        TelemetryPacket telemetry = new TelemetryPacket();
        telemetry.put("Intake Motor Velocity", intakemotor.getVelocity());
        telemetry.put("Index Motor Velocity", indexmotor.getVelocity());
        telemetry.put("Limit Switch", limit.getState());
        FtcDashboard.getInstance().sendTelemetryPacket(telemetry);
    }
}
