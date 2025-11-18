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

//    public void moveIntake(int distance){
//        int currentpos = intakemotor.getCurrentPosition();
//        int targetpos = currentpos + distance;
//
//        intakemotor.setTargetPosition(targetpos);
//        intakemotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        intakemotor.setPower(intakepower);
//    }
//
//    public boolean reachedIndexPosition() {
//        return (Math.abs(indexmotor.getTargetPosition()-indexmotor.getCurrentPosition()) <= 3);
//    }
//
//    public void moveIndexer(int distance) {
//        int currentpos = indexmotor.getCurrentPosition();
//        int targetpos = currentpos + distance;
//
//        indexmotor.setTargetPosition(targetpos);
//        indexmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        indexmotor.setPower(indexpower);
//    }

    public void setIntakePower(double power){
        intakemotor.setPower(power);
    }

    // Only stop the index motor when the ball hits the limit if enableLimit is true
    public void setIndexPower(double power, boolean enableLimit){
        if (limit.getState() && !enableLimit){
            indexmotor.setPower(power);
        } else {
            indexmotor.setPower(0);
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
