package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    DcMotor intakemotor;
    DcMotor indexmotor;

    public static double indexpower = 0.7;
    public static double intakepower = 0.7;

    public IntakeSubsystem(HardwareMap hardwareMap){
        intakemotor = hardwareMap.get(DcMotor.class,"intake");
        indexmotor = hardwareMap.get(DcMotor.class,"indexer");

        intakemotor.setDirection(DcMotor.Direction.REVERSE);
        indexmotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void moveIntake(int distance){
        int currentpos = intakemotor.getCurrentPosition();
        int targetpos = currentpos + distance;

        intakemotor.setTargetPosition(targetpos);
        intakemotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakemotor.setPower(intakepower);
    }

    public boolean reachedIndexPosition() {
        return (Math.abs(indexmotor.getTargetPosition()-indexmotor.getCurrentPosition()) <= 3);
    }

    public void moveIndexer(int distance) {
        int currentpos = indexmotor.getCurrentPosition();
        int targetpos = currentpos + distance;

        indexmotor.setTargetPosition(targetpos);
        indexmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexmotor.setPower(indexpower);
    }

    public void setIntakePower(double power){
        if (intakemotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            intakemotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        intakemotor.setPower(power);
    }

    public void setIndexPower(double power){
        if (indexmotor.getMode() != DcMotor.RunMode.RUN_WITHOUT_ENCODER){
            indexmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        indexmotor.setPower(power);
    }

    public void setPower(double intakepower, double indexpower){
        intakemotor.setPower(intakepower);
        indexmotor.setPower(indexpower);
    }
}
