package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeSubsystem {

    DcMotor intakemotor;
    DcMotor indexmotor;

    double indexpower = 0.3;
    double intakepower = 0.7;

    public IntakeSubsystem(HardwareMap hardwareMap){
        intakemotor = hardwareMap.get(DcMotor.class,"intake");
        indexmotor = hardwareMap.get(DcMotor.class,"indexer");
    }

    public void moveIntake(int distance){
        int currentpos = intakemotor.getCurrentPosition();
        int targetpos = currentpos + distance;

        intakemotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intakemotor.setTargetPosition(targetpos);
        intakemotor.setPower(intakepower);
    }

    public void moveIndexer(int distance) {
        int currentpos = indexmotor.getCurrentPosition();
        int targetpos = currentpos + distance;

        indexmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        indexmotor.setTargetPosition(targetpos);
        indexmotor.setPower(indexpower);
    }

    public void setPower(double intakepower, double indexpower){
        intakemotor.setPower(-intakepower);
        indexmotor.setPower(indexpower);
    }
}
