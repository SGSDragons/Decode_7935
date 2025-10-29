package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ShooterSubsystem {

    // Motors
    DcMotorEx flywheel;
    DcMotorEx mover;
    DcMotorEx intake;

    // Servos


    public ShooterSubsystem(HardwareMap hardwareMap) {
        // Assign motors from hardware map
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        mover = hardwareMap.get(DcMotorEx.class, "indexer");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        flywheel.setDirection(DcMotor.Direction.REVERSE);
    }

    double speed;
    int lastpos;
    double lasttime;
    public static int setback;
    public static int setforward;

    public static int tolerance;
    public static double speed_needed;
    public static double speed_needed2;
    public static double speed_needed3;
    public double targetflywheelspeed = speed_needed2;

     Integer moverismoved;


    // remember to activate last time and pos.

    public void setTargetSpeed(int selection) {
        switch(selection) {
            case 1: targetflywheelspeed = speed_needed; break;
            case 2: targetflywheelspeed = speed_needed2; break;
            case 3: targetflywheelspeed = speed_needed3; break;
            default: break;
        }
    }
    public void enableShooter() {

        // Rev up motors.... wait for state, etc.
        int position = flywheel.getCurrentPosition();
        double time = System.nanoTime();

        double difference = position - lastpos;
        double timedifference = time - lasttime;
        speed = (difference * 1) / (timedifference / 1000000);

        if (moverismoved == null) {
            moverismoved = mover.getCurrentPosition() - setback;
            mover.setTargetPosition(moverismoved);
        }

        if (Math.abs(mover.getCurrentPosition() - moverismoved) < tolerance) {
            flywheel.setVelocity(targetflywheelspeed);
        } else {
            flywheel.setPower(0);
        }



        if (speed > targetflywheelspeed) {
            mover.setTargetPosition(mover.getCurrentPosition() + setforward);
        } else {
        }

        lasttime = time;
        lastpos = position;

        updateTelemetry();
    }

    public void disableShooter () {
        // Turn off motors...
        flywheel.setPower(0);
        mover.setPower(1);
        updateTelemetry();
        moverismoved = null;
    }

    public void updateTelemetry() {
        TelemetryPacket telemetry = new TelemetryPacket();
        telemetry.put("flywheel.speed", speed);
        telemetry.put("setback", setback);
        telemetry.put("targt_speed" , targetflywheelspeed);
        telemetry.put("setforward", setforward);
    }

    public void intake() {
        intake.setPower(1);
    }

    public void runShooter(double power) {
        flywheel.setPower(power);
    }
}


