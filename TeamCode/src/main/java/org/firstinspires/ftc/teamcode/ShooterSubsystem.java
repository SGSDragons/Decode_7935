package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ShooterSubsystem {

    public static int feedSpeed = 700;
    public static double defalt_speed = 1400;
    public static double speed_needed1 = 1600;
    public static double speed_needed2 = 1800;
    public static double speed_needed3 = 2100;
    public double targetflywheelspeed = speed_needed2;
    Integer moverismoved;

    // Motors
    DcMotorEx flywheel;
    DcMotorEx indexer;
    DcMotorEx intake;


    public ShooterSubsystem(HardwareMap hardwareMap) {
        // Assign motors from hardware map
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        indexer = hardwareMap.get(DcMotorEx.class, "indexer");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void setTargetSpeed(int selection) {
        switch(selection) {
            case 0: targetflywheelspeed = defalt_speed; break;
            case 1: targetflywheelspeed = speed_needed1; break;
            case 2: targetflywheelspeed = speed_needed2; break;
            case 3: targetflywheelspeed = speed_needed3; break;
            default: break;
        }
    }
    public void enableShooter() {

        // Don't run index motors if the wheel is set to its default speed
        flywheel.setVelocity(targetflywheelspeed);
        if (atTargetVelocity() && targetflywheelspeed != defalt_speed) {
            indexer.setVelocity(-feedSpeed);
        }

        updateTelemetry();
    }

    public void disableShooter () {
        // Turn off motors...
//        flywheel.setPower(0);
//        indexer.setPower(1);
        updateTelemetry();
        moverismoved = null;
    }

    public boolean atTargetVelocity() {
        return (Math.abs(flywheel.getVelocity() - targetflywheelspeed) < 100);
    }

    public void runShooter(double power) {
        flywheel.setPower(power);
    }

    public void updateTelemetry() {
        TelemetryPacket telemetry = new TelemetryPacket();
        telemetry.put("flywheel.speed", flywheel.getVelocity());
        telemetry.put("target.speed" , targetflywheelspeed);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetry);
    }
}


