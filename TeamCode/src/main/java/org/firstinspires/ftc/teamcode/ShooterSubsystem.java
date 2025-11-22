package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ShooterSubsystem {

    public static int feedSpeed = 700;
    public double defalt_speed = 900;
    public static double speed_needed1 = 1500;
    public static double speed_needed2 = 1550;
    public static double speed_needed3 = 1700;
    public static double speed_needed5 = 1450;
    public double targetflywheelspeed = speed_needed2;

    public static double tolorance = 50;
    public static double kp = 50;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 15;

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
            case 5: targetflywheelspeed = speed_needed5; break;
            default: break;
        }
    }
    public void enableShooter() {
        // Don't run index motors if the wheel is set to its default speed
        flywheel.setVelocityPIDFCoefficients(kp,ki,kd,kf);
        flywheel.setVelocity(targetflywheelspeed);
    }

    public void disableShooter () {
        // Turn off motors...
//        flywheel.setPower(0);
//        indexer.setPower(1);
        updateTelemetry();
    }

    public boolean atTargetVelocity() {
        return (Math.abs(flywheel.getVelocity() - targetflywheelspeed) < tolorance);
    }

    public void runShooter(double power) {
        flywheel.setPower(power);
    }

    public void updateTelemetry() {
        TelemetryPacket telemetry = new TelemetryPacket();
        telemetry.put("flywheel velocity", flywheel.getVelocity());
        telemetry.put("target velicity" , targetflywheelspeed);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetry);
    }
}


