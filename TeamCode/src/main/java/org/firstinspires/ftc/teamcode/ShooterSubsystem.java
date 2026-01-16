package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ShooterSubsystem {

    public static int feedSpeed = 700;
    public double defalt_speed = 900;
    public static double speed_needed1 = 1100;
    public static double speed_needed2 = 1400;
    public static double speed_needed3 = 1400;
    public double targetflywheelspeed = speed_needed2;

    public static double tolorance = 50;
    public static double kp = 200;
    public static double ki = 0;
    public static double kd = 20;
    public static double kf = 20;

    // Motors
    DcMotorEx leftflywheel;
    DcMotorEx rightflywheel;
    DcMotorEx indexer;
    DcMotorEx intake;


    public ShooterSubsystem(HardwareMap hardwareMap) {
        // Assign motors from hardware map
        leftflywheel = hardwareMap.get(DcMotorEx.class, "leftflywheel");
        leftflywheel.setDirection(DcMotor.Direction.REVERSE);

        rightflywheel = hardwareMap.get(DcMotorEx.class, "rightflywheel");
        rightflywheel.setDirection(DcMotor.Direction.REVERSE);

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
        // set both motors to the same pidf coeficcients and set the same targetvelocity;
        leftflywheel.setVelocityPIDFCoefficients(kp,ki,kd,kf);
        rightflywheel.setVelocityPIDFCoefficients(kp,ki,kd,kf);

        leftflywheel.setVelocity(targetflywheelspeed);
        rightflywheel.setVelocity(targetflywheelspeed);
    }

    public void disableShooter () {
        // Turn off motors...
//        flywheel.setPower(0);
//        indexer.setPower(1);
        updateTelemetry();
    }

    public boolean atTargetVelocity() {
        return (Math.abs(leftflywheel.getVelocity() - targetflywheelspeed) < tolorance && targetflywheelspeed != defalt_speed);
    }

    public boolean shooterisEnabled() {
        return (targetflywheelspeed != defalt_speed);
    }

    public void runShooter(double power) {
        leftflywheel.setPower(power);
        rightflywheel.setPower(power);
    }

    public void updateTelemetry() {
        TelemetryPacket telemetry = new TelemetryPacket();
        telemetry.put("flywheel velocity", leftflywheel.getVelocity());
        telemetry.put("target velicity" , targetflywheelspeed);
        FtcDashboard.getInstance().sendTelemetryPacket(telemetry);
    }
}


