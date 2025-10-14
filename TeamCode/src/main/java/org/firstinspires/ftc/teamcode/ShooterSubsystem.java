package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ShooterSubsystem {

    // Motors
    DcMotor flywheel;

    // Servos
    Servo valve;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        // Assign motors from hardware map
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
    }

    int lastpos;
    int position;
    double speed;
    double speed_needed = 10;
    double lasttime;

    // remember to activate last time and pos.

    public void enableShooter() {
        // Rev up motors.... wait for state, etc.
        int position = flywheel.getCurrentPosition();
        double time = System.nanoTime();

        double difference = position - lastpos;
        double timedifference = time - lasttime;

        speed = (difference * 1) / (timedifference / 1000000);

        flywheel.setPower(1.0);

        if (speed > speed_needed) {
            valve.setPosition(0);
        } else {
            valve.setPosition(1);
        }

        lasttime = time;
        lastpos = position;
    }

    public void disableShooter () {
        // Turn off motors...
        flywheel.setPower(0);

    }
}


