package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoShootIntake{
    IntakeSubsystem intake;
    ShooterSubsystem shooter;

    public AutoShootIntake(HardwareMap hardwareMap) {
        this.intake = new IntakeSubsystem(hardwareMap);
        this.shooter = new ShooterSubsystem(hardwareMap);
    }

    public class ShootThreeClose implements Action {

        private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        boolean first = true;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (first) {
                timer.reset();
                first = false;
            }

            if (timer.time() > 1500.0) {
                return false;
            }

            else {
                intake.runIndexerClose(shooter.atTargetVelocity());
                return true;
            }
        }
    }

    public class ShootThreeFar implements Action {

        private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);;

        @Override
        public boolean run(@NonNull TelemetryPacket p) {

            if (timer.time() > 1500.0) {
                return false;
            }

            else {
                intake.runIndexerFar(shooter.atTargetVelocity());
                return true;
            }
        }
    }

    Action runIntake = (p) -> {intake.runIntake(); return true; };
    Action stopIntake = (p) -> {intake.stopIntake(); return false; };

}