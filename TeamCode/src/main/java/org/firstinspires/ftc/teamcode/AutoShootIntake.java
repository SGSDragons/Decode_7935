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

    public class ShootThree implements Action {

        private ElapsedTime timer;
        RoadRunnerDemo.State state = RoadRunnerDemo.State.LOADING;
        int shotsFired = 0;

        int loadedTests = 0;
        int unloadedTests = 0;

        private RoadRunnerDemo.State deduceCurrentState() {
            if (state == RoadRunnerDemo.State.FIRING && !intake.isLoaded()) {
                unloadedTests += 1;
                if (unloadedTests < 10) {
                    // Don't change states until we're pretty sure the limit switch
                    // is staying up.
                    return state;
                }
                // It was firing, but the ball isn't there anymore
                shotsFired += 1;

                if (shotsFired == 3) {
                    // All shots have fired
                    return RoadRunnerDemo.State.FINISHED;
                } else {
                    // More shots to fire. Go back to loading with a new time limit
                    loadedTests = 0;
                    timer.reset();
                    return RoadRunnerDemo.State.LOADING;
                }
            }

            if (state == RoadRunnerDemo.State.LOADING && timer.time() > 3000.0) {
                // Loading is taking too long. Give up. Maybe no more balls
                return RoadRunnerDemo.State.FINISHED;
            }

            if (state == RoadRunnerDemo.State.LOADING && intake.isLoaded()) {
                // Change from LOADING to FIRING because a ball is in position
                loadedTests += 1;
                if (loadedTests > 10) {
                    unloadedTests = 0;
                    return RoadRunnerDemo.State.FIRING;
                }
            }

            // No important changes happened, so stay in the current state
            return state;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            if (timer == null) {
                timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            }
            p.put("Shots Fired", shotsFired);
            intake.updateTelemetry(p);

            state = deduceCurrentState();

            p.put("Shooter:", shotsFired + " of 3... " + state.toString());
            intake.updateTelemetry(p);

            switch (state) {
                case FINISHED:
                    intake.stopIntake();
                    return false;
                case LOADING:
                    intake.runIntake();
                    return true;
                case FIRING:
                    intake.runIndexer(shooter.atTargetVelocity());
                    return true;
                default:
                    return true;
            }
        }
    }

    Action runIntake = (p) -> {intake.runIntake(); return true; };
    Action stopIntake = (p) -> {intake.stopIntake(); return false; };

}