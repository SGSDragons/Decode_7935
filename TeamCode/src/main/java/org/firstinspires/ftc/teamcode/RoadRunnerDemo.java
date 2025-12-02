package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.rr.Localizer;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;

@TeleOp(name = "RoadRunner")
public class RoadRunnerDemo extends LinearOpMode {

    IntakeSubsystem intake;
    ShooterSubsystem shooter;

    public enum State {
        LOADING, // Getting a ball into position
        FIRING,  // Moving the ball to the flywheel
        FINISHED // Completed doing everything
    }
    class ShootThree implements Action {

        private ElapsedTime timer;
        State state = State.LOADING;
        int shotsFired = 0;

        private State deduceCurrentState() {
            if (state == State.FIRING && !intake.isLoaded()) {
                // It was firing, but the ball isn't there anymore
                shotsFired += 1;
                if (shotsFired == 3) {
                    // All shots have fired
                    return State.FINISHED;
                } else {
                    // More shots to fire. Go back to loading with a new time limit
                    timer.reset();
                    return State.LOADING;
                }
            }

            if (state == State.LOADING && timer.time() > 3000.0) {
                // Loading is taking too long. Give up. Maybe no more balls
                return State.FINISHED;
            }

            if (state == State.LOADING && intake.isLoaded()) {
                // Change from LOADING to FIRING because a ball is in position
                return State.FIRING;
            }

            // No important changes happened, so stay in the current state
            return state;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (timer == null) {
                timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            }

            state = deduceCurrentState();

            telemetry.addData("Shooter:", shotsFired + " of 3... " + state.toString());
            telemetry.update();

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

    Action runIntake = (p) -> {intake.runIntake(); return false; };
    Action stopIntake = (p) -> {intake.stopIntake(); return false; };

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d init = new Pose2d(-52.0, 50.0, Math.toRadians(135));
        final MecanumDrive drive = new MecanumDrive(hardwareMap, init);
        this.intake = new IntakeSubsystem(hardwareMap);
        this.shooter = new ShooterSubsystem(hardwareMap);

        waitForStart();
        shooter.setTargetSpeed(1);
        shooter.enableShooter();

        Pose2d firingPoint = new Pose2d(-30, 29, Math.toRadians(135));
        Pose2d firstBallRow = new Pose2d(-9, 25, Math.toRadians(90));

        TrajectoryActionBuilder autoMode = drive.actionBuilder(init);

        // Backup and take the first shot
        autoMode = autoMode
                .setReversed(true)
                .splineToLinearHeading(firingPoint, Math.toRadians(180.0)) // Tangent points backwards along the route
                .stopAndAdd(new ShootThree());

        // Move to the first row of balls and grab them
        autoMode = autoMode
                .splineToLinearHeading(firstBallRow, 0)
                .stopAndAdd(runIntake)
                .setTangent(Math.PI/2)
                .lineToYConstantHeading(60.0, new TranslationalVelConstraint(15))
                .stopAndAdd(stopIntake);

        // Return to the shooting position and shoot again.
        autoMode = autoMode
                .lineToYLinearHeading(15.0, Math.toRadians(105))
                .setTangent(Math.toRadians(135))
                .splineToLinearHeading(firingPoint, Math.toRadians(45))
                .stopAndAdd(new ShootThree());

        // Move to the second row of balls and grab them
        autoMode = autoMode
                .turnTo(Math.toRadians(-30))
                .splineToLinearHeading(new Pose2d(14, 25, 0), 0)
                .turnTo(Math.PI/2)
                .stopAndAdd(runIntake)
                .lineToYConstantHeading(60.0, new TranslationalVelConstraint(15))
                .stopAndAdd(stopIntake);

        // Return to the shooting position and shoot again
        autoMode = autoMode
                .setReversed(true)
                .splineTo(new Vector2d(-5, 15), Math.toRadians(45))
                .turnTo(Math.toRadians(135))
                .splineToLinearHeading(new Pose2d(-30, 29, Math.toRadians(135)), Math.toRadians(45))
                .stopAndAdd(new ShootThree());

        Actions.runBlocking(autoMode.build());

        while(opModeIsActive()) {
            // Stall for assessment
        }
    }
}
