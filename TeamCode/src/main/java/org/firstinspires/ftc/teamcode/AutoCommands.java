package org.firstinspires.ftc.teamcode;

public class AutoCommands {

    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    public AutoCommands(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
    }

    // Prevent wobble by setting min turn power lower when the robot is moving
    public void move(int drive, int strafe){
        this.driveSubsystem.setTargetPosition(drive,strafe);
        this.driveSubsystem.setTargetHeading(this.driveSubsystem.getHeading());

        while (!this.driveSubsystem.reachedPosition() || !this.driveSubsystem.reachedHeading()) {
            double[] translation = this.driveSubsystem.gotoPosition();
            double turn = this.driveSubsystem.reachedPosition()? this.driveSubsystem.gotoHeading(true) : this.driveSubsystem.gotoHeading(false);
            this.driveSubsystem.setMotion(translation[0], translation[1], turn);

            this.driveSubsystem.updateTelemetry();
        }
        this.driveSubsystem.stop();
    }

    // turn
    public void turn(double degrees){
        this.driveSubsystem.setTargetHeading(degrees);

        while (!this.driveSubsystem.reachedHeading()) {
            double[] translation = this.driveSubsystem.gotoPosition();
            double turn = this.driveSubsystem.gotoHeading(true);
            this.driveSubsystem.setMotion(0, 0, turn);

            this.driveSubsystem.updateTelemetry();
        }
        this.driveSubsystem.stop();
    }

    // intake and drive
    public void intakeball(int drive, int strafe){
        this.driveSubsystem.setTargetPosition(drive,strafe);
        this.driveSubsystem.setTargetHeading(this.driveSubsystem.getHeading());

        while (!this.driveSubsystem.reachedPosition() || !this.driveSubsystem.reachedHeading()) {
            double[] translation = this.driveSubsystem.gotoPosition();
            double turn = this.driveSubsystem.reachedPosition()? this.driveSubsystem.gotoHeading(true) : this.driveSubsystem.gotoHeading(false);
            this.driveSubsystem.setMotion(translation[0], translation[1], turn);
            this.intakeSubsystem.setPower(-0.7, -0.7, true);

            this.intakeSubsystem.updateTelemetry();
            this.driveSubsystem.updateTelemetry();
        }
        this.driveSubsystem.stop();
    }

    // shoot ball and intake when shooter is up to speed
    public void shootball(){
        long start = System.nanoTime();
        while (System.nanoTime() - start <= 5000000) {
            this.shooterSubsystem.setTargetSpeed(1);
            this.shooterSubsystem.enableShooter();

            if (this.shooterSubsystem.atTargetVelocity()) {
                this.intakeSubsystem.setPower(-0.7,-0.7, false);
            } else {
                this.intakeSubsystem.setPower(-0.7,0, true);
            }

            this.intakeSubsystem.updateTelemetry();
            this.shooterSubsystem.updateTelemetry();
        }
    }
}
