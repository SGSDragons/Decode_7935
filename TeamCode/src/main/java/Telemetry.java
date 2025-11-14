import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.teamcode.DriveSubsystem;
import org.firstinspires.ftc.teamcode.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.ShooterSubsystem;

@Config
public class Telemetry {

    DriveSubsystem driveSubsystem;
    IntakeSubsystem intakeSubsystem;
    ShooterSubsystem shooterSubsystem;

    public Telemetry(DriveSubsystem driveSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
        driveSubsystem = this.driveSubsystem;
        intakeSubsystem = this.intakeSubsystem;
        shooterSubsystem = this.shooterSubsystem;
    }

    public void updateTelemetryPacket() {
        TelemetryPacket telemetry = new TelemetryPacket();

        FtcDashboard.getInstance().sendTelemetryPacket(telemetry);
    }
}
