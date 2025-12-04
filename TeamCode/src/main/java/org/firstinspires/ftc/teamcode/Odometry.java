package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Odometry {

    final int ticksperrotation = 2000;
    final double circumfrence = Math.PI*2;
    final double ticksperinch = ticksperrotation / circumfrence;
    final double wheeldistance = 12;

    DcMotorEx leftwheel;
    DcMotorEx rightwheel;
    DcMotorEx strafewheel;
    IMU imu;

    double xpos;
    double ypos;
    double heading;

    double xprev;
    double yprev;

    public Odometry() {
        leftwheel = hardwareMap.get(DcMotorEx.class, "leftwheel");
        rightwheel = hardwareMap.get(DcMotorEx.class, "indexer");
        strafewheel = hardwareMap.get(DcMotorEx.class, "intake");
        imu = hardwareMap.get(IMU.class, "imu");

        xprev = getHorizontal();
        yprev = getVertical();
    }

    // Inches
    public void setPose(double x, double y, double angle) {
        this.xpos = x;
        this.ypos = x;
        this.heading = angle;
    }

    public void updatePose() {
        double xdiff = getHorizontal() - xpos;
        double ydiff = getVertical() - ypos;
        double currentheading = getHeading() * Math.PI / 180;

        double xtranslation = xdiff*Math.cos(currentheading) - ydiff*Math.sin(currentheading);
        double ytranslation = xdiff*Math.sin(currentheading) + ydiff*Math.cos(currentheading);

        xpos += xtranslation;
        ypos += ytranslation;
        heading = getHeading();
    }

    // Returns in Inches
    public double getVertical() {
        return ((double)(leftwheel.getCurrentPosition() + rightwheel.getCurrentPosition())/2) / ticksperinch;

    }

    // Returns in Inches
    public double getHorizontal() {
        return strafewheel.getCurrentPosition() / ticksperinch;
    }

    public double getHeading() {
//        double imuheading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
//        double wheelheading = (ticksperinch*wheeldistance*Math.PI) / ((double)(leftwheel.getCurrentPosition()-rightwheel.getCurrentPosition()) / 2) * 360;
//        return  (imuheading + wheelheading)/2;
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
