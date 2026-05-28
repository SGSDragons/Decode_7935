package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="TestShooting", group="Linear OpMode")
@Config
public class TestShooting extends LinearOpMode{

    ElapsedTime runtime = new ElapsedTime();
    DcMotorEx intakemotor;
    DcMotorEx indexmotor;
    DcMotorEx leftflywheel;
    DcMotorEx rightflywheel;

    double targetflywheelspeed;
    double shootingspeed1;
    double shootingspeed2;
    double shootingspeed3;

    @Override
    public void runOpMode(){

        targetflywheelspeed = 500;
        shootingspeed1 = 1000;
        shootingspeed2 = 1250;
        shootingspeed3 = 1500;

        intakemotor = hardwareMap.get(DcMotorEx.class,"intake");
        indexmotor = hardwareMap.get(DcMotorEx.class,"indexer");

        intakemotor.setDirection(DcMotor.Direction.FORWARD);
        indexmotor.setDirection(DcMotor.Direction.REVERSE);

        leftflywheel = hardwareMap.get(DcMotorEx.class, "leftflywheel");
        leftflywheel.setDirection(DcMotor.Direction.REVERSE);

        rightflywheel = hardwareMap.get(DcMotorEx.class, "rightflywheel");
        rightflywheel.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){

            leftflywheel.setVelocityPIDFCoefficients(200,0,20,20);
            rightflywheel.setVelocityPIDFCoefficients(200,0,20,20);

            if (gamepad2.a) {
                leftflywheel.setVelocity(shootingspeed1);
                rightflywheel.setVelocity(shootingspeed1);
            }
            else if (gamepad2.x) {
                leftflywheel.setVelocity(shootingspeed2);
                rightflywheel.setVelocity(shootingspeed2);
            }
            else if (gamepad2.y) {
                leftflywheel.setVelocity(shootingspeed3);
                rightflywheel.setVelocity(shootingspeed3);
            }
            else {
                leftflywheel.setVelocity(targetflywheelspeed);
                rightflywheel.setVelocity(targetflywheelspeed);
            }

            double intakepower = gamepad2.right_bumper ? 0 : -gamepad2.left_stick_y;
            double indexpower = gamepad2.left_bumper ? 0 : -gamepad2.left_stick_y;

            intakemotor.setPower(intakepower);
            indexmotor.setPower(indexpower);

            telemetry.update();
        }
    }
}
