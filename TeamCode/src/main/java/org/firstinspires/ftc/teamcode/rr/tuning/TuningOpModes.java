package org.firstinspires.ftc.teamcode.rr.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.AngularRampLogger;
import com.acmerobotics.roadrunner.ftc.DeadWheelDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.DriveType;
import com.acmerobotics.roadrunner.ftc.DriveView;
import com.acmerobotics.roadrunner.ftc.DriveViewFactory;
import com.acmerobotics.roadrunner.ftc.EncoderGroup;
import com.acmerobotics.roadrunner.ftc.EncoderRef;
import com.acmerobotics.roadrunner.ftc.ForwardPushTest;
import com.acmerobotics.roadrunner.ftc.ForwardRampLogger;
import com.acmerobotics.roadrunner.ftc.LateralPushTest;
import com.acmerobotics.roadrunner.ftc.LateralRampLogger;
import com.acmerobotics.roadrunner.ftc.LazyImu;
import com.acmerobotics.roadrunner.ftc.LynxQuadratureEncoderGroup;
import com.acmerobotics.roadrunner.ftc.ManualFeedforwardTuner;
import com.acmerobotics.roadrunner.ftc.MecanumMotorDirectionDebugger;
import com.acmerobotics.roadrunner.ftc.OTOSAngularScalarTuner;
import com.acmerobotics.roadrunner.ftc.OTOSHeadingOffsetTuner;
import com.acmerobotics.roadrunner.ftc.OTOSLinearScalarTuner;
import com.acmerobotics.roadrunner.ftc.OTOSPositionOffsetTuner;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;
import org.firstinspires.ftc.teamcode.rr.MecanumDrive;
import org.firstinspires.ftc.teamcode.rr.ThreeDeadWheelLocalizer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public final class TuningOpModes {

    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = true;

    private TuningOpModes() {}

    private static OpModeMeta metaForClass(Class<? extends OpMode> cls) {
        return new OpModeMeta.Builder()
                .setName(cls.getSimpleName())
                .setGroup(GROUP)
                .setFlavor(OpModeMeta.Flavor.TELEOP)
                .build();
    }

    @OpModeRegistrar
    public static void register(OpModeManager manager) {
        if (DISABLED) return;

        DriveViewFactory dvf = hardwareMap -> {
            MecanumDrive md = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
            LazyImu lazyImu = md.lazyImu;

            List<EncoderGroup> encoderGroups = new ArrayList<>();
            List<EncoderRef> leftEncs = new ArrayList<>();
            List<EncoderRef> rightEncs = new ArrayList<>();

            ThreeDeadWheelLocalizer dl = (ThreeDeadWheelLocalizer) md.localizer;
            encoderGroups.add(new LynxQuadratureEncoderGroup(
                    hardwareMap.getAll(LynxModule.class),
                    Arrays.asList(dl.par0, dl.par1, dl.perp)
            ));

            List<EncoderRef> parEncs = List.of(
                    new EncoderRef(0, 0),
                    new EncoderRef(0, 1));
            List<EncoderRef> perpEncs = List.of(new EncoderRef(0, 2));

            return new DriveView(
                    DriveType.MECANUM,
                    MecanumDrive.PARAMS.inPerTick,
                    MecanumDrive.PARAMS.maxWheelVel,
                    MecanumDrive.PARAMS.minProfileAccel,
                    MecanumDrive.PARAMS.maxProfileAccel,
                    encoderGroups,
                    Arrays.asList(
                            md.leftFront,
                            md.leftBack
                    ),
                    Arrays.asList(
                            md.rightFront,
                            md.rightBack
                    ),
                    leftEncs,
                    rightEncs,
                    parEncs,
                    perpEncs,
                    lazyImu,
                    md.voltageSensor,
                    () -> new MotorFeedforward(MecanumDrive.PARAMS.kS,
                            MecanumDrive.PARAMS.kV / MecanumDrive.PARAMS.inPerTick,
                            MecanumDrive.PARAMS.kA / MecanumDrive.PARAMS.inPerTick),
                    0
            );
        };

        manager.register(metaForClass(AngularRampLogger.class), new AngularRampLogger(dvf));
        manager.register(metaForClass(ForwardPushTest.class), new ForwardPushTest(dvf));
        manager.register(metaForClass(ForwardRampLogger.class), new ForwardRampLogger(dvf));
        manager.register(metaForClass(LateralPushTest.class), new LateralPushTest(dvf));
        manager.register(metaForClass(LateralRampLogger.class), new LateralRampLogger(dvf));
        manager.register(metaForClass(ManualFeedforwardTuner.class), new ManualFeedforwardTuner(dvf));
        manager.register(metaForClass(MecanumMotorDirectionDebugger.class), new MecanumMotorDirectionDebugger(dvf));
        manager.register(metaForClass(DeadWheelDirectionDebugger.class), new DeadWheelDirectionDebugger(dvf));

        manager.register(metaForClass(ManualFeedbackTuner.class), ManualFeedbackTuner.class);
        manager.register(metaForClass(SplineTest.class), SplineTest.class);
        manager.register(metaForClass(LocalizationTest.class), LocalizationTest.class);

        manager.register(metaForClass(OTOSAngularScalarTuner.class), new OTOSAngularScalarTuner(dvf));
        manager.register(metaForClass(OTOSLinearScalarTuner.class), new OTOSLinearScalarTuner(dvf));
        manager.register(metaForClass(OTOSHeadingOffsetTuner.class), new OTOSHeadingOffsetTuner(dvf));
        manager.register(metaForClass(OTOSPositionOffsetTuner.class), new OTOSPositionOffsetTuner(dvf));

        FtcDashboard.getInstance().withConfigRoot(configRoot -> {
            for (Class<?> c : Arrays.asList(
                    AngularRampLogger.class,
                    ForwardRampLogger.class,
                    LateralRampLogger.class,
                    ManualFeedforwardTuner.class,
                    MecanumMotorDirectionDebugger.class,
                    ManualFeedbackTuner.class
            )) {
                configRoot.putVariable(c.getSimpleName(), ReflectionConfig.createVariableFromClass(c));
            }
        });
    }
}