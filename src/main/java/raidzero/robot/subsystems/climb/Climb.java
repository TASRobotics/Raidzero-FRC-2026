package raidzero.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Climb implements Subsystem {
    private TalonFX winch;
    private CANrange[] ranges;

    private static Climb system;

    private Climb() {
        winch = new TalonFX(0, "climbivore");
        winch.getConfigurator().apply(winchConfiguration());
        winch.setNeutralMode(NeutralModeValue.Brake);

        ranges = new CANrange[3];

        for (int i = 0; i < 3; i++) {
            ranges[i] = new CANrange(i + 1, "climbivore");
            ranges[i].getConfigurator().apply(canrangeConfig(i));
        }
    }

    public Command printStates() {
        return defer(() -> runOnce(() -> {
            System.out.println("isleftof " + this.isLeftOf());
            System.out.println("is centered " + this.isCentered());
            System.out.println("is rightof " + this.isRightOf());
            System.out.println("left dist " + ranges[0].getDistance());
            System.out.println("center dist " + ranges[1].getDistance());
            System.out.println("right dist " + ranges[2].getDistance());
        }));
    }

    public Command run(double speed) {
        return run(() -> winch.set(speed));
    }

    public Command stop() {
        return run(() -> winch.stopMotor());
    }

    public CANrange[] getRanges() {
        return ranges;
    }

    public boolean isLeftOf() {
        return ranges[0].getIsDetected().getValue() && ranges[1].getIsDetected().getValue();
    }

    public boolean isRightOf() {
        return ranges[1].getIsDetected().getValue() && ranges[2].getIsDetected().getValue();
    }

    public boolean isCentered() {
        return !ranges[0].getIsDetected().getValue() && ranges[1].getIsDetected().getValue() && !ranges[2].getIsDetected().getValue();
    }

    private TalonFXConfiguration winchConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration()
            .withOpenLoopRamps(
                new OpenLoopRampsConfigs()
                    .withDutyCycleOpenLoopRampPeriod(0.1)
            );

        return configuration;
    }

    private CANrangeConfiguration canrangeConfig(int idx) {
        int fovCenterX = idx == 0 ? 11 : (idx == 1 ? 0 : -11);

        CANrangeConfiguration configuration = new CANrangeConfiguration()
            .withFovParams(
                new FovParamsConfigs()
                    .withFOVCenterX(fovCenterX)
                    .withFOVCenterY(0)
                    .withFOVRangeX(6.75)
                    .withFOVRangeY(6.75)
            )
            .withProximityParams(
                new ProximityParamsConfigs()
                    .withProximityThreshold(0.5)
            )
            .withToFParams(
                new ToFParamsConfigs()
                    .withUpdateMode(UpdateModeValue.LongRangeUserFreq)
                    .withUpdateFrequency(50)
            );

        return configuration;
    }

    public static Climb system() {
        if (system == null) {
            system = new Climb();
        }

        return system;
    }
}