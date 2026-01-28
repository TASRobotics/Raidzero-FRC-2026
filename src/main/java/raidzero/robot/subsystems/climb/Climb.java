package raidzero.robot.subsystems.climb;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
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
        ranges[0] = new CANrange(1, "climbivore");
        ranges[1] = new CANrange(2, "climbivore");
        ranges[2] = new CANrange(3, "climbivore");
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

    private TalonFXConfiguration winchConfiguration() {
        TalonFXConfiguration configuration = new TalonFXConfiguration()
            .withOpenLoopRamps(
                new OpenLoopRampsConfigs()
                    .withDutyCycleOpenLoopRampPeriod(0.1)
            );

        return configuration;
    }

    private CANrangeConfiguration canrangeConfiguration() {
        CANrangeConfiguration configuration = new CANrangeConfiguration()
            .withFovParams(new FovParamsConfigs());

        return configuration;
    }

    public static Climb system() {
        if (system == null) {
            system = new Climb();
        }

        return system;
    }
}