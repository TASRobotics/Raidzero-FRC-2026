package raidzero.robot.subsystems.intake;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;

import raidzero.lib.R0Subsystem;

public class Intake extends R0Subsystem<IntakeIO> {
    private static Intake system;

    public Command go() {
        return run(() -> io.go(IntakeConstants.Intake.INTAKE_SPEED));
    }

    public Command stop() {
        return run(() -> io.stop());
    }

    private Intake(IntakeIO io) {
        super(io);
    }

    public static Intake system() {
        if (system == null) {
            if (Utils.isSimulation())
                system = new Intake(new IntakeIO.Sim());
            else
                system = new Intake(new IntakeIO.Real());
        }
        return system;
    }

}
