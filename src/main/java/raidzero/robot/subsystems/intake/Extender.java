package raidzero.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.R0Subsystem;

public class Extender extends R0Subsystem<ExtenderIO> {
    private static Extender system;

    public Command extend() {
        return run(() -> io.extend());
    }

    public Command retract() {
        return run(() -> io.retract());
    }

    public Command stop() {
        return run(() -> io.stop());
    }

    public Trigger isExtended() {
        return new Trigger(() -> io.isExtended());
    }

    public void setStartupAngle() {
        io.setStartupAngle();
    }

    private Extender(ExtenderIO io) {
        super(io);
    }

    public static Extender system() {
        if (system == null) {
            if (com.ctre.phoenix6.Utils.isSimulation())
                system = new Extender(new ExtenderIO.Sim());
            else
                system = new Extender(new ExtenderIO.Real());
        }
        return system;
    }

}
