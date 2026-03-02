package raidzero.lib;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class R0Subsystem<T extends SubsystemIO> extends SubsystemBase {
    protected T io;

    /**
     * Creates a new instance of this subsystem

     * @param io the hardware IO to use
     */
    protected R0Subsystem(T io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateTelemetry();
    }
}