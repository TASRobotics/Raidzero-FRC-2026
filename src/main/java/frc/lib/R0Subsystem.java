package frc.lib;

import org.wpilib.command2.SubsystemBase;

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
        // io.updateTelemetry();
    }
}