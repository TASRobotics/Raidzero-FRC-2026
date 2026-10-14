package frc.lib;

import org.wpilib.command3.Mechanism;

public abstract class R0Subsystem<T extends SubsystemIO> extends Mechanism {
    protected T io;

    /**
     * Creates a new instance of this subsystem

     * @param io the hardware IO to use
     */
    protected R0Subsystem(T io) {
        this.io = io;
    }

    public void periodic() {
        io.updateTelemetry();
    }
}