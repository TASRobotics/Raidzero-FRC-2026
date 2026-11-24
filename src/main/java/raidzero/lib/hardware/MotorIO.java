package raidzero.lib.hardware;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * The seam between robot code and one motor (with its followers). Every read goes through {@link
 * Inputs}, which AdvantageKit records and can replay; robot code must not read the device any other
 * way, or replay will differ from what happened.
 *
 * <p>Everything is in mechanism rotations. Implementations do the gearing.
 *
 * <p>Every method has a do-nothing default, so {@code new MotorIO() {}} is the replay
 * implementation: no hardware, and Inputs come from the log.
 */
public interface MotorIO {
    /** Reads the device into {@code inputs}. Called once per loop, before anything else. */
    default void updateInputs(Inputs inputs) {}

    /** Moves to {@code position} using the onboard closed loop and profile. */
    default void setPositionGoal(Angle position) {}

    /** Drives open-loop, e.g. for characterization. Device soft limits still apply. */
    default void setVoltage(Voltage voltage) {}

    /** Stops driving; the motor brakes or coasts per its neutral mode. */
    default void stop() {}

    /** Tells the device it is at {@code position} now, without moving. Used for zeroing. */
    default void setPosition(Angle position) {}

    /**
     * Simulation only: tells the device where the simulated mechanism is, so the onboard control
     * loop runs on simulated feedback. Archetypes call this from their physics sim.
     */
    default void simulate(Angle position, AngularVelocity velocity) {}

    /** What the device reports each loop. Written by hand because the AutoLog processor isn't set up. */
    final class Inputs implements LoggableInputs {
        public boolean connected = false;
        public boolean followersConnected = false;
        public double positionRotations = 0;
        public double velocityRotationsPerSec = 0;
        /** Where the onboard profile wants the mechanism right now. */
        public double closedLoopReferenceRotations = 0;

        public double closedLoopErrorRotations = 0;
        public double appliedVolts = 0;
        public double statorCurrentAmps = 0;
        public double supplyCurrentAmps = 0;
        public double temperatureCelsius = 0;

        @Override
        public void toLog(LogTable table) {
            table.put("Connected", connected);
            table.put("FollowersConnected", followersConnected);
            table.put("PositionRotations", positionRotations);
            table.put("VelocityRotationsPerSec", velocityRotationsPerSec);
            table.put("ClosedLoopReferenceRotations", closedLoopReferenceRotations);
            table.put("ClosedLoopErrorRotations", closedLoopErrorRotations);
            table.put("AppliedVolts", appliedVolts);
            table.put("StatorCurrentAmps", statorCurrentAmps);
            table.put("SupplyCurrentAmps", supplyCurrentAmps);
            table.put("TemperatureCelsius", temperatureCelsius);
        }

        @Override
        public void fromLog(LogTable table) {
            connected = table.get("Connected", connected);
            followersConnected = table.get("FollowersConnected", followersConnected);
            positionRotations = table.get("PositionRotations", positionRotations);
            velocityRotationsPerSec = table.get("VelocityRotationsPerSec", velocityRotationsPerSec);
            closedLoopReferenceRotations = table.get("ClosedLoopReferenceRotations", closedLoopReferenceRotations);
            closedLoopErrorRotations = table.get("ClosedLoopErrorRotations", closedLoopErrorRotations);
            appliedVolts = table.get("AppliedVolts", appliedVolts);
            statorCurrentAmps = table.get("StatorCurrentAmps", statorCurrentAmps);
            supplyCurrentAmps = table.get("SupplyCurrentAmps", supplyCurrentAmps);
            temperatureCelsius = table.get("TemperatureCelsius", temperatureCelsius);
        }
    }
}
