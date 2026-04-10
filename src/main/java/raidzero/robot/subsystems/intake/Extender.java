package raidzero.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.R0Subsystem;

public class Extender extends R0Subsystem<ExtenderIO> {
    private static Extender system;

    /**
     * Extends the intake outwards
     *
     * @return A {@link Command} that extends the intake
     */
    public Command extend() {
        return run(() -> io.extend());
    }

    /**
     * Retracts the intake inwards
     *
     * @return A {@link Command} that retracts the intake
     */
    public Command retract() {
        return run(() -> io.retract());
    }

    /**
     * Agitates the intake at level 1 for 0.2s
     *
     * @return A {@link Command} that agitates the intake
     */
    public Command agitate() {
        return run(() -> io.agitateL1())
            .withTimeout(0.2)
            .andThen(run(() -> io.retract()));
    }

    /**
     * Agitates the intake during auto by by cycling through all 3 agitation levels
     *
     * @apiNote This should only be used during auto
     *
     * @return A {@link Command} that agitates the intake during auto
     */
    public Command autoAgitate() {
        return run(() -> io.agitateL1())
            .withTimeout(0.5)
            .andThen(
                run(() -> io.extend())
                    .withTimeout(0.5)
            )
            .andThen(
                run(() -> io.agitateL2())
                    .withTimeout(0.5)
            )
            .andThen(
                run(() -> io.agitateL1())
                    .withTimeout(0.5)
            )
            .andThen(
                run(() -> io.agitateL3())
                    .withTimeout(0.5)
            )
            .andThen(
                run(() -> io.agitateL2())
                    .withTimeout(0.5)
            )
            .andThen(run(() -> io.extend()).withTimeout(0.3));
    }

    /**
     * Agitates the intake at level 1
     * @return A {@link Command} that moves the extender to the level 1 agitate setpoint
     */
    public Command agitateL1Setpoint() {
        return run(() -> io.agitateL1());
    }

    /**
     * Agitates the intake at level 2
     * @return A {@link Command} that moves the extender to the level 2 agitate setpoint
     */
    public Command agitateL2Setpoint() {
        return run(() -> io.agitateL2());
    }

    /**
     * Agitates the intake at level 3
     * @return A {@link Command} that moves the extender to the level 3 agitate setpoint
     */
    public Command agitateL3Setpoint() {
        return run(() -> io.agitateL3());
    }

    /**
     * Stops the extender motors
     *
     * @return A {@link Command} that stops the extender
     */
    public Command stop() {
        return run(() -> io.stop());
    }

    /**
     * Returns a {@link Trigger} that is active when the extender is fully extended
     *
     * @return a {@link Trigger} that is active when the extender is fully extended
     */
    public Trigger isExtended() {
        return new Trigger(() -> io.isExtended());
    }

    public Command lowAgitate(){
        return this.agitateL1Setpoint().withTimeout(0.3).andThen(this.extend().withTimeout(0.3));

    }

    public Command midAgitate(){
        return this.agitateL2Setpoint().withTimeout(0.3).andThen(this.agitateL1Setpoint().withTimeout(0.3));
    }

    public Command highAgitate(){
        return this.agitateL3Setpoint().withTimeout(0.3).andThen(this.agitateL2Setpoint().withTimeout(0.3));
    }

    /**
     * Sets the starting angle for the intake
     */
    public void setStartupAngle() {
        io.setStartupAngle();
    }

    /**
     * Increases the agitate level of the extender
     *
     * @return A {@link Command} that increases the agitate level of the extender
     */
    public Command increaseAgitate() {
        return runOnce(() -> io.increaseAgitate());
    }

    /**
     * Decreases the agitate level of the extender
     *
     * @return A {@link Command} that decreases the agitate level of the extender
     */
    public Command decreaseAgitate() {
        return runOnce(() -> io.decreaseAgitate());
    }

    /**
     * Zeros the encoder position for the extender
     *
     * @return A {@link Command} that zeros the encoder position
     */
    public Command zeroEncoderPosition() {
        return runOnce(() -> io.overrideEncoderPosition(Rotations.of(0.0)));
    }

    /**
     * Constructs the {@link Extender} subsystem
     *
     * @param io The {@link ExtenderIO} implementation to use for this subsystem
     */
    private Extender(ExtenderIO io) {
        super(io);
    }

    /**
     * Gets the {@link Extender} subsystem instance
     *
     * @return The {@link Extender} subsystem instance
     */
    public static Extender system() {
        if (system == null) {
            if (com.ctre.phoenix6.Utils.isSimulation()){
                system = new Extender(new ExtenderIO.Sim());
                SmartDashboard.putString("Extender State", "Initialized");
            }
            else
                system = new Extender(new ExtenderIO.Real());
        }
        return system;
    }
}
