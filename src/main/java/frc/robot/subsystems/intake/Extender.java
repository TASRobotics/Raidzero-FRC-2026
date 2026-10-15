package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.Rotations;

import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Trigger;
import org.wpilib.system.Timer;

import frc.lib.R0Subsystem;

public class Extender extends R0Subsystem<ExtenderIO> {
    private static Extender system;

    /**
     * Extends the intake outwards
     *
     * @return A {@link Command} that extends the intake
     */
    public Command extend() {
        return runRepeatedly(() -> io.extend())
            .named("Extender extend");
    }

    /**
     * Retracts the intake inwards
     *
     * @return A {@link Command} that retracts the intake
     */
    public Command retract() {
        return runRepeatedly(() -> io.retract())
            .named("Extender retract");
    }

    /**
     * Agitates the intake at level 1 for 0.2s
     *
     * @return A {@link Command} that agitates the intake
     */
    public Command agitate() {
        return run(coroutine -> {
            Timer timer = new Timer();
            timer.start();
            while (timer.get() < 0.1) {
                io.agitateL1();
                coroutine.yield();
            }

            while (true) {
                io.retract();
                coroutine.yield();
            }
        }).named("Intake agitate");
    }

    /**
     * Agitates the intake during auto by by cycling through all 3 agitation levels
     *
     * @apiNote This should only be used during auto
     *
     * @return A {@link Command} that agitates the intake during auto
     */
    public Command autoAgitate() {
        return run(coroutine -> {
            step(coroutine, io::agitateL1, 0.5);
            step(coroutine, io::extend,    0.5);
            step(coroutine, io::agitateL2, 0.5);
            step(coroutine, io::agitateL1, 0.5);
            step(coroutine, io::agitateL3, 0.5);
            step(coroutine, io::agitateL2, 0.5);
            step(coroutine, io::extend,    0.3);
        }).named("Intake agitate");
    }

    /**
     * Agitates the intake at level 1
     * @return A {@link Command} that moves the extender to the level 1 agitate setpoint
     */
    public Command agitateL1Setpoint() {
        return runRepeatedly(() -> io.agitateL1())
            .named("Extender agitate L1");
    }

    /**
     * Agitates the intake at level 2
     * @return A {@link Command} that moves the extender to the level 2 agitate setpoint
     */
    public Command agitateL2Setpoint() {
        return runRepeatedly(() -> io.agitateL2())
            .named("Extender agitate L2");
    }

    /**
     * Agitates the intake at level 3
     * @return A {@link Command} that moves the extender to the level 3 agitate setpoint
     */
    public Command agitateL3Setpoint() {
        return runRepeatedly(() -> io.agitateL3())
            .named("Extender agitate L3");
    }

    /**
     * Stops the extender motors
     *
     * @return A {@link Command} that stops the extender
     */
    public Command stop() {
        return runRepeatedly(() -> io.stop())
            .named("Extender stop");
    }

    /**
     * Returns a {@link Trigger} that is active when the extender is fully extended
     *
     * @return a {@link Trigger} that is active when the extender is fully extended
     */
    public Trigger isExtended() {
        return new Trigger(() -> io.isExtended());
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
        return run(coroutine -> io.increaseAgitate())
            .named("Extender increase agitate");
    }

    /**
     * Decreases the agitate level of the extender
     *
     * @return A {@link Command} that decreases the agitate level of the extender
     */
    public Command decreaseAgitate() {
        return run(coroutine -> io.decreaseAgitate())
            .named("Extender decrease agitate");
    }

    /**
     * Zeros the encoder position for the extender
     *
     * @return A {@link Command} that zeros the encoder position
     */
    public Command zeroEncoderPosition() {
        return run(coroutine -> io.overrideEncoderPosition(Rotations.of(0.0)))
            .named("extender zero encoder");
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
            if (com.ctre.phoenix6.Utils.isSimulation())
                system = new Extender(new ExtenderIO.Sim());
            else
                system = new Extender(new ExtenderIO.Real());
        }
        return system;
    }

    private void step(Coroutine coroutine, Runnable action, double seconds) {
        Timer timer = new Timer();
        timer.start();
        while (timer.get() < seconds) {
            action.run();
            coroutine.yield();
        }
    }
}
