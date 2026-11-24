package raidzero.lib.hardware;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

/** A {@link MotorIO} for tests: records what it was told and reports what the test sets. */
public class FakeMotorIO implements MotorIO {
    public boolean connected = true;
    public double positionRotations = 0;
    public Angle lastGoal;
    public Voltage lastVoltage;
    public boolean stopped;

    @Override
    public void updateInputs(Inputs inputs) {
        inputs.connected = connected;
        inputs.positionRotations = positionRotations;
    }

    @Override
    public void setPositionGoal(Angle position) {
        lastGoal = position;
        stopped = false;
    }

    @Override
    public void setVoltage(Voltage voltage) {
        lastVoltage = voltage;
    }

    @Override
    public void stop() {
        stopped = true;
    }

    @Override
    public void setPosition(Angle position) {
        positionRotations = position.in(Rotations);
    }
}
