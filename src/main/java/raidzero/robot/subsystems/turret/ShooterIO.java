package raidzero.robot.subsystems.turret;

import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static raidzero.robot.subsystems.turret.TurretConstants.Shooter.*;

import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO extends SubsystemIO {

    void setVelocity(AngularVelocity velocity);

    void setPercent(double speedPercent);

    void stop();

    public class Real implements ShooterIO {
        LazyTalon flywheel = new LazyTalon(MOTOR_ID, SENSOR_TO_MECHANISM_RATIO, INVERTED, STATOR_CURRENT_LIMIT.in(Amps),
                SUPPLY_CURRENT_LIMIT.in(Amps))
                .withMotionMagicConfiguration(P, I, D, 0, 0, 0, 0, 0, 0, GravityTypeValue.Elevator_Static, 0,
                        MAX_ACCELERATION.in(RotationsPerSecondPerSecond))
                .build();

        @Override
        public void setVelocity(AngularVelocity velocity) {
            flywheel.setVelocity(velocity);
        }

        @Override
        public void setPercent(double speedPercent) {
            flywheel.setPercent(MOTOR_ID);
        }

        @Override
        public void stop() {
            flywheel.stop();
        }

        @Override
        public void updateTelemetry() {
            flywheel.updateTelemetry(TELEMETRY);
        }

    }

    public class Sim extends Real {
    }
}