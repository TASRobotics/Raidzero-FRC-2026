package raidzero.robot.subsystems.turret;

import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static raidzero.robot.subsystems.turret.TurretConstants.Indexer.ACCELERATION;
import static raidzero.robot.subsystems.turret.TurretConstants.Indexer.CRUISE_VELOCITY;
import static raidzero.robot.subsystems.turret.TurretConstants.Shooter.*;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.AngularVelocity;

public interface ShooterIO extends SubsystemIO {

    void setVelocity(AngularVelocity velocity);

    void setPercent(double speedPercent);

    void stop();

    AngularVelocity getVelocity();

    public class Real implements ShooterIO {
        LazyTalon flywheel = new LazyTalon(MOTOR_ID, CAN_BUS_NAME, SENSOR_TO_MECHANISM_RATIO, INVERTED,
                STATOR_CURRENT_LIMIT.in(Amps),
                SUPPLY_CURRENT_LIMIT.in(Amps))
                .withPIDFConfiguration(P, I, D, S, G, V, A, GravityTypeValue.Elevator_Static,
                        StaticFeedforwardSignValue.UseClosedLoopSign, 0)
                .withMotionMagicConfiguration(0, 0, CRUISE_VELOCITY.in(RotationsPerSecond),
                        ACCELERATION.in(RotationsPerSecondPerSecond))
                .build();

        @Override
        public void setVelocity(AngularVelocity velocity) {
            this.flywheel.setVelocity(velocity, 0);
        }

        @Override
        public void setPercent(double speedPercent) {
            flywheel.setPercent(MOTOR_ID);
        }

        @Override
        public void stop() {
            flywheel.stop();
        }

        public AngularVelocity getVelocity() {
            return flywheel.getFeedbackVelocity();
        }

        @Override
        public void updateTelemetry() {
            flywheel.updateTelemetry(TELEMETRY);
        }

    }

    public class Sim extends Real {
    }

}
