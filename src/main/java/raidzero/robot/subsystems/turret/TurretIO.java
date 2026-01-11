package raidzero.robot.subsystems.turret;

import edu.wpi.first.units.measure.Angle;
import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static raidzero.robot.subsystems.turret.TurretConstants.Yaw;

import com.ctre.phoenix6.signals.GravityTypeValue;

public interface TurretIO extends SubsystemIO {

    Angle getYaw();

    Angle getPitch();

    Angle moveTo(Angle pitch, Angle Yaw);

    public class Real implements TurretIO {
        private LazyTalon yaw, pitch;

        public Real() {
            yaw = new LazyTalon(Yaw.MOTOR_ID, Yaw.SENSOR_TO_MECHANISM_RATIO, Yaw.INVERTED,
                    Yaw.STATOR_CURRENT_LIMIT.in(Amps),
                    Yaw.SUPPLY_CURRENT_LIMIT.in(Amps))
                    .withMotionMagicConfiguration(Yaw.P, Yaw.I, Yaw.D, Yaw.S, Yaw.G, Yaw.V, Yaw.A, Yaw.EXPO_V,
                            Yaw.EXPO_A, GravityTypeValue.Elevator_Static, Yaw.CRUISE_VELOCITY.in(RotationsPerSecond),
                            Yaw.ACCELERATION.in(RotationsPerSecondPerSecond))
                    .build();
        }

        @Override
        public Angle getYaw() {
            return yaw.getFeedbackPosition();
        }

        @Override
        public Angle getPitch() {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'getPitch'");
        }

        @Override
        public Angle moveTo(Angle pitch, Angle Yaw) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'moveTo'");
        }

        @Override
        public void updateTelemetry() {
            yaw.updateTelemetry(Yaw.TELEMETRY);
        }
    }

    public class Sim extends Real {
    }
}
