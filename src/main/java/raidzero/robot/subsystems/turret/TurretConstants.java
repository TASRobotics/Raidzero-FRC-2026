package raidzero.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.HashMap;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import raidzero.lib.wrappers.motors.LazyMotor.MotorTelemetry;

public class TurretConstants {
    public static final class Yaw {
        public static final int MOTOR_ID = 10;

        public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);
        public static final Time SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.0);

        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.0;
        public static final double G = 0.0; // * NOTE: this should be 0
        public static final double V = 0.0;
        public static final double A = 0.0;

        // * NOTE: This should start at 10 and go down to increase speed
        public static final double EXPO_V = 10.0;
        public static final double EXPO_A = 10.0;

        public static final Angle FORWARD_SOFT_LIMIT = Rotations.of(1.0);
        public static final Angle REVERSE_SOFT_LIMIT = Rotations.of(-1.0);

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(1.0);
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(1.0);

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static final class Pitch {
        public static final int MOTOR_ID = 11;

        public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);
        public static final Time SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.0);

        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.0;
        public static final double G = 0.0; // * NOTE: this should be 0
        public static final double V = 0.0;
        public static final double A = 0.0;

        // * NOTE: This should start at 10 and go down to increase speed
        public static final double EXPO_V = 10.0;
        public static final double EXPO_A = 10.0;

        public static final Angle FORWARD_SOFT_LIMIT = Rotations.of(1.0);
        public static final Angle REVERSE_SOFT_LIMIT = Rotations.of(-1.0);

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(1.0);
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(1.0);

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static final class Shooter {
        public static final int MOTOR_ID = 12;

        public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);
        public static final Time SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.0);
        
        public static final AngularVelocity MAX_SHOOTER_VELOCITY = RotationsPerSecond.of(100);

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static final class Limelight {

    }

    public static final class Setpoints {
        
    }

    public class ShooterSetting {
        public Angle pitch, yaw;
        public AngularVelocity shooterVelocity;

        public LinearVelocity xExitVelocity, yExitVelocity;

    }

}
