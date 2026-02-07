package raidzero.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import raidzero.lib.wrappers.motors.LazyMotor.MotorTelemetry;

public final class ShooterConstants {
    public static final class Flywheel {
        public static final int MOTOR_ID = 12;
        public static final String CAN_BUS_NAME = "canShoot";

        public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);

        public static final double P = 0.22;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.25;
        public static final double G = 0.0;
        public static final double V = 0.107;
        public static final double A = 0.0;

        public static final GravityTypeValue FF_GRAVITY_TYPE = GravityTypeValue.Elevator_Static; // * Note: no gravity
                                                                                                 // (yippee!)
        public static final StaticFeedforwardSignValue FF_SIGN = StaticFeedforwardSignValue.UseVelocitySign;

        public static final int VELOCITY_SLOT = 0;

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(0);
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(0);

        public static final int FOLLOW_ID = 13;
        public static final MotorAlignmentValue FOLLOWER_INVERTED = MotorAlignmentValue.Opposed;

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static final class Indexer {
        public static final int MOTOR_ID = 15;
        public static final String CAN_BUS_NAME = "canShoot";

        public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;

        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(50);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(30);

        public static final double P = 0.295;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.25;
        public static final double G = 0.0;
        public static final double V = 0.127;
        public static final double A = 0.0;

        public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Elevator_Static;

        // * NOTE: This should start at 10 and go down to increase speed
        public static final double EXPO_V = 10.0;
        public static final double EXPO_A = 10.0;

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(94.0); // 94 @ 12 V
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(188.0); // increasable

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }
}