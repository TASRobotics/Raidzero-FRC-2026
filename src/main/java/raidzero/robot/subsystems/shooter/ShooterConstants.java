package raidzero.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import raidzero.lib.wrappers.motors.LazyCTRE.MotorTelemetry;

public final class ShooterConstants {
    public static final class Flywheel {
        public static final int MOTOR_ID = 10;
        public static final String CAN_BUS_NAME = "canDoAttitude";

        public static final double SENSOR_TO_MECHANISM_RATIO = 2.0;
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(80); // 50
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(60);

        public static final double P = 5.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.2;
        public static final double G = 0.0;
        public static final double V = 0.0;
        public static final double A = 0.0;

        public static final double TORQUE_FF_A = 13.0;

        public static final GravityTypeValue FF_GRAVITY_TYPE = GravityTypeValue.Elevator_Static; // * Note: no gravity
                                                                                                 // (yippee!)
        public static final StaticFeedforwardSignValue FF_SIGN = StaticFeedforwardSignValue.UseVelocitySign;

        public static final int VELOCITY_SLOT = 0;

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(100.0);
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(0);

        public static final int FOLLOW_ID = 11;
        public static final MotorAlignmentValue FOLLOWER_INVERTED = MotorAlignmentValue.Opposed;

        public static final AngularVelocity VELOCIOTY_TOLERANCE = RotationsPerSecond.of(5.0);

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static final class Feeder {
        public static final int MOTOR_ID = 12;
        public static final String CAN_BUS_NAME = "canDoAttitude";

        public static final double SENSOR_TO_MECHANISM_RATIO = 60.0 / 26.0;

        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(50);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);

        public static final double P = 1.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.28;
        public static final double G = 0.0;
        public static final double V = 0.27;
        public static final double A = 0.0;

        public static final GravityTypeValue FF_GRAVITY_TYPE = GravityTypeValue.Elevator_Static; // * Note: no gravity
                                                                                                 // (yippee!)
        public static final StaticFeedforwardSignValue FF_SIGN = StaticFeedforwardSignValue.UseVelocitySign;

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(43.3);
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(0.0);

        public static final AngularVelocity FEEDER_SPEED = RotationsPerSecond.of(40.0);

        public static final AngularVelocity VELOCITY_TOLERANCE = RotationsPerSecond.of(5.0);

        public static MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static final class Canrange {

        public static final int CAN_ID = 1;
        public static final String CAN_BUS_NAME = "canDoAttitude";

        public static final double FOV_CENTER_X = 0;
        public static final double FOV_CENTER_Y = 0;
        public static final double FOV_RANGE_X = 6.75;
        public static final double FOV_RANGE_Y = 6.75;

        public static final double PROXIMITY_THRESHOLD = 0.13;

        public static final UpdateModeValue UPDATE_MODE = UpdateModeValue.LongRangeUserFreq;
        public static final double UPDATE_FREQUENCY = 50;

        public static final double DETECTION_WINDOW = 0.25;
    }

}