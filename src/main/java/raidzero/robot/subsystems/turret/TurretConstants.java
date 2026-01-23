package raidzero.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import raidzero.lib.wrappers.motors.LazyMotor.MotorTelemetry;

public class TurretConstants {
    public static final class Yaw {
        public static final int MOTOR_ID = 10;
        public static final String CAN_BUS_NAME = "canShoot";

        public static final double SENSOR_TO_MECHANISM_RATIO = 250.0 / 10.0;
        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(30);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);
        public static final Time SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.0);

        public static final double P = 20.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.23;
        public static final double G = 0.0; // * NOTE: this should be 0
        public static final double V = 0.0;
        public static final double A = 0.0;

        // * NOTE: This should start at 10 and go down to increase speed
        public static final double EXPO_V = 10.0;
        public static final double EXPO_A = 10.0;

        public static final Angle FORWARD_SOFT_LIMIT = Rotations.of(0.5);
        public static final Angle REVERSE_SOFT_LIMIT = Rotations.of(-0.55);

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(3.0); // max is 3.2
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(5.0); // max is 10

        public static final int CANCODER_ID = 10;
        public static final FeedbackSensorSourceValue CANCODER_TYPE = FeedbackSensorSourceValue.FusedCANcoder;

        public static final double MAGNET_OFFSET = 0;
        public static final SensorDirectionValue CANCODER_SENSOR_DIRECTION = SensorDirectionValue.CounterClockwise_Positive;

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static final class Pitch {
        public static final int MOTOR_ID = 11;
        public static final String CAN_BUS_NAME = "canShoot";

        public static final double SENSOR_TO_MECHANISM_RATIO = (27.0 / 15.0) * (215.0 / 8.0);
        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(30.0);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);
        public static final Time SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.0);

        public static final double P = 70.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.35;
        public static final double G = 0.25;
        public static final double V = 0.0;
        public static final double A = 0.0;

        public static final GravityTypeValue GRAVITY_TYPE = GravityTypeValue.Elevator_Static;

        // * NOTE: This should start at 10 and go down to increase speed
        public static final double EXPO_V = 10.0;
        public static final double EXPO_A = 10.0;

        public static final Angle STARTING_ANGLE = Degrees.of(10.0);

        public static final Angle FORWARD_SOFT_LIMIT = Rotations.of(0.1);
        public static final Angle REVERSE_SOFT_LIMIT = Rotations.of(0.0);

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(0.6); // 48.375
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(5.0);

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static final class Shooter {
        public static final int MOTOR_ID = 12;
        public static final int FOLLOW_ID = 13;
        public static final String CAN_BUS_NAME = "canShoot";

        public static final double P = 0.22;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.0;
        public static final double G = 0.0;
        public static final double V = 0.112;
        public static final double A = 0.0;

        public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(7000);
        public static final double MAX_JERK = 0.0;

        public static final double SENSOR_TO_MECHANISM_RATIO = 1.0;
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);
        public static final Time SUPPLY_CURRENT_LOWER_TIME = Seconds.of(0.0);

        public static final AngularVelocity MAX_SHOOTER_VELOCITY = RotationsPerSecond.of(100);

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
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(188.0); //increasable

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static final class Limelight {
        public static final Angle MOUNT_ANGLE = Degrees.of(45.0);

        public static final Time LATENCY = Seconds.of(0.040); // Make constant? Get from NetworkTables? Physically test?

        public static final String LIMELIGHT_NAME = "turret";

        public static final int[] RED_ALLIANCE_TAG_IDS = { 10, 11, 8 };
        public static final int[] BLUE_ALLIANCE_TAG_IDS = { 25, 27, 24 };

        public static final int[] STARTUP_REFERENCE_TAG_IDS = { 2, 8, 24, 18 };
    }

    public static final class Setpoints {
        // yaw corections from the april tag location in the off-angles
        public static final InterpolatingDoubleTreeMap YAW_CORRECTION_TABLE = new InterpolatingDoubleTreeMap();

        public static final InterpolatingDoubleTreeMap PITCH_TABLE = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap RPS_TABLE = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap TOF_TABLE = new InterpolatingDoubleTreeMap();

        public static final InterpolatingDoubleTreeMap CLIMB_PITCH_TABLE = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap CLIMB_RPS_TABLE = new InterpolatingDoubleTreeMap();

        public static final Translation2d TURRET_OFFSET = new Translation2d(0, 0);
    }

}
