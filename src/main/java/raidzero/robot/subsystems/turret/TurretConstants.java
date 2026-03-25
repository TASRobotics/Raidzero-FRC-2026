package raidzero.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import raidzero.lib.wrappers.motors.LazyCTRE.MotorTelemetry;

public final class TurretConstants {
    public static final class Yaw {
        public static final int MOTOR_ID = 14;
        public static final String CAN_BUS_NAME = "canDoAttitude";
        public static final MotorArrangementValue MOTOR_ARRANGEMENT = MotorArrangementValue.Minion_JST;

        public static final double SENSOR_TO_MECHANISM_RATIO = 280.0 / 10.0; // old ratio: 250.0 / 10.0;
        public static final double TURRET_RING_TEETH = 280.0;
        public static final double JARED_TEETH_1 = 29.0;
        public static final double JARED_TEETH_2 = 22.0;

        public static final int JMM_1_ID = 6; // 29 tooth
        public static final int JMM_2_ID = 5; // 22 tooth

        public static final double JMM_OFFSET_1 = -0.574463; // 29 tooth
        public static final double JMM_OFFSET_2 = -0.622559; // 22 tooth

        public static final SensorDirectionValue JMM_1_INVERTED = SensorDirectionValue.Clockwise_Positive;
        public static final SensorDirectionValue JMM_2_INVERTED = SensorDirectionValue.Clockwise_Positive;

        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(30);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);

        public static final double POSITION_P = 50.0;
        public static final double POSITION_I = 0.0;
        public static final double POSITION_D = 0.0;

        public static final double POSITION_S = 0.25;
        public static final double POSITION_G = 0.0; // * NOTE: this should be 0
        public static final double POSITION_V = 2.6;
        public static final double POSITION_A = 0.0;

        public static final GravityTypeValue FF_GRAVITY_TYPE = GravityTypeValue.Elevator_Static; // * Note: no gravity
                                                                                                 // (yippee!)
        public static final StaticFeedforwardSignValue FF_SIGN = StaticFeedforwardSignValue.UseVelocitySign;

        public static final int POSITION_SLOT = 0;

        public static final double EXPO_V = 3.0;
        public static final double EXPO_A = 3.0;

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(1.0); // max is 3.2
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(1.0); // max is 10

        public static final Angle FORWARD_SOFT_LIMIT = Rotations.of(1.16);
        public static final boolean ENALBE_FORWARD_SOFT_LIMIT = true;

        public static final Angle REVERSE_SOFT_LIMIT = Rotations.of(-0.4);
        public static final boolean ENABLE_REVERSE_SOFT_LIMIT = true;

        public static MotorTelemetry TELEMETRY = new MotorTelemetry();

        public static final double TOLERANCE_SRT = 0.5;
    }

    public static final class Pitch {
        public static final int MOTOR_ID = 13;
        public static final String CAN_BUS_NAME = "canDoAttitude";
        public static final MotorArrangementValue MOTOR_ARRANGEMENT = MotorArrangementValue.Minion_JST;

        public static final double SENSOR_TO_MECHANISM_RATIO = (42.0 / 10.0) * (42.0 / 14.0) * (25.0 / 16.0) * (205.0 / 12.0);
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40.0);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);

        public static final double P = 400.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.23;
        public static final double G = 0.2;
        public static final double V = 3.2;
        public static final double A = 0.0;

        public static final GravityTypeValue FF_GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;
        public static final StaticFeedforwardSignValue FF_SIGN = StaticFeedforwardSignValue.UseVelocitySign;

        public static final int POSITION_SLOT = 0;

        // * NOTE: This should start at 10 and go down to increase speed
        public static final double EXPO_V = 3.2;
        public static final double EXPO_A = 1.0;

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(0.29); // 48.375
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(5.0);

        public static final Angle FORWARD_SOFT_LIMIT = Rotations.of(0.098);
        public static final boolean ENALBE_FORWARD_SOFT_LIMIT = true;

        public static final Angle REVERSE_SOFT_LIMIT = Rotations.of(0.0275);
        public static final boolean ENABLE_REVERSE_SOFT_LIMIT = true;

        public static final Angle STARTING_ANGLE = Degrees.of(10.0);
        public static final Angle PASSING_ANGLE = Degrees.of(34.5);

        public static MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    // public static final class Limelight {
    // public static final Angle MOUNT_ANGLE = Degrees.of(5.0);

    // public static final Time LATENCY = Seconds.of(0.040); // Make constant? Get
    // from NetworkTables? Physically test?

    // public static final String LIMELIGHT_NAME = "limelight-turret";

    // public static final int[] RED_ALLIANCE_TAG_IDS = { 10, 11, 8 };
    // public static final int[] BLUE_ALLIANCE_TAG_IDS = { 25, 27, 24 };

    // public static final int[] STARTUP_REFERENCE_TAG_IDS = { 2, 8, 24, 18 };

    // public static final Distance HUB_HEIGHT = Inches.of(44.25);

    // public static final Distance CAMERA_HEIGHT = Meters.of(0.77);
    // }

    public static record TurretAim(Pose2d fieldRelativeTurretPose, Pose2d fieldRelativeGoalPose) {}

    public static final class Kinetics {
        public static final Translation2d TURRET_MOUNT_OFFSET = new Translation2d(Meters.of(-0.123), Meters.of(-0.138));

        // Convert robot m/s to Shooter RPS
        // Scalar formula: 1 / (Shooter_Wheel_Diameter_Meters * PI)
        // 1 / (0.1016 * 3.14) ≈ 3.13
        public static final double VELOCITY_COMPENSATION_SCALAR = 1 / (Inches.of(3.125).in(Meters) * Math.PI);

        // yaw corections from the april tag location in the off-angles
        public static final InterpolatingDoubleTreeMap YAW_CORRECTION_TABLE = new InterpolatingDoubleTreeMap();

        public static final InterpolatingDoubleTreeMap PITCH_TABLE = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap RPS_TABLE = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap TOF_TABLE = new InterpolatingDoubleTreeMap();

        public static final InterpolatingDoubleTreeMap CLIMB_PITCH_TABLE = new InterpolatingDoubleTreeMap();
        public static final InterpolatingDoubleTreeMap CLIMB_RPS_TABLE = new InterpolatingDoubleTreeMap();

        public static final Translation2d GOAL_LOCATION_BLUE = new Translation2d(Inches.of(182.11), Inches.of(158.84));
        public static final Translation2d GOAL_LOCATION_RED = new Translation2d(Inches.of(465.89), Inches.of(158.84));

        static {
            // Updated PITCH_TABLE from supplied data
            PITCH_TABLE.put(1.55, 12.0);
            PITCH_TABLE.put(2.05, 13.0);
            PITCH_TABLE.put(2.53, 15.0);
            PITCH_TABLE.put(2.98, 15.0);
            PITCH_TABLE.put(3.4, 16.0);
            PITCH_TABLE.put(3.65, 16.0);
            PITCH_TABLE.put(4.00, 18.0);
            PITCH_TABLE.put(4.2, 19.0);
            PITCH_TABLE.put(4.50, 20.0);
            PITCH_TABLE.put(5.43, 25.0);

            // Updated RPS_TABLE from supplied data
            RPS_TABLE.put(1.55, 24.5);
            RPS_TABLE.put(2.05, 26.5);
            RPS_TABLE.put(2.53, 27.0);
            RPS_TABLE.put(2.98, 28.0);
            RPS_TABLE.put(3.65, 30.0);
            RPS_TABLE.put(4.00, 29.5);
            RPS_TABLE.put(4.2, 29.5);
            RPS_TABLE.put(4.50, 31.0);
            RPS_TABLE.put(5.43, 31.5);

            TOF_TABLE.put(1.5, 1.393);
            TOF_TABLE.put(2.0, 1.554);
            TOF_TABLE.put(2.5, 1.557);
            TOF_TABLE.put(3.0, 1.56);
            TOF_TABLE.put(3.65, 1.634);
            TOF_TABLE.put(4.0, 1.64);
            TOF_TABLE.put(4.5, 1.654);
        }
    }

}
