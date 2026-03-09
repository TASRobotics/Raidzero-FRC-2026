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

        public static final double JMM_OFFSET_1 = -0.499512; // 29 tooth
        public static final double JMM_OFFSET_2 = -0.685791; // 22 tooth

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

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(0.5); // max is 3.2
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(0.5); // max is 10

        public static final Angle FORWARD_SOFT_LIMIT = Rotations.of(1.0);
        public static final boolean ENALBE_FORWARD_SOFT_LIMIT = true;

        public static final Angle REVERSE_SOFT_LIMIT = Rotations.of(-0.5);
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

        public static final Angle FORWARD_SOFT_LIMIT = Degrees.of(35.0);
        public static final boolean ENALBE_FORWARD_SOFT_LIMIT = true;

        public static final Angle REVERSE_SOFT_LIMIT = Rotations.of(0.0275);
        public static final boolean ENABLE_REVERSE_SOFT_LIMIT = true;

        public static final Angle STARTING_ANGLE = Degrees.of(10.0);

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
        public static final Translation2d TURRET_MOUNT_OFFSET = new Translation2d(Meters.of(.123), Meters.of(-0.138));

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
            PITCH_TABLE.put(1.58, 14.0);
            PITCH_TABLE.put(2.05, 16.0);
            PITCH_TABLE.put(2.47, 19.0);
            PITCH_TABLE.put(2.94, 20.0);
            PITCH_TABLE.put(3.48, 22.0);
            PITCH_TABLE.put(3.97, 23.0);
            PITCH_TABLE.put(4.48, 26.0);
            PITCH_TABLE.put(5.08, 27.0);
            PITCH_TABLE.put(5.5, 28.0);

            RPS_TABLE.put(1.58, 30.0);
            RPS_TABLE.put(2.05, 31.0);
            RPS_TABLE.put(2.47, 31.0);
            RPS_TABLE.put(2.94, 32.0);
            RPS_TABLE.put(3.48, 35.0);
            RPS_TABLE.put(3.97, 35.5);
            RPS_TABLE.put(4.48, 35.5);
            RPS_TABLE.put(5.08, 37.5);
            RPS_TABLE.put(5.5, 38.5);

            TOF_TABLE.put(1.878, 1.156);
            TOF_TABLE.put(2.427, 1.254);
            TOF_TABLE.put(2.792, 1.28);
            TOF_TABLE.put(3.640, 1.304);
            TOF_TABLE.put(4.117, 1.334);
            TOF_TABLE.put(4.615, 1.37955);
            TOF_TABLE.put(5.093, 1.4127);
        }
    }

}
