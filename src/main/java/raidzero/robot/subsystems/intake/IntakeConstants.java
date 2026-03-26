package raidzero.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import raidzero.lib.wrappers.motors.LazyCTRE.MotorTelemetry;

public final class IntakeConstants {
    public static final class Intake {
        public static final int MOTOR_ID = 18;
        public static final String CAN_BUS_NAME = "canDoAttitude";

        public static final int FOLLOW_ID = 19;
        public static final MotorAlignmentValue FOLLOW_ALIGNMENT = MotorAlignmentValue.Opposed;

        public static final double SENSOR_TO_MECHANISM_RATIO = 3.0 / 1.0;
        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(60);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);

        public static final double P = 7.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double P_1 = 7.0;
        public static final double I_1 = 0.0;
        public static final double D_1 = 0.0;

        public static final double S = 0.47;
        public static final double G = 0.0;
        public static final double V = 0.2;
        public static final double A = 0.0;

        public static final double S_1 = 0.0;
        public static final double G_1 = 0.0;
        public static final double V_1 = 0.0;
        public static final double A_1 = 5.0;

        public static final GravityTypeValue FF_GRAVITY_TYPE = GravityTypeValue.Elevator_Static; // * Note: no gravity
                                                                                                 // (yippee!)
        public static final StaticFeedforwardSignValue FF_SIGN = StaticFeedforwardSignValue.UseVelocitySign;

        public static final int VELOCITY_SLOT = 0;

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(33.3);
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(0);

        public static final AngularVelocity INTAKE_SPEED = RotationsPerSecond.of(17.5);

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }

    public static final class Extender {
        public static final int MOTOR_ID = 16;
        public static final String CAN_BUS_NAME = "canDoAttitude";

        public static final int FOLLOW_ID = 17;
        public static final MotorAlignmentValue FOLLOW_ALIGNMENT = MotorAlignmentValue.Opposed;

        public static final double SENSOR_TO_MECHANISM_RATIO = 23.0 / 1.0;
        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(40);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);

        public static final double P = 8.0;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.45;
        public static final double G = 0.3;
        public static final double V = 2.56;
        public static final double A = 0.0;

        public static final GravityTypeValue FF_GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;
        public static final StaticFeedforwardSignValue FF_SIGN = StaticFeedforwardSignValue.UseVelocitySign;

        public static final double EXPO_V = 4.0;
        public static final double EXPO_A = 4.0;

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(0);
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(0);

        public static final Angle FORWARD_SOFT_LIMIT = Rotations.of(0.36);
        public static final boolean ENALBE_FORWARD_SOFT_LIMIT = true;
        public static final Angle REVERSE_SOFT_LIMIT = Rotations.of(0);
        public static final boolean ENABLE_REVERSE_SOFT_LIMIT = true;

        public static final Angle EXTENDED_SETPOINT = Degrees.of(0);
        public static final Angle RETRACTED_SETPOINT = Rotations.of(0.36);
        public static Angle AGITATE_ONE_SETPOINT = Rotations.of(0.08);
        public static Angle AGITATE_TWO_SETPOINT = Rotations.of(0.12);
        public static Angle AGITATE_THREE_SETPOINT = Rotations.of(0.18);

        public static final Angle ANGLE_TOLERANCE = Rotations.of(0.1);

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();

        // public static final Angle STARTUP_ANGLE = Rotations.of(0.3625);
        public static final Angle STARTUP_ANGLE = Rotations.of(0.0);
    }

    public static final class Spindexer {
        public static final int MOTOR_ID = 15;
        public static final String CAN_BUS_NAME = "canDoAttitude";

        public static final double SENSOR_TO_MECHANISM_RATIO = 9.0 / 1.0;
        public static final InvertedValue INVERTED = InvertedValue.CounterClockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(60);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(40);

        public static final double P = 1.5;
        public static final double I = 0.0;
        public static final double D = 0.0;

        public static final double S = 0.33;
        public static final double G = 0.0;
        public static final double V = 0.88;
        public static final double A = 0.0;

        public static final GravityTypeValue FF_GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;
        public static final StaticFeedforwardSignValue FF_SIGN = StaticFeedforwardSignValue.UseVelocitySign;

        public static final int VELOCITY_SLOT = 0;

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(13.6);
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(0);

        public static final AngularVelocity INDEX_SPEED = RotationsPerSecond.of(6.0);
        public static final AngularVelocity ANTIJAM_SPEED = RotationsPerSecond.of(6.0);

        public static final AngularVelocity VELOCITY_TOLERANCE = RotationsPerSecond.of(3.0);

        public static final MotorTelemetry TELEMETRY = new MotorTelemetry();
    }
}
