package raidzero.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import raidzero.lib.wrappers.motors.LazyCTRE.MotorTelemetry;

public final class ClimbConstants {
    public static final class Winch {
        public static final int MOTOR_ID = 20;
        public static final String CAN_BUS_NAME = "canDoAttitude";

        public static final double SENSOR_TO_MECHANISM_RATIO = 9.0 / 1.0;

        public static final InvertedValue INVERTED = InvertedValue.Clockwise_Positive;

        public static final Current STATOR_CURRENT_LIMIT = Amps.of(30);
        public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50);

        public static final double POSITION_P = 15.0;
        public static final double POSITION_I = 0.0;
        public static final double POSITION_D = 0.0;

        public static final double POSITION_S = 0.2;
        public static final double POSITION_G = 0.0;
        public static final double POSITION_V = 0.0;
        public static final double POSITION_A = 0.0;

        public static final GravityTypeValue FF_GRAVITY_TYPE = GravityTypeValue.Elevator_Static; // * Note: no gravity
                                                                                                 // (yippee!)
        public static final StaticFeedforwardSignValue FF_SIGN = StaticFeedforwardSignValue.UseVelocitySign;

        public static final int POSITION_SLOT = 0;

        public static final double EXPO_V = 10.0;
        public static final double EXPO_A = 10.0;

        public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(3.2); // max is 3.2
        public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(6.0); // max is 10

        public static final Angle FORWARD_SOFT_LIMIT = Rotations.of(0.67);
        public static final boolean ENALBE_FORWARD_SOFT_LIMIT = true;

        public static final Angle REVERSE_SOFT_LIMIT = Rotations.of(-0.58);
        public static final boolean ENABLE_REVERSE_SOFT_LIMIT = true;

        public static MotorTelemetry TELEMETRY = new MotorTelemetry();
    }
}
