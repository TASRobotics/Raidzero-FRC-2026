package raidzero.robot.subsystems.turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import raidzero.lib.LimelightHelpers;
import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyFXS;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static raidzero.robot.subsystems.turret.TurretConstants.Yaw;

import static raidzero.robot.subsystems.turret.TurretConstants.Pitch;
import static raidzero.robot.subsystems.turret.TurretConstants.Limelight;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

public interface TurretIO extends SubsystemIO {

    Angle getYaw();

    void setPitch(Angle pitch);

    void setYaw(Angle yaw);

    Angle getPitch();

    void setStartupYaw(Angle yaw);

    void moveTo(Angle pitch, Angle Yaw);

    Angle getYawFromTarget();

    Distance getHubDistance();

    boolean hasHubLOS();

    void stopMotors();

    void setStartupPitch(Angle startingAngle);

    void runYawSimple(double percent);

    public class Real implements TurretIO {
        private LazyFXS yaw, pitch;

        public Real() {
            yaw = new LazyFXS(
                    Yaw.MOTOR_ID, Yaw.CAN_BUS_NAME,
                    MotorArrangementValue.Minion_JST,
                    Yaw.SENSOR_TO_MECHANISM_RATIO,
                    Yaw.INVERTED,
                    Yaw.STATOR_CURRENT_LIMIT.in(Amps),
                    Yaw.SUPPLY_CURRENT_LIMIT.in(Amps))
                    .withMotionMagicConfiguration(
                            Yaw.P, Yaw.I, Yaw.D,
                            Yaw.S, Yaw.G, Yaw.V, Yaw.A,
                            Yaw.EXPO_V, Yaw.EXPO_A,
                            GravityTypeValue.Elevator_Static,
                            Yaw.CRUISE_VELOCITY.in(RotationsPerSecond),
                            Yaw.ACCELERATION.in(RotationsPerSecondPerSecond))
                    .withSoftLimits(
                            true, Yaw.FORWARD_SOFT_LIMIT.in(Rotations),
                            true, Yaw.REVERSE_SOFT_LIMIT.in(Rotations))
                    .build();

            pitch = new LazyFXS(
                    Pitch.MOTOR_ID, Pitch.CAN_BUS_NAME,
                    MotorArrangementValue.Minion_JST,
                    Pitch.SENSOR_TO_MECHANISM_RATIO,
                    Pitch.INVERTED,
                    Pitch.STATOR_CURRENT_LIMIT.in(Amps),
                    Pitch.SUPPLY_CURRENT_LIMIT.in(Amps))
                    .withMotionMagicConfiguration(
                            Pitch.P, Pitch.I, Pitch.D,
                            Pitch.S, Pitch.G, Pitch.V, Pitch.A,
                            Pitch.EXPO_V, Pitch.EXPO_A,
                            Pitch.GRAVITY_TYPE,
                            Pitch.CRUISE_VELOCITY.in(RotationsPerSecond),
                            Pitch.ACCELERATION.in(RotationsPerSecondPerSecond))
                    .build();
        }

        @Override
        public Angle getYaw() {
            return yaw.getFeedbackPosition();
        }

        @Override
        public Angle getPitch() {
            return pitch.getFeedbackPosition();
        }

        @Override
        public void setStartupYaw(Angle yaw) {
            this.yaw.getMotor().setPosition(yaw);
        }

        @Override
        public void setPitch(Angle pitch) {
            // this.pitch.moveTo(pitch);
            this.pitch.moveWithMMVolt(pitch);
        }

        @Override
        public void setYaw(Angle yaw) {
            this.yaw.moveWithMMVolt(yaw);
        }

        @Override
        public void moveTo(Angle pitch, Angle yaw) {
            this.setPitch(pitch);
            this.setYaw(yaw);
        }

        @Override
        public Angle getYawFromTarget() {
            if (!LimelightHelpers.getTV(Limelight.LIMELIGHT_NAME)) {
                return Degrees.of(0);
            }

            return Degrees.of(LimelightHelpers.getTX(Limelight.LIMELIGHT_NAME));
        }

        @Override
        public Distance getHubDistance() {
            if (!LimelightHelpers.getTV(Limelight.LIMELIGHT_NAME)) {
                return Meters.of(0);
            }

            double ty = LimelightHelpers.getTY(Limelight.LIMELIGHT_NAME);

            double totalAngleRadians = Math.toRadians(Limelight.MOUNT_ANGLE.in(Degrees) + ty);

            double heightDifference = Limelight.HUB_HEIGHT.in(Meters) - Limelight.CAMERA_HEIGHT.in(Meters);
            double distance = heightDifference / Math.tan(totalAngleRadians);

            return Meters.of(distance);
        }

        @Override
        public boolean hasHubLOS() {
            return LimelightHelpers.getTV(Limelight.LIMELIGHT_NAME);
        }

        @Override
        public void updateTelemetry() {
            yaw.updateTelemetry(Yaw.TELEMETRY);
            pitch.updateTelemetry(Pitch.TELEMETRY);
        }

        @Override
        public void stopMotors() {
            yaw.stop();
            yaw.getMotor();
            pitch.stop();
        }

        @Override
        public void setStartupPitch(Angle startingAngle) {
            pitch.getMotor().setPosition(startingAngle);
        }

        @Override
        public void runYawSimple(double percent) {
            yaw.setPercent(percent);
        }
    }

    public class Sim extends Real {
    }
}