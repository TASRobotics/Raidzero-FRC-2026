package raidzero.robot.subsystems.turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import raidzero.lib.LimelightHelpers;
import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static raidzero.robot.subsystems.turret.TurretConstants.Yaw;

import static raidzero.robot.subsystems.turret.TurretConstants.Pitch;
import static raidzero.robot.subsystems.turret.TurretConstants.Limelight;

import com.ctre.phoenix6.signals.GravityTypeValue;

public interface TurretIO extends SubsystemIO {

    Angle getYaw();

    Angle getPitch();

    void moveTo(Angle pitch, Angle Yaw);

    Angle getYawFromTarget();

    Distance getHubDistance();

    boolean hasHubLOS();

    public class Real implements TurretIO {
        private LazyTalon yaw, pitch;

        public Real() {
            yaw = new LazyTalon(
                    Yaw.MOTOR_ID,
                    Yaw.SENSOR_TO_MECHANISM_RATIO,
                    Yaw.INVERTED,
                    Yaw.STATOR_CURRENT_LIMIT.in(Amps), Yaw.SUPPLY_CURRENT_LIMIT.in(Amps))
                    .withMotionMagicConfiguration(
                            Yaw.P, Yaw.I, Yaw.D,
                            Yaw.S, Yaw.G, Yaw.V, Yaw.A,
                            Yaw.EXPO_V, Yaw.EXPO_A,
                            GravityTypeValue.Elevator_Static,
                            Yaw.CRUISE_VELOCITY.in(RotationsPerSecond),
                            Yaw.ACCELERATION.in(RotationsPerSecondPerSecond))
                    .withCANCoder(Yaw.CANCODER_ID, Yaw.CANCODER_TYPE, Yaw.MAGNET_OFFSET, Yaw.CANCODER_SENSOR_DIRECTION)
                    .build();

            yaw = new LazyTalon(Pitch.MOTOR_ID, Pitch.SENSOR_TO_MECHANISM_RATIO, Pitch.INVERTED,
                    Pitch.STATOR_CURRENT_LIMIT.in(Amps),
                    Pitch.SUPPLY_CURRENT_LIMIT.in(Amps))
                    .withMotionMagicConfiguration(Pitch.P, Pitch.I, Pitch.D, Pitch.S, Pitch.G, Pitch.V, Pitch.A,
                            Pitch.EXPO_V,
                            Pitch.EXPO_A, GravityTypeValue.Elevator_Static,
                            Pitch.CRUISE_VELOCITY.in(RotationsPerSecond),
                            Pitch.ACCELERATION.in(RotationsPerSecondPerSecond))
                    .build();
        }

        private int[] getTargetTagIds() {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent() && alliance.get() == Alliance.Red) {
                return Limelight.RED_ALLIANCE_TAG_IDS;
            }
            return Limelight.BLUE_ALLIANCE_TAG_IDS;
        }

        private boolean isValidTagId(int tagId) {
            for (int validId : getTargetTagIds()) {
                if (tagId == validId) {
                    return true;
                }
            }
            return false;
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
        public void moveTo(Angle pitch, Angle yaw) {
            this.yaw.moveTo(yaw);
            this.pitch.moveTo(pitch);
        }

        @Override
        public Angle getYawFromTarget() {
            if (!LimelightHelpers.getTV(Limelight.LIMELIGHT_NAME)) {
                return Degrees.of(0);
            }

            LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(Limelight.LIMELIGHT_NAME);

            if (fiducials == null || fiducials.length == 0) {
                return Degrees.of(0);
            }

            LimelightHelpers.RawFiducial bestFiducial = null;
            double bestArea = 0;

            for (LimelightHelpers.RawFiducial fiducial : fiducials) {
                if (isValidTagId(fiducial.id) && fiducial.ta > bestArea) {
                    bestArea = fiducial.ta;
                    bestFiducial = fiducial;
                }
            }

            if (bestFiducial == null) {
                return Degrees.of(0);
            }

            return Degrees.of(bestFiducial.txnc);
        }

        @Override
        public Distance getHubDistance() {
            if (!LimelightHelpers.getTV(Limelight.LIMELIGHT_NAME)) {
                return Meters.of(0);
            }

            LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(Limelight.LIMELIGHT_NAME);

            if (fiducials == null || fiducials.length == 0) {
                return Meters.of(0);
            }

            LimelightHelpers.RawFiducial bestFiducial = null;
            double bestArea = 0;

            for (LimelightHelpers.RawFiducial fiducial : fiducials) {
                if (isValidTagId(fiducial.id) && fiducial.ta > bestArea) {
                    bestArea = fiducial.ta;
                    bestFiducial = fiducial;
                }
            }

            if (bestFiducial == null) {
                return Meters.of(0);
            }

            return Meters.of(bestFiducial.distToCamera);
        }

        @Override
        public boolean hasHubLOS() {
            if (!LimelightHelpers.getTV(Limelight.LIMELIGHT_NAME)) {
                return false;
            }

            LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(Limelight.LIMELIGHT_NAME);

            if (fiducials == null || fiducials.length == 0) {
                return false;
            }

            for (LimelightHelpers.RawFiducial fiducial : fiducials) {
                if (isValidTagId(fiducial.id)) {
                    return true;
                }
            }

            return false;
        }

        @Override
        public void updateTelemetry() {
            yaw.updateTelemetry(Yaw.TELEMETRY);
            pitch.updateTelemetry(Pitch.TELEMETRY);
        }
    }

    public class Sim extends Real {
    }
}