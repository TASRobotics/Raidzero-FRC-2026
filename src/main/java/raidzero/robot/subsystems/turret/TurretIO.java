package raidzero.robot.subsystems.turret;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    int getStartupTag();

    Angle getAngleForStartup();

    void setStartupYaw(Angle yaw);

    void moveTo(Angle pitch, Angle Yaw);

    Angle getYawFromTarget();

    Distance getHubDistance();

    boolean hasHubLOS();

    void stopMotors();

    void setStartupPitch(Angle startingAngle);

    public class Real implements TurretIO {
        private LazyFXS yaw, pitch;

        public Real() {
            yaw = new LazyFXS(
                    Yaw.MOTOR_ID,
                    MotorArrangementValue.Minion_JST,
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
                    .withSoftLimits(true, Yaw.FORWARD_SOFT_LIMIT.in(Rotations), true,
                            Yaw.REVERSE_SOFT_LIMIT.in(Rotations))
                    .build();

            pitch = new LazyFXS(Pitch.MOTOR_ID, MotorArrangementValue.Minion_JST, Pitch.SENSOR_TO_MECHANISM_RATIO,
                    Pitch.INVERTED,
                    Pitch.STATOR_CURRENT_LIMIT.in(Amps),
                    Pitch.SUPPLY_CURRENT_LIMIT.in(Amps))
                    .withMotionMagicConfiguration(Pitch.P, Pitch.I, Pitch.D, Pitch.S, Pitch.G, Pitch.V, Pitch.A,
                            Pitch.EXPO_V,
                            Pitch.EXPO_A, Pitch.GRAVITY_TYPE,
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
        public int getStartupTag() {
            if (!LimelightHelpers.getTV(Limelight.LIMELIGHT_NAME)) {
                return -1; // No valid target
            }

            LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(Limelight.LIMELIGHT_NAME);

            if (fiducials == null || fiducials.length == 0) {
                return -1; // No fiducials detected
            }

            // Find the closest valid startup tag
            int bestTagId = -1;
            double closestDistance = Double.MAX_VALUE;

            for (LimelightHelpers.RawFiducial fiducial : fiducials) {
                if (isValidStartupTag(fiducial.id) && fiducial.distToCamera < closestDistance) {
                    closestDistance = fiducial.distToCamera;
                    bestTagId = fiducial.id;
                }
            }

            return bestTagId;
        }

        private boolean isValidStartupTag(int tagId) {
            for (int validId : Limelight.STARTUP_REFERENCE_TAG_IDS) {
                if (tagId == validId) {
                    return true;
                }
            }
            return false;
        }

        @Override
        public Angle getAngleForStartup() {
            int startupTagId = getStartupTag();

            if (startupTagId == -1) {
                return Degrees.of(0); // No valid startup tag found
            }

            LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(Limelight.LIMELIGHT_NAME);

            if (fiducials == null || fiducials.length == 0) {
                return Degrees.of(0);
            }

            // Find the fiducial matching our startup tag and return its TX
            for (LimelightHelpers.RawFiducial fiducial : fiducials) {
                if (fiducial.id == startupTagId) {
                    return Degrees.of(fiducial.txnc);
                }
            }

            return Degrees.of(0); // Shouldn't reach here, but fallback
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

            double yawRotations = yaw.in(Rotations);
            double forwardLimit = Yaw.FORWARD_SOFT_LIMIT.in(Rotations);
            double reverseLimit = Yaw.REVERSE_SOFT_LIMIT.in(Rotations);

            // Wrap to [0, 1) range, then shift to [reverseLimit, reverseLimit + 1)
            double reverseWrapped = ((yawRotations - reverseLimit) % 1.0 + 1.0) % 1.0 + reverseLimit;
            double forwardWrapped = ((yawRotations - forwardLimit) % 1.0 + 1.0) % 1.0 + forwardLimit;

            double bestWrapped = Math.min(getYaw().in(Rotations) - reverseWrapped,
                    getYaw().in(Rotations) - forwardWrapped);

            // Clamp to soft limits
            double clamped = Math.max(reverseLimit, Math.min(forwardLimit, bestWrapped));

            // this.yaw.moveTo(Rotations.of(clamped));
            this.yaw.moveWithMMVolt(Rotations.of(clamped));
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

        @Override
        public void stopMotors() {
            yaw.stop();
            pitch.stop();
        }

        @Override
        public void setStartupPitch(Angle startingAngle) {
            pitch.getMotor().setPosition(startingAngle);
        }
    }

    public class Sim extends Real {
    }


}