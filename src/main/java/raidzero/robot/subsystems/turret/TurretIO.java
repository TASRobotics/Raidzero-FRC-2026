package raidzero.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static raidzero.robot.subsystems.turret.TurretConstants.Limelight;
import static raidzero.robot.subsystems.turret.TurretConstants.Yaw;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import raidzero.lib.LimelightHelpers;
import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyFXS;
import raidzero.lib.wrappers.motors.LazyFXSBuilder;
import raidzero.robot.Superstructure;

public interface TurretIO extends SubsystemIO {

    /**
     * Moves the yaw to the specified position
     *
     * @param Yaw the {@link Angle}
     */
    void moveTo(Angle yaw);

    /**
     * Gets the current yaw
     *
     * @return the {@link Angle}
     */
    Angle getYaw();

    /**
     * Sets the yaw's position for startup
     *
     * @param yaw the {@link Angle}
     */
    void setStartupYaw(Angle yaw);

    /**
     * Returns true if the bot has line-of-sight with the hub
     *
     * @return true if the bot has line-of-sight with the hub
     */
    boolean hasHubLOS();

    /**
     * Gets the yaw angle from the goal target
     *
     * @return the {@link Angle}
     */
    Angle getYawFromTarget();

    /**
     * Gets the distance to the hub
     *
     * @return the {@link Distance}
     */
    Distance getHubDistance();

    /**
     * Sets the yaw to move in a set speed in percent
     *
     * @param percent the percent [0, 1]
     */
    void runYawPercent(double percent);

    /**
     * Stops the yaw motor
     */
    void stop();

    /**
     * Gets the reading from the first Jared modulo method sensor (22 teeth)
     * @return the sensor reading in rotations
     */
    double getJMM1Reading();

    /**
     * Gets the reading from the second Jared modulo method sensor (29 teeth)
     * @return the sensor reading in rotations
     */
    double getJMM2Reading();

    public class Real implements TurretIO {
        private LazyFXS yaw;
        private CANcoder jmm1, jmm2;

        private double lastTagDistance;

        public Real() {
            yaw = new LazyFXSBuilder(
                Yaw.MOTOR_ID, Yaw.CAN_BUS_NAME,
                Yaw.MOTOR_ARRANGEMENT,
                Yaw.SENSOR_TO_MECHANISM_RATIO,
                Yaw.INVERTED,
                Yaw.STATOR_CURRENT_LIMIT.in(Amps),
                Yaw.SUPPLY_CURRENT_LIMIT.in(Amps)
            )
                .withPIDFConfiguration(
                    Yaw.POSITION_P, Yaw.POSITION_I, Yaw.POSITION_D,
                    Yaw.POSITION_S, Yaw.POSITION_G, Yaw.POSITION_V, Yaw.POSITION_A,
                    Yaw.FF_GRAVITY_TYPE,
                    Yaw.FF_SIGN,
                    Yaw.POSITION_SLOT
                )
                .withMotionMagicConfiguration(
                    Yaw.EXPO_V, Yaw.EXPO_A,
                    Yaw.CRUISE_VELOCITY,
                    Yaw.ACCELERATION
                )
                .withSoftLimits(
                    true, Yaw.FORWARD_SOFT_LIMIT.in(Rotations),
                    true, Yaw.REVERSE_SOFT_LIMIT.in(Rotations)
                )
                .build();

            CANcoderConfiguration jmm1Config = new CANcoderConfiguration();
            jmm1Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
            jmm1Config.MagnetSensor.SensorDirection = Yaw.JMM_1_INVERTED;
            jmm1Config.MagnetSensor.MagnetOffset = Yaw.JMM_OFFSET_1;

            jmm1 = new CANcoder(Yaw.JMM_1_ID, Yaw.CAN_BUS_NAME);
            jmm1.getConfigurator().apply(jmm1Config);

            CANcoderConfiguration jmm2Config = new CANcoderConfiguration();
            jmm2Config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
            jmm2Config.MagnetSensor.SensorDirection = Yaw.JMM_2_INVERTED;
            jmm2Config.MagnetSensor.MagnetOffset = Yaw.JMM_OFFSET_2;

            jmm2 = new CANcoder(Yaw.JMM_2_ID, Yaw.CAN_BUS_NAME);
            jmm2.getConfigurator().apply(jmm2Config);

            lastTagDistance = 0;
        }

        @Override
        public void moveTo(Angle setpoint) {
            yaw.setMMExpoTarget(setpoint, Yaw.POSITION_SLOT);
        }

        @Override
        public Angle getYaw() {
            return yaw.getPosition();
        }

        @Override
        public void setStartupYaw(Angle yaw) {
            this.yaw.getMotor().setPosition(yaw);
        }

        @Override
        public boolean hasHubLOS() {
            return LimelightHelpers.getTV(Limelight.LIMELIGHT_NAME);
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
                return Meters.of(lastTagDistance);
            }

            double[] cameraPoseArray = LimelightHelpers.getCameraPose_TargetSpace(Limelight.LIMELIGHT_NAME);

            if (cameraPoseArray.length >= 6) {
                Pose3d cameraPose = new Pose3d(
                    cameraPoseArray[0], // X
                    cameraPoseArray[1], // Y
                    cameraPoseArray[2], // Z
                    new Rotation3d(
                        Math.toRadians(cameraPoseArray[3]), // Roll
                        Math.toRadians(cameraPoseArray[4]), // Pitch
                        Math.toRadians(cameraPoseArray[5]) // Yaw
                    )
                );

                // Distance from camera to target (origin is the target)
                double distance = cameraPose.getTranslation().getNorm();
                lastTagDistance = distance;

                return Meters.of(distance);
            } else {
                return Meters.of(lastTagDistance);
            }
        }

        @Override
        public void runYawPercent(double percent) {
            yaw.setDutyCycle(percent);
        }

        @Override
        public void stop() {
            yaw.stop();
        }

        @Override
        public double getJMM1Reading() {
            return jmm1.getAbsolutePosition().getValueAsDouble() * Yaw.JARED_TEETH_1;
        }

        @Override
        public double getJMM2Reading() {
            return jmm2.getAbsolutePosition().getValueAsDouble() * Yaw.JARED_TEETH_2;
        }

        @Override
        public void updateTelemetry() {
            yaw.updateTelemetry(Yaw.TELEMETRY);
        }
    }

    public class Sim extends Real {}
}