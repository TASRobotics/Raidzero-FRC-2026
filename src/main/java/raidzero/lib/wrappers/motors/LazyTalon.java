package raidzero.lib.wrappers.motors;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;

public class LazyTalon implements LazyCTRE<TalonFX, FeedbackSensorSourceValue>, LazyMotor<TalonFX> {
    private TalonFX motor, follower;
    private int motorID;

    private TalonFXConfiguration motorConfiguration, followConfiguration;

    private CANcoder canCoder;
    private CANcoderConfiguration canCoderConfiguration;

    /**
     * Constructs a LazyTalon instance with the specified motor identifier and
     * configuration parameters.
     *
     * @param motorID                the device id of the motor controller
     * @param sensorToMechanismRatio the gear ratio of the sensor to the mechanism
     *                               (tell the mech team (driven / driving) *
     *                               planetary product)
     * @param invertedValue          what direction that is positive for the motor
     * @param statorCurrentLimit     the maximum stator current
     * @param supplyCurrentLimit     the maximum supply current
     */
    public LazyTalon(int motorID, double sensorToMechanismRatio, InvertedValue invertedValue, double statorCurrentLimit,
            double supplyCurrentLimit) {
        this(motorID, "", sensorToMechanismRatio, invertedValue, statorCurrentLimit, supplyCurrentLimit);
    }

    public LazyTalon(int motorID, String canBus, double sensorToMechanismRatio, InvertedValue invertedValue,
            double statorCurrentLimit,
            double supplyCurrentLimit) {
        this.motorID = motorID;

        motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;

        motorConfiguration.MotorOutput.Inverted = invertedValue;

        motorConfiguration.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        motorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        motor = new TalonFX(motorID, canBus);
    }

    @Override
    public LazyTalon withFollower(int followerID, boolean isInverted) {
        followConfiguration = new TalonFXConfiguration();

        follower = new TalonFX(followerID);
        follower.getConfigurator().apply(followConfiguration);
        follower.setControl(new Follower(motorID, isInverted));

        return this;
    }

    @Override
    public LazyTalon withCANCoder(int CANCoderID, FeedbackSensorSourceValue sensorType, double magnetOffset,
            SensorDirectionValue sensorDirection) {
        canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = magnetOffset;
        canCoderConfiguration.MagnetSensor.SensorDirection = sensorDirection;

        canCoder = new CANcoder(CANCoderID);
        canCoder.getConfigurator().apply(canCoderConfiguration);

        motorConfiguration.Feedback.FeedbackRemoteSensorID = CANCoderID;
        motorConfiguration.Feedback.FeedbackSensorSource = sensorType;

        return this;
    }

    @Override
    public LazyTalon withCANCoder(int CANCoderID, FeedbackSensorSourceValue sensorType, double magnetOffset,
            SensorDirectionValue sensorDirection, double discontinuityPoint, double rotorToSensorRatio) {
        withCANCoder(CANCoderID, sensorType, magnetOffset, sensorDirection);

        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = discontinuityPoint;
        canCoder.getConfigurator().apply(canCoderConfiguration);

        motorConfiguration.Feedback.RotorToSensorRatio = rotorToSensorRatio;

        return this;
    }

    @Override
    public LazyTalon withPIDFConfiguration(double p, double i, double d, double s, double g, double v, double a, GravityTypeValue gravityType, StaticFeedforwardSignValue staticSignValue, int slotNumber) {
        switch (slotNumber) {
            case 0:
                motorConfiguration.Slot0.kP = p;
                motorConfiguration.Slot0.kI = i;
                motorConfiguration.Slot0.kD = d;
                motorConfiguration.Slot0.kS = s;
                motorConfiguration.Slot0.kG = g;
                motorConfiguration.Slot0.kV = v;
                motorConfiguration.Slot0.kA = a;
                motorConfiguration.Slot0.GravityType = gravityType;
                motorConfiguration.Slot0.StaticFeedforwardSign = staticSignValue;
                break;
            case 1:
                motorConfiguration.Slot1.kP = p;
                motorConfiguration.Slot1.kI = i;
                motorConfiguration.Slot1.kD = d;
                motorConfiguration.Slot1.kS = s;
                motorConfiguration.Slot1.kG = g;
                motorConfiguration.Slot1.kV = v;
                motorConfiguration.Slot1.kA = a;
                motorConfiguration.Slot1.GravityType = gravityType;
                motorConfiguration.Slot1.StaticFeedforwardSign = staticSignValue;
                break;
            case 2:
                motorConfiguration.Slot2.kP = p;
                motorConfiguration.Slot2.kI = i;
                motorConfiguration.Slot2.kD = d;
                motorConfiguration.Slot2.kS = s;
                motorConfiguration.Slot2.kG = g;
                motorConfiguration.Slot2.kV = v;
                motorConfiguration.Slot2.kA = a;
                motorConfiguration.Slot2.GravityType = gravityType;
                motorConfiguration.Slot2.StaticFeedforwardSign = staticSignValue;
                break;
            default:
                DriverStation.reportWarning("PDF Slot number is invalid for device ID: " + motorID, true);
        }

        return this;
    }

    @Override
    public LazyTalon withMotionMagicConfiguration(double expoV, double expoA, double cruiseVelocity, double maxAcceleration) {
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        motorConfiguration.MotionMagic.MotionMagicAcceleration = maxAcceleration;

        motorConfiguration.MotionMagic.MotionMagicExpo_kV = expoV;
        motorConfiguration.MotionMagic.MotionMagicExpo_kA = expoA;

        return this;
    }

    @Override
    public LazyTalon withMotionMagicConfiguration(double expoV, double expoA, double cruiseVelocity, double maxAcceleration, double jerk) {
        withMotionMagicConfiguration(expoV, expoA, cruiseVelocity, maxAcceleration);
        motorConfiguration.MotionMagic.MotionMagicJerk = jerk;

        return this;
    }

    @Override
    public LazyTalon withLimitSwitch(boolean forwardLimitEnable, boolean forwardLimitAutosetPositionEnable,
            double forwardLimitAutosetPositionValue, boolean reverseLimitEnable,
            boolean reverseLimitAutosetPositionEnable, double reverseLimitAutosetPositionValue) {
        motorConfiguration.HardwareLimitSwitch.ForwardLimitEnable = forwardLimitEnable;
        motorConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = forwardLimitAutosetPositionEnable;
        motorConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = forwardLimitAutosetPositionValue;

        motorConfiguration.HardwareLimitSwitch.ReverseLimitEnable = reverseLimitEnable;
        motorConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = reverseLimitAutosetPositionEnable;
        motorConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = reverseLimitAutosetPositionValue;

        return this;
    }

    @Override
    public LazyTalon withSoftLimits(boolean forwardSoftLimitEnable, double forwardSoftLimit,
            boolean reverseSoftLimitEnable, double reverseSoftLimit) {
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardSoftLimitEnable;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimit;

        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseSoftLimitEnable;
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimit;

        return this;
    }

    @Override
    public LazyTalon withCustomConfiguration(ParentConfiguration configuration) {
        motorConfiguration = (TalonFXConfiguration) configuration;

        return this;
    }

    @Override
    public TalonFXConfiguration getConfiguration() {
        return motorConfiguration;
    }

    @Override
    public LazyTalon build() {
        motor.getConfigurator().apply(motorConfiguration);
        motor.setNeutralMode(NeutralModeValue.Brake);

        return this;
    }

    @Override
    public void moveTo(Angle setpoint) {
        motor.setControl(new MotionMagicExpoVoltage(setpoint).withEnableFOC(true));
    }

    @Override
    public void setVelocity(AngularVelocity velocity, int slotNumber) {
        motor.setControl(new MotionMagicVelocityVoltage(velocity.in(RotationsPerSecond)).withEnableFOC(true).withSlot(slotNumber));
    }

    @Override
    public void setPercent(double speed) {
        motor.set(speed);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void setBrake() {
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void setCoast() {
        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public Angle getFeedbackPosition() {
        return motor.getPosition().getValue();
    }

    @Override
    public AngularVelocity getFeedbackVelocity() {
        return motor.getVelocity().getValue();
    }

    @Override
    public AngularAcceleration getFeedbackAcceleration() {
        return motor.getAcceleration().getValue();
    }

    @Override
    public TalonFX getMotor() {
        return motor;
    }

    @Override
    public CANcoder getCanCoder() {
        return canCoder;
    }

    @Override
    public TalonFX getFollower() {
        return follower;
    }

    @Override
    public void updateTelemetry(MotorTelemetry telemetry) {
        telemetry.position = motor.getPosition().getValue();
        telemetry.velocity = motor.getVelocity().getValue();
        telemetry.acceleration = motor.getAcceleration().getValue();
        telemetry.statorCurrent = motor.getStatorCurrent().getValue();
        telemetry.supplyCurrent = motor.getSupplyCurrent().getValue();
    }
}