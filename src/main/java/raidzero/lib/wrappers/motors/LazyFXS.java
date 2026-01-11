package raidzero.lib.wrappers.motors;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class LazyFXS implements LazyCTRE<TalonFXS, ExternalFeedbackSensorSourceValue>, LazyMotor<TalonFXS> {
    private TalonFXS motor, follower;
    private MotorArrangementValue motorArrangement;
    private int motorID;

    private TalonFXSConfiguration motorConfiguration, followConfiguration;

    private CANcoder canCoder;
    private CANcoderConfiguration canCoderConfiguration;

    /**
     * Constructs a LazyFXS instance with the specified motor configurations
     *
     * @param motorID the device id of the motor controller
     * @param motorArrangement what type of motor the FXS is connected to
     * @param sensorToMechanismRatio the gear ratio of the sensor to the mechanism (tell the mech team (driven / driving) * planetary product)
     * @param invertedValue what direction that is positive for the motor
     * @param statorCurrentLimit the maximum stator current
     * @param supplyCurrentLimit the maximum supply current
     */
    public LazyFXS(int motorID, MotorArrangementValue motorArrangement, double sensorToMechanismRatio, InvertedValue invertedValue, double statorCurrentLimit, double supplyCurrentLimit) {
        this.motorID = motorID;

        motorConfiguration = new TalonFXSConfiguration();
        motorConfiguration.Commutation.MotorArrangement = motorArrangement;
        this.motorArrangement = motorArrangement;
        motorConfiguration.ExternalFeedback.SensorToMechanismRatio = sensorToMechanismRatio;
        motorConfiguration.MotorOutput.Inverted = invertedValue;

        motorConfiguration.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        motorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        motorConfiguration.ClosedLoopGeneral.ContinuousWrap = true;

        motor = new TalonFXS(motorID);
        motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public LazyFXS withFollower(int followerID, boolean isInverted) {
        followConfiguration = new TalonFXSConfiguration();
        followConfiguration.Commutation.MotorArrangement = this.motorArrangement;

        follower = new TalonFXS(followerID);
        follower.getConfigurator().apply(followConfiguration);
        follower.setControl(new Follower(motorID, isInverted));

        return this;
    }

    @Override
    public LazyFXS withCANCoder(int CANCoderID, ExternalFeedbackSensorSourceValue sensorType, double magnetOffset, SensorDirectionValue sensorDirection) {
        canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = magnetOffset;
        canCoderConfiguration.MagnetSensor.SensorDirection = sensorDirection;

        canCoder = new CANcoder(CANCoderID);
        canCoder.getConfigurator().apply(canCoderConfiguration);

        motorConfiguration.ExternalFeedback.FeedbackRemoteSensorID = CANCoderID;
        motorConfiguration.ExternalFeedback.ExternalFeedbackSensorSource = sensorType;

        return this;
    }

    @Override
    public LazyFXS withCANCoder(int CANCoderID, ExternalFeedbackSensorSourceValue sensorType, double magnetOffset, SensorDirectionValue sensorDirection, double discontinuityPoint, double rotorToSensorRatio) {
        withCANCoder(CANCoderID, sensorType, magnetOffset, sensorDirection);

        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = discontinuityPoint;
        canCoder.getConfigurator().apply(canCoderConfiguration);

        motorConfiguration.ExternalFeedback.RotorToSensorRatio = rotorToSensorRatio;

        return this;
    }

    @Override
    public LazyFXS withMotionMagicConfiguration(double p, double i, double d, double s, double g, double v, double a, double expoV, double expoA, GravityTypeValue gravityType, double cruiseVelocity, double maxAcceleration) {
        motorConfiguration.Slot0.kP = p;
        motorConfiguration.Slot0.kI = i;
        motorConfiguration.Slot0.kD = d;
        motorConfiguration.Slot0.kS = s;
        motorConfiguration.Slot0.kG = g;
        motorConfiguration.Slot0.kV = v;
        motorConfiguration.Slot0.kA = a;
        motorConfiguration.Slot0.GravityType = gravityType;

        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity;
        motorConfiguration.MotionMagic.MotionMagicAcceleration = maxAcceleration;

        motorConfiguration.MotionMagic.MotionMagicExpo_kV = expoV;
        motorConfiguration.MotionMagic.MotionMagicExpo_kA = expoA;

        return this;
    }

    @Override
    public LazyFXS withCustomConfiguration(ParentConfiguration configuration) {
        motorConfiguration = (TalonFXSConfiguration) configuration;

        return this;
    }

    @Override
    public LazyFXS withLimitSwitch(boolean forwardLimitEnable, boolean forwardLimitAutosetPositionEnable, double forwardLimitAutosetPositionValue, boolean reverseLimitEnable, boolean reverseLimitAutosetPositionEnable, double reverseLimitAutosetPositionValue) {
        motorConfiguration.HardwareLimitSwitch.ForwardLimitEnable = forwardLimitEnable;
        motorConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = forwardLimitAutosetPositionEnable;
        motorConfiguration.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = forwardLimitAutosetPositionValue;

        motorConfiguration.HardwareLimitSwitch.ReverseLimitEnable = reverseLimitEnable;
        motorConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = reverseLimitAutosetPositionEnable;
        motorConfiguration.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = reverseLimitAutosetPositionValue;

        return this;
    }

    @Override
    public LazyFXS withSoftLimits(boolean forwardSoftLimitEnable, double forwardSoftLimit, boolean reverseSoftLimitEnable, double reverseSoftLimit) {
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardSoftLimitEnable;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = forwardSoftLimit;

        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseSoftLimitEnable;
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = reverseSoftLimit;

        return this;
    }

    @Override
    public TalonFXSConfiguration getConfiguration() {
        return motorConfiguration;
    }

    @Override
    public LazyFXS build() {
        motor.getConfigurator().apply(motorConfiguration);

        return this;
    }

    @Override
    public void moveTo(Angle setpoint) {
        motor.setControl(new MotionMagicExpoVoltage(setpoint));
    }

    @Override
    public void moveWithVelocity(AngularVelocity velocity) {
        motor.setControl(new MotionMagicVelocityVoltage(velocity));
    }

    @Override
    public void set(double speed) {
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
    public TalonFXS getMotor() {
        return motor;
    }

    @Override
    public CANcoder getCanCoder() {
        return canCoder;
    }

    @Override
    public TalonFXS getFollower() {
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
