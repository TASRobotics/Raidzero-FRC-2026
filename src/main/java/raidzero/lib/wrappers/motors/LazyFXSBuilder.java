package raidzero.lib.wrappers.motors;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;

public class LazyFXSBuilder implements LazyCTREBuilder<TalonFXS, ExternalFeedbackSensorSourceValue> {
    private MotorArrangementValue motorArrangement;
    private int motorID, followID, canCoderID;
    private String canBus = "", followCanBus = "", canCoderCanBus = "";

    private TalonFXSConfiguration motorConfiguration = null, followConfiguration = null;
    private CANcoderConfiguration canCoderConfiguration = null;

    private MotorAlignmentValue followerInverted;

    /**
     * Constructs a LazyFXS instance with the specified motor configurations
     *
     * @param motorID                the device id of the motor controller
     * @param motorArrangement       what type of motor the FXS is connected to
     * @param sensorToMechanismRatio the gear ratio of the sensor to the mechanism
     *                               (tell the mech team (driven / driving) *
     *                               planetary product)
     * @param invertedValue          what direction that is positive for the motor
     * @param statorCurrentLimit     the maximum stator current
     * @param supplyCurrentLimit     the maximum supply current
     */
    public LazyFXSBuilder(int motorID, MotorArrangementValue motorArrangement, double sensorToMechanismRatio,
            InvertedValue invertedValue, double statorCurrentLimit, double supplyCurrentLimit) {
        this(motorID, "", motorArrangement, sensorToMechanismRatio, invertedValue, statorCurrentLimit,
                supplyCurrentLimit);
    }

    public LazyFXSBuilder(int motorID, String canBus, MotorArrangementValue motorArrangement,
            double sensorToMechanismRatio,
            InvertedValue invertedValue, double statorCurrentLimit, double supplyCurrentLimit) {
        this.motorID = motorID;
        this.canBus = canBus;

        motorConfiguration = new TalonFXSConfiguration();
        motorConfiguration.Commutation.MotorArrangement = motorArrangement;
        this.motorArrangement = motorArrangement;
        motorConfiguration.ExternalFeedback.SensorToMechanismRatio = sensorToMechanismRatio;
        motorConfiguration.MotorOutput.Inverted = invertedValue;

        motorConfiguration.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
        motorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 0.0;

        motorConfiguration.ClosedLoopGeneral.ContinuousWrap = false;
        motorConfiguration.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;

        motorConfiguration.Audio.AllowMusicDurDisable = true;
    }

    @Override
    public LazyFXSBuilder withFollower(int followerID, String canBus, MotorAlignmentValue isInverted) {
        this.followID = followerID;
        this.followCanBus = canBus;
        this.followerInverted = isInverted;

        followConfiguration = new TalonFXSConfiguration();
        followConfiguration.Commutation.MotorArrangement = this.motorArrangement;

        return this;
    }

    @Override
    public LazyFXSBuilder withCANCoder(int CANCoderID, String canCoderCanBus,
            ExternalFeedbackSensorSourceValue sensorType,
            double magnetOffset,
            SensorDirectionValue sensorDirection) {

        this.canCoderID = CANCoderID;
        this.canCoderCanBus = canCoderCanBus;

        canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = magnetOffset;
        canCoderConfiguration.MagnetSensor.SensorDirection = sensorDirection;

        motorConfiguration.ExternalFeedback.FeedbackRemoteSensorID = CANCoderID;
        motorConfiguration.ExternalFeedback.ExternalFeedbackSensorSource = sensorType;

        return this;
    }

    @Override
    public LazyFXSBuilder withCANCoder(int CANCoderID, String canCoderCanBus,
            ExternalFeedbackSensorSourceValue sensorType,
            double magnetOffset,
            SensorDirectionValue sensorDirection, double discontinuityPoint, double rotorToSensorRatio) {
        withCANCoder(CANCoderID, canCoderCanBus, sensorType, magnetOffset, sensorDirection);

        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = discontinuityPoint;

        motorConfiguration.ExternalFeedback.RotorToSensorRatio = rotorToSensorRatio;

        return this;
    }

    @Override
    public LazyFXSBuilder withPIDFConfiguration(double p, double i, double d, double s, double g, double v, double a,
            GravityTypeValue gravityType, StaticFeedforwardSignValue staticSignValue, int slotNumber) {
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
    public LazyFXSBuilder withMotionMagicConfiguration(double expoV, double expoA, AngularVelocity cruiseVelocity,
            AngularAcceleration maxAcceleration) {
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = cruiseVelocity.in(RotationsPerSecond);
        motorConfiguration.MotionMagic.MotionMagicAcceleration = maxAcceleration.in(RotationsPerSecondPerSecond);

        motorConfiguration.MotionMagic.MotionMagicExpo_kV = expoV;
        motorConfiguration.MotionMagic.MotionMagicExpo_kA = expoA;

        return this;
    }

    @Override
    public LazyFXSBuilder withMotionMagicConfiguration(double expoV, double expoA, AngularVelocity cruiseVelocity,
            AngularAcceleration maxAcceleration, double jerk) {
        withMotionMagicConfiguration(expoV, expoA, cruiseVelocity, maxAcceleration);
        motorConfiguration.MotionMagic.MotionMagicJerk = jerk;

        return this;
    }

    @Override
    public LazyFXSBuilder withCustomConfiguration(ParentConfiguration configuration) {
        motorConfiguration = (TalonFXSConfiguration) configuration;

        return this;
    }

    @Override
    public LazyFXSBuilder withLimitSwitch(boolean forwardLimitEnable, boolean forwardLimitAutosetPositionEnable,
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
    public LazyFXSBuilder withSoftLimits(boolean forwardSoftLimitEnable, double forwardSoftLimit,
            boolean reverseSoftLimitEnable, double reverseSoftLimit) {
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
        LazyFXS motor = new LazyFXS(motorID, canBus, motorConfiguration, followID, followCanBus, followConfiguration,
                followerInverted, canCoderID, canCoderCanBus, canCoderConfiguration);
        motor.setBrake();

        return motor;
    }
}