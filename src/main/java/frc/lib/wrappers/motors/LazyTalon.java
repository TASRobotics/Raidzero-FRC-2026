package frc.lib.wrappers.motors;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;

public class LazyTalon implements LazyCTRE {
    private TalonFX motor, follower;
    private CANcoder canCoder;
    private boolean enableFOC = true;

    /**
     * Constructs a LazyTalon with the given configuration
     *
     * @param motorID The CAN ID of the motor
     * @param canBus The CAN bus the motor is on
     * @param configuration The configuration to apply to the motor
     * @param followID The CAN ID of the follower motor
     * @param followCanBus The CAN bus the follower motor is on
     * @param followConfiguration The configuration to apply to the follower motor
     * @param followerInverted The inversion of the follower motor relative to the leader motor as a {@link MotorAlignmentValue}
     * @param canCoderID The CAN ID of the CANcoder
     * @param canCoderCanBus The CAN bus the CANcoder is on
     * @param canCoderConfiguration The configuration to apply to the CANcoder
     */
    public LazyTalon(int motorID, CANBus canBus, TalonFXConfiguration configuration, int followID, CANBus followCanBus, TalonFXConfiguration followConfiguration, MotorAlignmentValue followerInverted, int canCoderID, CANBus canCoderCanBus, CANcoderConfiguration canCoderConfiguration) {
        motor = new TalonFX(motorID, canBus);
        motor.getConfigurator().apply(configuration);

        if (followConfiguration != null) {
            follower = new TalonFX(followID, followCanBus);
            follower.getConfigurator().apply(followConfiguration);
            follower.setControl(new Follower(motorID, followerInverted));
        }

        if (canCoderConfiguration != null) {
            canCoder = new CANcoder(canCoderID, canCoderCanBus);
            canCoder.getConfigurator().apply(canCoderConfiguration);
        }
    }

    @Override
    public void setMMPositionTarget(Angle setpoint, int slot) {
        this.motor.setControl(new MotionMagicVoltage(setpoint).withEnableFOC(this.enableFOC).withSlot(slot));
    }

    @Override
    public void setMMExpoTarget(Angle setpoint, int slot) {
        this.motor.setControl(new MotionMagicExpoVoltage(setpoint).withEnableFOC(this.enableFOC).withSlot(slot));
    }

    @Override
    public void setMMVelocityTarget(AngularVelocity velocity, int slot) {
        this.motor.setControl(new MotionMagicVelocityVoltage(velocity).withEnableFOC(this.enableFOC).withSlot(slot));
    }

    @Override
    public void setPIDPositionTarget(Angle setpoint, int slot) {
        this.motor.setControl(new PositionVoltage(setpoint).withEnableFOC(this.enableFOC).withSlot(slot));
    }

    @Override
    public void setPIDVelocityTarget(AngularVelocity velocity, int slot) {
        this.motor.setControl(new VelocityVoltage(velocity).withEnableFOC(this.enableFOC).withSlot(slot));
    }

    @Override
    public void setVelocityTorqueCurrentTarget(AngularVelocity velocity, double current, int slot) {
        this.motor.setControl(new VelocityTorqueCurrentFOC(velocity).withFeedForward(current).withSlot(slot));
    }

    @Override
    public void setDutyCycle(double percent) {
        this.motor.set(percent);
    }

    @Override
    public void setCoast() {
        this.motor.setNeutralMode(NeutralModeValue.Coast);
    }

    @Override
    public void setBrake() {
        this.motor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void enableFOC() {
        this.enableFOC = true;
    }

    @Override
    public void disableFOC() {
        this.enableFOC = false;
    }

    @Override
    public void stop() {
        this.motor.stopMotor();
    }

    @Override
    public Angle getPosition() {
        return this.motor.getPosition().getValue();
    }

    @Override
    public AngularVelocity getVelocity() {
        return this.motor.getVelocity().getValue();
    }

    @Override
    public AngularAcceleration getAcceleration() {
        return this.motor.getAcceleration().getValue();
    }

    @Override
    public void updateTelemetry(MotorTelemetry telemetry) {
        telemetry.position = motor.getPosition().getValue();
        telemetry.velocity = motor.getVelocity().getValue();
        telemetry.acceleration = motor.getAcceleration().getValue();
        telemetry.statorCurrent = motor.getStatorCurrent().getValue();
        telemetry.supplyCurrent = motor.getSupplyCurrent().getValue();
    }

    public TalonFX getMotor() {
        return motor;
    }

    public TalonFX getFollower() {
        return follower;
    }

    public CANcoder getCanCoder() {
        return canCoder;
    }
}