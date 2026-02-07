package raidzero.lib.wrappers.motors;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class LazyFXS implements LazyCTRE, LazyMotor<TalonFXS> {
    private TalonFXS motor, follower;
    private CANcoder canCoder;

    public LazyFXS(int motorID, String canBus, TalonFXSConfiguration configuration, int followID, String followCanBus,
            TalonFXSConfiguration followConfiguration, MotorAlignmentValue followerInverted, int canCoderID,
            String canCoderCanBus, CANcoderConfiguration canCoderConfiguration) {
        motor = new TalonFXS(motorID, canBus);
        motor.getConfigurator().apply(configuration);

        if (followConfiguration != null) {
            follower = new TalonFXS(followID, followCanBus);
            follower.getConfigurator().apply(followConfiguration);
            follower.setControl(new Follower(motorID, followerInverted));
        }

        if (canCoderConfiguration != null) {
            canCoder = new CANcoder(canCoderID, canCoderCanBus);
            canCoder.getConfigurator().apply(canCoderConfiguration);
        }
    }

    @Override
    public void moveTo(Angle setpoint) {
        motor.setControl(new MotionMagicExpoVoltage(setpoint).withEnableFOC(true));
    }

    public void moveWithMMVolt(Angle setpoint, int slotNumber) {
        motor.setControl(new MotionMagicVoltage(setpoint).withEnableFOC(true).withSlot(slotNumber));
    }

    @Override
    public void setVelocity(AngularVelocity velocity, int slotNumber) {
        motor.setControl(new VelocityVoltage(velocity).withEnableFOC(true).withSlot(slotNumber));
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
    public TalonFXS getMotor() {
        return motor;
    }

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