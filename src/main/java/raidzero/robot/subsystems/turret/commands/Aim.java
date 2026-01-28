package raidzero.robot.subsystems.turret.commands;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.turret.Turret;

import static raidzero.robot.subsystems.turret.TurretConstants.Yaw;

public class Aim extends Command {
    private static Turret turret = Turret.system();
    private static Swerve swerve = Swerve.system();
    private boolean spinnySpin;
    private boolean unwrappingToForwardLimit;

    public Aim() {
        this.addRequirements(turret);
    }

    @Override
    public void initialize() {
        spinnySpin = false;
        unwrappingToForwardLimit = false;
    }

    @Override
    public void execute() {
        Angle yawOffset = turret.getYawFromTarget();
        double yawOffsetRotations = yawOffset.in(Rotations);
        double currentYawRotations = turret.getCurrentYaw().in(Rotations);

        double forwardLimit = Yaw.FORWARD_SOFT_LIMIT.in(Rotations);
        double reverseLimit = Yaw.REVERSE_SOFT_LIMIT.in(Rotations);

        boolean nearForwardLimit = Math.abs(currentYawRotations - forwardLimit) <= Yaw.POSITION_TOLERANCE.in(Rotations);
        boolean nearReverseLimit = Math.abs(currentYawRotations - reverseLimit) <= Yaw.POSITION_TOLERANCE.in(Rotations);

        double desiredYawRotations = currentYawRotations - yawOffsetRotations;

        SmartDashboard.putBoolean("Near Forward Limit", nearForwardLimit);
        SmartDashboard.putBoolean("Near reverse limit", nearReverseLimit);
        SmartDashboard.putBoolean("spinnyspin", spinnySpin);

        SmartDashboard.putNumber("Desired yaw", desiredYawRotations);

        if (spinnySpin) {
            boolean nearLimit = (unwrappingToForwardLimit && nearForwardLimit)
                    || (!unwrappingToForwardLimit && nearReverseLimit);

            if (nearLimit && turret.hasHubLOS().getAsBoolean()) {
                spinnySpin = false;
            } else {
                AngularVelocity minvVelocity = unwrappingToForwardLimit ? Yaw.MIN_UNWRAP_VELOCITY
                        : Yaw.MIN_UNWRAP_VELOCITY.times(-1.0);
                turret.setYawVelocity(RadiansPerSecond.of(-1.0 *
                        swerve.getState().Speeds.omegaRadiansPerSecond).plus(minvVelocity));
            }
        } else {

            if (desiredYawRotations <= forwardLimit && desiredYawRotations >= reverseLimit) {
                turret.moveYawTo(Rotations.of(desiredYawRotations));
            }

            if (desiredYawRotations <= reverseLimit) {
                unwrappingToForwardLimit = true;
                spinnySpin = true;
            } else if (desiredYawRotations >= forwardLimit) {
                unwrappingToForwardLimit = false;
                spinnySpin = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}
