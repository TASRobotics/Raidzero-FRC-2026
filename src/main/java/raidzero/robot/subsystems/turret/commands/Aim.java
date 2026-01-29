package raidzero.robot.subsystems.turret.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.turret.Turret;
import raidzero.robot.subsystems.turret.TurretConstants.Yaw;

public class Aim extends Command {
    private static final Turret turret = Turret.system();
    private static final Swerve swerve = Swerve.system();

    private Pose2d aimPose = null;

    public Aim() {
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        // Initial Goal angle (currently hardcoded for ID 26 blue hub)
        aimPose = new Pose2d(new Translation2d(Inches.of(158.34), Inches.of(158.84)), new Rotation2d());
    }

    @Override
    public void execute() {
        Angle turretYaw = turret.getCurrentYaw();
        Rotation2d robotHeading = swerve.getState().Pose.getRotation();

        Rotation2d targetFieldAngle = new Rotation2d();

        // 1. Update target's abosolute field angle if Vision is Available
        if (turret.hasHubLOS().getAsBoolean()) {
            Angle visionOffset = turret.getYawFromTarget();
            // target's Field Angle = Robot Heading + Turret Yaw - Vision Offset
            targetFieldAngle = robotHeading.plus(new Rotation2d(turretYaw)).minus(new Rotation2d(visionOffset));

            // Update the persistent aim pose with new rotation
            aimPose = new Pose2d(aimPose.getTranslation(), targetFieldAngle);
        } else if (aimPose != null) {
            // Keep the old field angle (lock direction)
            targetFieldAngle = aimPose.getRotation();
        } else {
            // not needed, as likely aimPose will never be null
            // if it is null and we don't have a LOS just skip this iteration
            return;
        }

        // 2. Calculate Desired Robot-Relative turret Yaw
        // turret Yaw = Target Field Angle - Current Robot Heading
        Rotation2d desiredRelativeYaw = targetFieldAngle.minus(robotHeading);

        // 3. Select Best Wrapped Setpoint
        double desiredRotations = desiredRelativeYaw.getRotations();
        double currentRotations = turretYaw.in(Rotations);
        double revLimit = Yaw.REVERSE_SOFT_LIMIT.in(Rotations);
        double fwdLimit = Yaw.FORWARD_SOFT_LIMIT.in(Rotations);

        double bestSetpoint = currentRotations;
        double bestDistance = Double.MAX_VALUE;

        for (int i = -2; i <= 2; i++) {
            double candidate = desiredRotations + i;

            if (candidate >= revLimit && candidate <= fwdLimit) {
                double dist = Math.abs(candidate - currentRotations);
                if (dist < bestDistance) {
                    bestDistance = dist;
                    bestSetpoint = candidate;
                }
            }
        }

        turret.moveYawTo(Rotations.of(bestSetpoint));
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
