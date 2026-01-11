package raidzero.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.robot.Constants;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.turret.Shooter;
import raidzero.robot.subsystems.turret.Turret;

public class ShootWhileMoving extends Command {

    private final Shooter m_shooter;
    private final Turret m_turret;
    private final Swerve m_drive;

    public ShootWhileMoving(Shooter shooter, Turret turret, Swerve drive) {
        m_shooter = shooter;
        m_turret = turret;
        m_drive = drive;

        // TODO: Add requirements I forgot how to do this lol
    }

    @Override
    public void execute() {

        // State
        Pose2d currentPose = m_drive.getState().Pose;
        ChassisSpeeds robotSpeeds = m_drive.getState().Speeds;

        // Consider Latency
        double latencySeconds = 0.040; // Make constant? Get from NetworkTables? Physically test?
        Pose2d futurePose = currentPose.exp(
                new Twist2d(
                        robotSpeeds.vxMetersPerSecond * latencySeconds,
                        robotSpeeds.vyMetersPerSecond * latencySeconds,
                        robotSpeeds.omegaRadiansPerSecond * latencySeconds));

        // Rotation Compensation
        Translation2d turretOffset = Constants.Turret.TURRET_OFFSET;
        ChassisSpeeds turretPhysicalSpeeds = new ChassisSpeeds(
                robotSpeeds.vxMetersPerSecond - (robotSpeeds.omegaRadiansPerSecond * turretOffset.getY()),
                robotSpeeds.vyMetersPerSecond + (robotSpeeds.omegaRadiansPerSecond * turretOffset.getX()),
                robotSpeeds.omegaRadiansPerSecond);

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(turretPhysicalSpeeds,
                m_drive.getState().Pose.getRotation());

        // Actual Target
        Translation2d actualGoal = new Translation2d();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            actualGoal = Constants.Field.GOAL_LOCATION_RED;
        } else {
            actualGoal = Constants.Field.GOAL_LOCATION_BLUE;
        }

        // Dist to Target
        double staticDist = futurePose.getTranslation().getDistance(actualGoal);
        double shotTime = Constants.Shooter.TOF_TABLE.get(staticDist);

        // VirtualGoal = ActualGoal - (RobotVelocity * TimeOfFlight)
        double virtualX = actualGoal.getX() - (fieldSpeeds.vxMetersPerSecond * shotTime);
        double virtualY = actualGoal.getY() - (fieldSpeeds.vyMetersPerSecond * shotTime);
        Translation2d virtualGoal = new Translation2d(virtualX, virtualY);

        // Aiming
        Translation2d delta = virtualGoal.minus(futurePose.getTranslation());
        double virtualDist = delta.getNorm();
        Rotation2d targetYaw = delta.getAngle();

        // Commands
        // NOTE: Based on shooter design, spin-up might be direct or indirect. Order may change/matter.
        m_turret.setYaw(targetYaw);

        double targetPitch = Constants.Shooter.PITCH_TABLE.get(virtualDist);
        m_turret.setPitch(targetPitch);

        double targetRPM = Constants.Shooter.RPM_TABLE.get(virtualDist);
        m_shooter.setRPM(targetRPM);
    }

    @Override
    public void end(boolean interrupted) {
        m_shooter.stop();
        m_turret.stop();
    }
}