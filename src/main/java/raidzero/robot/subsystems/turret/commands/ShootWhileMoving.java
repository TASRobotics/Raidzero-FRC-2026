package raidzero.robot.subsystems.turret.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

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
import raidzero.robot.subsystems.turret.Turret;
import raidzero.robot.subsystems.turret.TurretConstants;
import raidzero.robot.subsystems.turret.TurretConstants.Limelight;
import raidzero.robot.subsystems.turret.TurretConstants.Setpoints;
import static raidzero.robot.subsystems.turret.TurretConstants.*;

public class ShootWhileMoving extends Command {

    private final Turret turret;
    private final Swerve swerve;

    public ShootWhileMoving(Turret turret, Swerve swerve) {
        this.turret = turret;
        this.swerve = swerve;

        this.addRequirements(turret);
    }

    @Override
    public void execute() {

        // State
        Pose2d currentPose = swerve.getState().Pose;
        ChassisSpeeds robotSpeeds = swerve.getState().Speeds;

        // Consider Latency
        Pose2d futurePose = currentPose.exp(
                new Twist2d(
                        robotSpeeds.vxMetersPerSecond * Limelight.LATENCY.in(Seconds),
                        robotSpeeds.vyMetersPerSecond * Limelight.LATENCY.in(Seconds),
                        robotSpeeds.omegaRadiansPerSecond * Limelight.LATENCY.in(Seconds)));

        // Rotation Compensation
        ChassisSpeeds turretPhysicalSpeeds = new ChassisSpeeds(
                robotSpeeds.vxMetersPerSecond - (robotSpeeds.omegaRadiansPerSecond * Yaw.TURRET_MOUNT_OFFSET.getY()),
                robotSpeeds.vyMetersPerSecond + (robotSpeeds.omegaRadiansPerSecond * Yaw.TURRET_MOUNT_OFFSET.getX()),
                robotSpeeds.omegaRadiansPerSecond);

        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(turretPhysicalSpeeds,
                swerve.getState().Pose.getRotation());

        // Actual Target
        Translation2d actualGoal = new Translation2d();
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            actualGoal = Constants.Field.GOAL_LOCATION_RED;
        } else {
            actualGoal = Constants.Field.GOAL_LOCATION_BLUE;
        }

        // --- ITERATIVE TIME OF FLIGHT CALCULATION ---
        // 1. Initial guess at TOF based on current distance
        double staticDist = futurePose.getTranslation().getDistance(actualGoal);
        double shotTime = Setpoints.TOF_TABLE.get(staticDist);

        double virtualDist = staticDist;
        Translation2d virtualGoal = actualGoal;

        // 2. Iterate to find the "converged" virtual distance
        // Doing this three times is usually enough for FRC speeds to hit sub-centimeter
        // accuracy
        for (int i = 0; i < 3; i++) {
            double virtualX = actualGoal.getX() - (fieldSpeeds.vxMetersPerSecond * shotTime);
            double virtualY = actualGoal.getY() - (fieldSpeeds.vyMetersPerSecond * shotTime);
            virtualGoal = new Translation2d(virtualX, virtualY);

            // Re-calculate distance to the virtual goal
            virtualDist = virtualGoal.getDistance(futurePose.getTranslation());

            // Update shotTime based on the distance the ball actually needs to travel
            shotTime = Setpoints.TOF_TABLE.get(virtualDist);
        }

        // Aiming
        Translation2d delta = virtualGoal.minus(futurePose.getTranslation());
        virtualDist = delta.getNorm();
        Rotation2d targetYaw = delta.getAngle();

        // Commands
        // NOTE: Based on shooter design, spin-up might be direct or indirect. Order may
        // change/matter.
        turret.moveYawTo(targetYaw.getMeasure());

        double targetPitch = TurretConstants.Setpoints.PITCH_TABLE.get(virtualDist);
        turret.setPitch(Degrees.of(targetPitch));
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }
}