package raidzero.robot.subsystems.turret;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import raidzero.robot.subsystems.turret.TurretConstants.Kinetics;

/**
 * All field centric poses
 */
public record AimBot(Pose2d robot, Pose2d turret, Pose2d goal) {

    public static final double CODE_LOOP_LATENCY = 0.04;

    /**
     * Adjusts the turret aim to the static goal location without kinetic lead
     *
     * @param currentTurretYaw     the current turret angle
     * @param allianceGoalLocation the current alliance goal location
     * @return the adjusted turret aim
     */
    public AimBot adjustStaticAim(Pose2d robotPose, Rotation2d currentTurretYaw, Translation2d allianceGoalLocation) {
        // where is turret center offset from robot center
        Translation2d turretOffsetVectorFieldCentric = Kinetics.TURRET_MOUNT_OFFSET.rotateBy(robotPose.getRotation());
        Translation2d fieldRelativeTurret = robotPose.getTranslation().plus(turretOffsetVectorFieldCentric);

        // where is turret angle from field zero (+x axis)
        Rotation2d fieldRelativeTurretAngle = robotPose.getRotation().plus(currentTurretYaw);
        Pose2d fieldRelativeTurretPose = new Pose2d(fieldRelativeTurret, fieldRelativeTurretAngle);

        // what field angle should the turret be at to point at the goal
        Translation2d turretToGoalVector = allianceGoalLocation.minus(fieldRelativeTurretPose.getTranslation());
        Pose2d fieldRelativeGoalPose = new Pose2d(allianceGoalLocation, turretToGoalVector.getAngle());

        return new AimBot(robotPose, fieldRelativeTurretPose, fieldRelativeGoalPose);
    }

    /**
     * Adjusts the turret aim for the current robot movement
     *
     * @param robotSpeeds the current robot speeds
     * @return the adjusted turret aim with kinetic lead
     */
    public AimBot adjustKineticLead(ChassisSpeeds robotSpeeds) {
        Pose2d kineticRobotPose = calculateKineticRobotPose(robotSpeeds, robot);

        Translation2d turretOffsetRotated = Kinetics.TURRET_MOUNT_OFFSET.rotateBy(kineticRobotPose.getRotation());
        Pose2d kineticTurretPose = new Pose2d(kineticRobotPose.getTranslation().plus(turretOffsetRotated),
                turret.getRotation());

        var whipSpeeds = calculateWhip(robotSpeeds, kineticRobotPose);
        Pose2d kineticGoalPose = calculateKineticGoal(whipSpeeds, kineticTurretPose, goal);

        return new AimBot(kineticRobotPose, kineticTurretPose, kineticGoalPose);
    }

    /**
     * Calculates the total velocity of the turret mount moving toward or away from the goal.
     * @param whipSpeeds The field-relative speeds of the turret (includes robot v + omega whip)
     * @return LinearVelocity (Positive = closing distance, Negative = opening distance)
     */
    public LinearVelocity getRadialVelocity(ChassisSpeeds whipSpeeds) {
        Translation2d vectorToGoal = goal.getTranslation().minus(turret.getTranslation());
        Rotation2d angleToGoal = vectorToGoal.getAngle();
        
        // Project the whip speeds onto the angle to the goal to get the radial velocity (Dot product)
        double mps = (whipSpeeds.vxMetersPerSecond * angleToGoal.getCos()) + 
                    (whipSpeeds.vyMetersPerSecond * angleToGoal.getSin());

        return MetersPerSecond.of(mps);
    }

    /**
     * add in code latency to next motor update to account for vel added to ball
     * turret omega cancels out as it is counter to robot omega
     *
     * @param currentSpeeds current robot chasis speeds
     * @param currentPose   current robot pose
     * @return kinetic turret pose
     */
    private Pose2d calculateKineticRobotPose(ChassisSpeeds currentSpeeds, Pose2d currentPose) {
        double dx = currentSpeeds.vxMetersPerSecond * CODE_LOOP_LATENCY;
        double dy = currentSpeeds.vyMetersPerSecond * CODE_LOOP_LATENCY;
        double dTheta = currentSpeeds.omegaRadiansPerSecond * CODE_LOOP_LATENCY;

        Pose2d robotFuture = currentPose.exp(new Twist2d(dx, dy, dTheta));
        return robotFuture;
    }

    /**
     * because turret isnt in center of robot rotation
     * robot omega will cause tangential velocity at the turret
     * chasisspeeds + (chassisSpeeds.omega * turretOffsetRadius) = whip
     * linear on ball + tangential on ball = exit velocity
     *
     * @param relativeSpeeds current robot chasis speeds
     * @return Whip speeds as a {@link ChassisSpeeds}
     */
    private ChassisSpeeds calculateWhip(ChassisSpeeds relativeSpeeds, Pose2d futurePose) {
        var vx = relativeSpeeds.vxMetersPerSecond
                - relativeSpeeds.omegaRadiansPerSecond * Kinetics.TURRET_MOUNT_OFFSET.getY();
        var vy = relativeSpeeds.vyMetersPerSecond
                + relativeSpeeds.omegaRadiansPerSecond * Kinetics.TURRET_MOUNT_OFFSET.getX();
        var omega = relativeSpeeds.omegaRadiansPerSecond;
        return ChassisSpeeds.fromRobotRelativeSpeeds(vx, vy, omega, futurePose.getRotation());
    }

    /**
     * Calculates the pose of the goal with a kinetic lead for SOTM
     * @param whipSpeeds the whip speeds of the robot
     * @param kineticTurret the kinetic pose of the turret
     * @param staticGoal the static pose of the goal
     * @return the {@link Pose2d} of the goal with a kinetic lead
     */
    private Pose2d calculateKineticGoal(ChassisSpeeds whipSpeeds, Pose2d kineticTurret, Pose2d staticGoal) {
        Translation2d virtualGoalTranslation = staticGoal.getTranslation();
        double turretToGoalDist = kineticTurret.getTranslation().getDistance(virtualGoalTranslation);

        double currentDistanceTOF = Kinetics.TOF_TABLE.get(turretToGoalDist);

        for (int i = 0; i < 15; i++) {
            // Include CODE_LOOP_LATENCY in TOF for the virtual goal projection
            double totalLookahead = currentDistanceTOF + CODE_LOOP_LATENCY;
            double virtualGoalx = staticGoal.getX() - totalLookahead * whipSpeeds.vxMetersPerSecond;
            double virtualGoaly = staticGoal.getY() - totalLookahead * whipSpeeds.vyMetersPerSecond;
            
            virtualGoalTranslation = new Translation2d(virtualGoalx, virtualGoaly);
            turretToGoalDist = kineticTurret.getTranslation().getDistance(virtualGoalTranslation);
            currentDistanceTOF = Kinetics.TOF_TABLE.get(turretToGoalDist);
        }

        Rotation2d turretToVirtualGoalFieldAngle = virtualGoalTranslation.minus(kineticTurret.getTranslation()).getAngle();
        Pose2d virtualGoalPose = new Pose2d(virtualGoalTranslation, turretToVirtualGoalFieldAngle);

        return virtualGoalPose;
    }
}
