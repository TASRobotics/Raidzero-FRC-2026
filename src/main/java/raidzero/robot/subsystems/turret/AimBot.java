package raidzero.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import raidzero.robot.subsystems.turret.TurretConstants.Kinetics;

/**
 * Immutable snapshot of all field-relative poses needed for shoot-on-the-move
 * (SOTM) aiming.
 * 
 * @param robot  field-relative robot pose (projected forward for kinetic
 *               instance)
 * @param turret field-relative turret pose, accounting for mount offset and
 *               current yaw
 * @param goal   field-relative goal — real goal for static, virtual goal for
 *               kinetic
 */
public record AimBot(Pose2d robot, Pose2d turret, Pose2d goal) {

    public static final double CODE_LOOP_LATENCY = 0.03;

    /**
     * Computes a fresh static {@link AimBot} from the current robot pose and turret
     * yaw.
     * Rotate the turret mount offset into field space and add to robot translation
     * to find where the turret pivot actually is on the field.
     * Add robot heading and turret yaw to get the turret's field-relative heading.
     * Compute the angle from the turret position to the real goal.
     *
     * @param robotPose            current field-relative robot pose
     * @param currentTurretYaw     current turret yaw relative to the robot
     * @param allianceGoalLocation field-relative position of the scoring goal
     * @return a new static {@link AimBot}
     */
    public AimBot adjustStaticAim(
            Pose2d robotPose,
            Rotation2d currentTurretYaw,
            Translation2d allianceGoalLocation) {

        // Rotate mount offset from robot-relative into field-relative
        Translation2d turretOffsetFieldRelative = Kinetics.TURRET_MOUNT_OFFSET.rotateBy(robotPose.getRotation());

        // Field-relative turret pivot position
        Translation2d fieldTurretPosition = robotPose.getTranslation().plus(turretOffsetFieldRelative);

        // Field-relative turret heading = robot heading + turret yaw
        Rotation2d fieldTurretHeading = robotPose.getRotation().plus(currentTurretYaw);
        Pose2d fieldTurretPose = new Pose2d(fieldTurretPosition, fieldTurretHeading);

        // Angle from turret pivot to real goal
        Translation2d turretToGoal = allianceGoalLocation.minus(fieldTurretPose.getTranslation());
        Pose2d goalPose = new Pose2d(allianceGoalLocation, turretToGoal.getAngle());

        return new AimBot(robotPose, fieldTurretPose, goalPose);
    }

    /**
     * Derives a kinetic {@link AimBot} from this static instance by accounting for
     * robot motion.
     * Project the robot pose forward by {@link #CODE_LOOP_LATENCY} to compensate
     * for
     * motor update latency.
     * Compute whip speeds — the field-relative velocity at the turret mount,
     * including
     * the tangential component from robot rotation about the offset mount point.
     * Iteratively solve for the virtual goal.
     *
     * @param filteredSpeeds filtered robot chassis speeds (robot-relative)
     * @return a new kinetic {@link AimBot} with virtual goal
     */
    public AimBot adjustKineticLead(ChassisSpeeds filteredSpeeds) {
        // Step 1: project robot forward one code loop
        Pose2d kineticRobotPose = projectRobotPose(filteredSpeeds, robot);

        // Recompute turret position from the projected robot pose
        Translation2d turretOffsetRotated = Kinetics.TURRET_MOUNT_OFFSET.rotateBy(kineticRobotPose.getRotation());
        Pose2d kineticTurretPose = new Pose2d(
                kineticRobotPose.getTranslation().plus(turretOffsetRotated),
                turret.getRotation());

        // Step 2: compute field-relative velocity at the turret mount
        ChassisSpeeds whipSpeeds = calculateWhipSpeeds(filteredSpeeds, kineticRobotPose);

        // Step 3: solve for virtual goal using whip speeds
        Pose2d virtualGoal = calculateVirtualGoal(whipSpeeds, kineticTurretPose, goal);

        return new AimBot(kineticRobotPose, kineticTurretPose, virtualGoal);
    }

    /**
     * Projects the robot pose forward by {@link #CODE_LOOP_LATENCY} seconds.
     *
     * @param speeds      current robot chassis speeds (robot-relative)
     * @param currentPose current field-relative robot pose
     * @return predicted field-relative robot {@link Pose2d} one code loop ahead
     */
    private Pose2d projectRobotPose(ChassisSpeeds speeds, Pose2d currentPose) {
        return currentPose.exp(new Twist2d(
                speeds.vxMetersPerSecond * CODE_LOOP_LATENCY,
                speeds.vyMetersPerSecond * CODE_LOOP_LATENCY,
                speeds.omegaRadiansPerSecond * CODE_LOOP_LATENCY));
    }

    /**
     * Computes the field-relative velocity at the turret mount point.
     * Because the turret mount is offset from the robot center, robot angular
     * velocity ω
     * generates a tangential velocity at the mount in addition to robot
     * translation:
     * vx_mount = vx_robot - ω * offset.y
     * vy_mount = vy_robot + ω * offset.x
     * 
     * Converted to field-relative using the kinetic robot heading.
     *
     * @param robotSpeeds robot-relative chassis speeds
     * @param kineticPose projected robot pose, used for robot-to-field rotation
     * @return field-relative {@link ChassisSpeeds} at the turret mount
     */
    private ChassisSpeeds calculateWhipSpeeds(ChassisSpeeds robotSpeeds, Pose2d kineticPose) {
        double vxMount = robotSpeeds.vxMetersPerSecond
                - robotSpeeds.omegaRadiansPerSecond * Kinetics.TURRET_MOUNT_OFFSET.getY();
        double vyMount = robotSpeeds.vyMetersPerSecond
                + robotSpeeds.omegaRadiansPerSecond * Kinetics.TURRET_MOUNT_OFFSET.getX();

        return ChassisSpeeds.fromRobotRelativeSpeeds(
                vxMount, vyMount, robotSpeeds.omegaRadiansPerSecond,
                kineticPose.getRotation());
    }

    /**
     * Iteratively solves for the virtual goal position.
     *
     * The virtual goal is the aim point such that a ball carrying whip speeds for
     * the
     * duration of its time of flight (TOF) arrives at the real goal:
     * virtual_goal = real_goal - whip * (TOF(dist) + CODE_LOOP_LATENCY)
     * dist = distance(turret, virtual_goal)
     * repeat until convergence (~5 iterations; 15 is conservative)
     *
     * Sign convention: if the turret is moving in +x, we aim at a point offset in
     * -x from the real goal. The inherited +x velocity carries the ball back to the
     * real goal
     * during flight.
     *
     * @param whipSpeeds    field-relative velocity at the turret mount
     * @param kineticTurret projected turret pose
     * @param staticGoal    the real (non-virtual) goal pose
     * @return converged virtual goal {@link Pose2d}
     */
    private Pose2d calculateVirtualGoal(ChassisSpeeds whipSpeeds, Pose2d kineticTurret, Pose2d staticGoal) {

        Translation2d virtualGoal = staticGoal.getTranslation();
        double dist = kineticTurret.getTranslation().getDistance(virtualGoal);
        double tof = Kinetics.TOF_TABLE.get(dist);

        for (int i = 0; i < 15; i++) {
            double lookahead = tof + CODE_LOOP_LATENCY;

            virtualGoal = new Translation2d(
                    staticGoal.getX() - lookahead * whipSpeeds.vxMetersPerSecond,
                    staticGoal.getY() - lookahead * whipSpeeds.vyMetersPerSecond);

            dist = kineticTurret.getTranslation().getDistance(virtualGoal);
            tof = Kinetics.TOF_TABLE.get(dist);
        }

        return new Pose2d(
                virtualGoal,
                virtualGoal.minus(kineticTurret.getTranslation()).getAngle());
    }
}
