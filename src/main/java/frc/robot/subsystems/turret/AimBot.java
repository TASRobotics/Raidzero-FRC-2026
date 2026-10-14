package frc.robot.subsystems.turret;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import frc.robot.subsystems.turret.TurretConstants.Kinetics;

/**
 * All field centic poses
 */
public record AimBot(Pose2d robot, Pose2d turret, Pose2d goal) {

    public static final double CODE_LOOP_LATENCY = 0.03;

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
    public AimBot adjustKineticLead(ChassisVelocities robotSpeeds) {
        Pose2d kineticRobotPose = calculateKineticRobotPose(robotSpeeds, robot);

        Translation2d turretOffsetRotated = Kinetics.TURRET_MOUNT_OFFSET.rotateBy(kineticRobotPose.getRotation());
        Pose2d kineticTurretPose = new Pose2d(kineticRobotPose.getTranslation().plus(turretOffsetRotated),
                turret.getRotation());

        var whipSpeeds = calculateWhip(robotSpeeds, kineticRobotPose);
        Pose2d kineticGoalPose = calculateKineticGoal(whipSpeeds, kineticTurretPose, goal);

        return new AimBot(kineticRobotPose, kineticTurretPose, kineticGoalPose);
    }

    /**
     * add in code latency to next motor update to account for vel added to ball
     * turret omega cancels out as it is counter to robot omega
     *
     * @param currentSpeeds current robot chasis speeds
     * @param currentPose   current robot pose
     * @return kinetic turret pose
     */
    private Pose2d calculateKineticRobotPose(ChassisVelocities currentSpeeds, Pose2d currentPose) {
        double dx = currentSpeeds.vx * CODE_LOOP_LATENCY;
        double dy = currentSpeeds.vy * CODE_LOOP_LATENCY;
        double dTheta = currentSpeeds.omega * CODE_LOOP_LATENCY;

        Pose2d robotFuture = currentPose.plus(new Twist2d(dx, dy, dTheta).exp());
        return robotFuture;
    }

    /**
     * because turret isnt in center of robot rotation
     * robot omega will cause tangential velocity at the turret
     * chasisspeeds + (chassisSpeeds.omega * turretOffsetRadius) = whip
     * linear on ball + tangential on ball = exit velocity
     *
     * @param relativeSpeeds current robot chasis speeds
     * @return Whip speeds as a {@link ChassisVelocities}
     */
    private ChassisVelocities calculateWhip(ChassisVelocities relativeSpeeds, Pose2d futurePose) {
        var vx = relativeSpeeds.vx
                - relativeSpeeds.omega * Kinetics.TURRET_MOUNT_OFFSET.getY();
        var vy = relativeSpeeds.vy
                + relativeSpeeds.omega * Kinetics.TURRET_MOUNT_OFFSET.getX();
        var omega = relativeSpeeds.omega;
        return new ChassisVelocities(vx, vy, omega).toRobotRelative(futurePose.getRotation());
    }

    /**
     * Calculates the pose of the goal with a kinetic lead for SOTM
     * @param whipSpeeds the whip speeds of the robot
     * @param kineticTurret the kinetic pose of the turret
     * @param staticGoal the static pose of the goal
     * @return the {@link Pose2d} of the goal with a kinetic lead
     */
    private Pose2d calculateKineticGoal(ChassisVelocities whipSpeeds, Pose2d kineticTurret, Pose2d staticGoal) {
        Translation2d virtualGoalTranslation = staticGoal.getTranslation();
        double turretToGoalDist = kineticTurret.getTranslation().getDistance(virtualGoalTranslation);

        double currentDistanceTOF = Kinetics.TOF_TABLE.get(turretToGoalDist);

        for (int i = 0; i < 7; i++) {
            double virtualGoalx = staticGoal.getX()
                    - currentDistanceTOF * whipSpeeds.vx;
            double virtualGoaly = staticGoal.getY()
                    - currentDistanceTOF * whipSpeeds.vy;
            virtualGoalTranslation = new Translation2d(virtualGoalx, virtualGoaly);

            turretToGoalDist = kineticTurret.getTranslation()
                    .getDistance(virtualGoalTranslation);
            currentDistanceTOF = Kinetics.TOF_TABLE.get(turretToGoalDist);
        }

        Rotation2d turretToVirtualGoalFieldAngle = virtualGoalTranslation.minus(kineticTurret.getTranslation()).getAngle();
        Pose2d virtualGoalPose = new Pose2d(virtualGoalTranslation, turretToVirtualGoalFieldAngle);

        return virtualGoalPose;
    }
}
