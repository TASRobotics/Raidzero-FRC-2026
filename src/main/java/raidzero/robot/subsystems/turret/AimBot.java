package raidzero.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import raidzero.robot.subsystems.turret.TurretConstants.Kinetics;
import raidzero.robot.subsystems.turret.TurretConstants.Yaw;

public record AimBot(Pose2d robot, Pose2d turret, Pose2d goal) {

    public static final double CODE_LOOP_LATENCY = 0.02;

    /**
     * Adjusts the turret aim to the static goal location without kinetic lead
     * 
     * @param currentTurretYaw     the current turret angle
     * @param allianceGoalLocation the current alliance goal location
     * @return the adjusted turret aim
     */
    public AimBot adjustStaticAim(Pose2d robotPose, Rotation2d currentTurretYaw, Translation2d allianceGoalLocation) {
        // where is turret center offset from robot center
        var turretOffsetVectorFieldCentric = Yaw.TURRET_MOUNT_OFFSET.rotateBy(robotPose.getRotation());
        var fieldRelativeTurret = robotPose.getTranslation().plus(turretOffsetVectorFieldCentric);

        // where is turret angle from field zero (+x axis)
        var fieldRelativeTurretAngle = robotPose.getRotation().plus(currentTurretYaw);
        var fieldRelativeTurretPose = new Pose2d(fieldRelativeTurret, fieldRelativeTurretAngle);

        // what field angle should the turret be at to point at the goal
        var turretToGoalVector = allianceGoalLocation.minus(fieldRelativeTurretPose.getTranslation());
        var fieldRelativeGoalPose = new Pose2d(allianceGoalLocation, turretToGoalVector.getAngle());
        return new AimBot(robotPose, fieldRelativeTurretPose, fieldRelativeGoalPose);
    }

    /**
     * Adjusts the turret aim for the current robot movement
     * 
     * @param robotSpeeds the current robot speeds
     * @return the adjusted turret aim with kinetic lead
     */
    public AimBot adjustKineticLead(ChassisSpeeds robotSpeeds) {
        var kineticRobotPose = calculateKineticRobotPose(robotSpeeds, robot);
        var turretOffsetRotated = Yaw.TURRET_MOUNT_OFFSET.rotateBy(kineticRobotPose.getRotation());
        var kineticTurretPose = new Pose2d(kineticRobotPose.getTranslation().plus(turretOffsetRotated),
                turret.getRotation());
        var whipSpeeds = calculateWhip(robotSpeeds, robot);
        var kineticGoalPose = calculateKineticGoal(whipSpeeds, kineticTurretPose, goal);
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
    private Pose2d calculateKineticRobotPose(ChassisSpeeds currentSpeeds, Pose2d currentPose) {
        var dx = currentSpeeds.vxMetersPerSecond * CODE_LOOP_LATENCY;
        var dy = currentSpeeds.vyMetersPerSecond * CODE_LOOP_LATENCY;
        var dTheta = currentSpeeds.omegaRadiansPerSecond * CODE_LOOP_LATENCY;
        var robotFuture = currentPose.exp(new Twist2d(dx, dy, dTheta));
        return robotFuture;
    }

    /**
     * because turret isnt in center of robot rotation
     * robot omeega will cause tangential velocity at the turret
     * chasisspeeds + (chassisSpeeds.omega * turretOffsetRadius) = whip
     * linear on ball + tangential on ball = exit velocity
     * 
     * @param currentSpeeds current robot chasis speeds
     * @return
     */
    private ChassisSpeeds calculateWhip(ChassisSpeeds currentSpeeds, Pose2d currentPose) {
        var vx = currentSpeeds.vxMetersPerSecond - currentSpeeds.omegaRadiansPerSecond * Yaw.TURRET_MOUNT_OFFSET.getY();
        var vy = currentSpeeds.vyMetersPerSecond + currentSpeeds.omegaRadiansPerSecond * Yaw.TURRET_MOUNT_OFFSET.getX();
        var omega = currentSpeeds.omegaRadiansPerSecond;
        return ChassisSpeeds.fromRobotRelativeSpeeds(vx, vy, omega, currentPose.getRotation());
    }

    private Pose2d calculateKineticGoal(ChassisSpeeds whipSpeeds, Pose2d kineticTurret,
            Pose2d staticGoal) {
        var virtualGoalTranslation = staticGoal.getTranslation();
        var turretToGoalDist = kineticTurret.getTranslation().getDistance(virtualGoalTranslation);
        var currentDistanceTOF = Kinetics.TOF_TABLE.get(turretToGoalDist);

        for (int i = 0; i < 3; i++) {
            var virtualGoalx = virtualGoalTranslation.getX()
                    - currentDistanceTOF * whipSpeeds.vxMetersPerSecond;
            var virtualGoaly = virtualGoalTranslation.getY()
                    - currentDistanceTOF * whipSpeeds.vyMetersPerSecond;
            virtualGoalTranslation = new Translation2d(virtualGoalx, virtualGoaly);
            turretToGoalDist = kineticTurret.getTranslation()
                    .getDistance(virtualGoalTranslation);
            currentDistanceTOF = Kinetics.TOF_TABLE.get(turretToGoalDist);
        }
        var turretToVirtualGoalFieldAngle = virtualGoalTranslation.minus(kineticTurret.getTranslation()).getAngle();
        var virtualGoalPose = new Pose2d(virtualGoalTranslation, turretToVirtualGoalFieldAngle);
        return virtualGoalPose;
    }
}
