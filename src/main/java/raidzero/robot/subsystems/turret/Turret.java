package raidzero.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.R0Subsystem;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.turret.TurretConstants.Pitch;
import raidzero.robot.subsystems.turret.TurretConstants.Setpoints;
import raidzero.robot.subsystems.turret.TurretConstants.Yaw;

public class Turret extends R0Subsystem<TurretIO> {
    private static Turret system;
    private boolean isSpinningAround = false;
    private Angle currentTarget = Rotations.of(0.0);
    private static Swerve swerve = Swerve.system();

    /**
     * Aims the turret at the hub and holds position if there is no hub LOS
     * 
     * @return a {@link Command} to be scheduled and run
     */
    public Command aim() {
        return run(() -> {
            // Distance hubDistance = io.getHubDistance();

            Angle targetYaw = calculateTurretWrap();

            SmartDashboard.putNumber("Target Yaw", targetYaw.in(Degrees));

            // Angle targetPitch = Rotations.of(
            // Setpoints.PITCH_TABLE.get(hubDistance.in(Meters)));
            // SmartDashboard.putNumber("Target Pitch", targetPitch.in(Degrees));

            // io.moveTo(targetPitch, targetYaw);
            io.setYaw(targetYaw);
        });
    }

    public Command moveSimple(double percent) {
        return run(() -> io.runYawSimple(percent));
    }

    private Angle calculateTurretWrap() {
        Angle yawOffset = io.getYawFromTarget();
        double yawOffsetRotations = yawOffset.in(Rotations);
        double currentYawRotations = io.getYaw().in(Rotations);

        double desiredYawRotations = currentYawRotations - yawOffsetRotations;
        double targetYawRotations = currentYawRotations;

        double forwardLimit = Yaw.FORWARD_SOFT_LIMIT.in(Rotations);
        double reverseLimit = Yaw.REVERSE_SOFT_LIMIT.in(Rotations);

        if (desiredYawRotations <= forwardLimit && desiredYawRotations >= reverseLimit) {
            return Rotations.of(desiredYawRotations);
        } else if (desiredYawRotations < reverseLimit) {
            // Wrap forward by 1 rotation
            targetYawRotations = desiredYawRotations + 1.0;
            // Clamp if still out of bounds
            if (targetYawRotations > forwardLimit) {
                targetYawRotations = forwardLimit;
                return Yaw.FORWARD_SOFT_LIMIT;
            }

            return Rotations.of(targetYawRotations);

        } else if (desiredYawRotations > forwardLimit) {
            // Wrap forward by 1 rotation
            targetYawRotations = desiredYawRotations + 1.0;
            // Clamp if still out of bounds
            if (targetYawRotations > forwardLimit) {
                targetYawRotations = forwardLimit;
                return Yaw.FORWARD_SOFT_LIMIT;
            }

            return Rotations.of(targetYawRotations);
        }
        return io.getYaw();
    }

    /**
     * Calculates the kinetic lead needed for kinetic shooting
     * 
     * @param turretYaw   the current turret yaw (robot-relative)
     * @param hubDistance the distance from the hub
     * @return the correction angle
     */
    private Angle calculateKineticLead(Angle turretYaw, Distance hubDistance) {
        double distMeters = hubDistance.in(Meters);
        if (distMeters <= 0.1) {
            return Degrees.of(0);
        }

        ChassisSpeeds robotSpeeds = swerve.getState().Speeds;

        // Account for turret offset from robot center (rotation compensation)
        ChassisSpeeds turretPhysicalSpeeds = new ChassisSpeeds(
                robotSpeeds.vxMetersPerSecond - (robotSpeeds.omegaRadiansPerSecond * Setpoints.TURRET_OFFSET.getY()),
                robotSpeeds.vyMetersPerSecond + (robotSpeeds.omegaRadiansPerSecond * Setpoints.TURRET_OFFSET.getX()),
                robotSpeeds.omegaRadiansPerSecond);

        // Convert to field-relative speeds
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
                turretPhysicalSpeeds,
                swerve.getState().Pose.getRotation());

        // Field-relative aim direction
        double robotHeadingRad = swerve.getState().Pose.getRotation().getRadians();
        double turretFieldAngleRad = robotHeadingRad + turretYaw.in(Radians);

        // Direction vector toward target (unit vector)
        double toTargetX = Math.cos(turretFieldAngleRad);
        double toTargetY = Math.sin(turretFieldAngleRad);

        // Perpendicular direction (90° CCW)
        double perpX = -toTargetY;
        double perpY = toTargetX;

        // Project velocities
        double sidewaysSpeed = fieldSpeeds.vxMetersPerSecond * perpX + fieldSpeeds.vyMetersPerSecond * perpY;
        double radialSpeed = fieldSpeeds.vxMetersPerSecond * toTargetX + fieldSpeeds.vyMetersPerSecond * toTargetY;

        // --- ITERATIVE TIME OF FLIGHT CALCULATION ---
        // 1. Initial guess at TOF based on current distance
        double shotTime = Setpoints.TOF_TABLE.get(distMeters);
        double virtualDist = distMeters;

        // 2. Iterate to find the "converged" virtual distance
        for (int i = 0; i < 3; i++) {
            // Calculate where the robot will be relative to the hub after shotTime
            // Moving toward/away from hub (radial) and side-to-side (sideways)
            double virtualX = distMeters - (radialSpeed * shotTime);
            double virtualY = -(sidewaysSpeed * shotTime);

            // Re-calculate the actual distance the ball travels through the air
            virtualDist = Math.sqrt(Math.pow(virtualX, 2) + Math.pow(virtualY, 2));

            // Update shotTime based on the distance the ball actually needs to travel
            shotTime = Setpoints.TOF_TABLE.get(virtualDist);
        }
        // --------------------------------------------

        // How far sideways the robot will move during shot flight
        double sidewaysDisplacement = sidewaysSpeed * shotTime;

        // Calculate lead angle
        double leadRad = Math.atan2(-sidewaysDisplacement, distMeters);

        return Radians.of(leadRad);
    }

    /**
     * Calculates the angle that the turret is at relative to the robot using the
     * bot's startup position & turret limelight angle
     */
    public void setStartupAngle() {
        Angle swerveAngle = swerve.getState().Pose.getRotation().getMeasure();
        Angle llAngle = io.getYawFromTarget();

        // io.setStartupYaw(swerveAngle.minus(llAngle));
        io.setStartupPitch(Pitch.STARTING_ANGLE);
    }

    /**
     * Moves the pitch and yaw
     * 
     * @param pitch
     * @param yaw
     * @return a {@link Command} to be scheduled and run
     */
    public Command moveTo(Angle pitch, Angle yaw) {
        return run(() -> io.moveTo(pitch, yaw));
    }

    /**
     * Moves the yaw to the specified angle
     * 
     * @param yaw
     */
    public void setYaw(Angle yaw) {
        io.setYaw(yaw);
    }

    /**
     * Moves the pitch to the specified angle
     * 
     * @param pitch
     */
    public void setPitch(Angle pitch) {
        io.setPitch(pitch);
    }

    /**
     * Stops both the pitch and the yaw motors
     * 
     * @return a {@link Command} to be scheduled and run
     */
    public Command stop() {
        return run(() -> io.stopMotors());
    }

    /**
     * Gets the distance from the hub april tag to the turret
     * 
     * @return
     */
    public Distance getHubDistance() {
        return io.getHubDistance();
    }

    /**
     * Checks if the turret limelight can see the hub april tags on the scoring side
     * 
     * @return a {@link Trigger} object
     */
    public Trigger hasHubLOS() {
        return new Trigger(() -> io.hasHubLOS());
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("LL Turret Angle", io.getYawFromTarget().in(Degrees));
        SmartDashboard.putNumber("LL Turret Distance", io.getHubDistance().in(Meters));

        SmartDashboard.putNumber("Turret Angle", io.getYaw().in(Rotations));

        SmartDashboard.putBoolean("Is spinning", isSpinningAround);
    }

    private Turret(TurretIO io) {
        super(io);
    }

    public static Turret system() {
        if (system == null) {
            if (Utils.isSimulation()) {
                system = new Turret(new TurretIO.Sim());
            } else {
                system = new Turret(new TurretIO.Real());
            }
        }
        return system;
    }

}
