package raidzero.robot.subsystems.turret.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.drive.TunerConstants;
import raidzero.robot.subsystems.turret.Turret;
import raidzero.robot.subsystems.turret.TurretConstants.Yaw;

public class Aim extends Command {
    private final Turret turret = Turret.system();
    private final Swerve swerve = Swerve.system();

    public Aim() {
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        var robotPose = swerve.getState().Pose;
    }

    @Override
    public void execute() {
        Angle turretYaw = turret.getCurrentYaw();
        Pose2d robotPose = swerve.getState().Pose;

        // get the field centric poses
        var fieldRelativeGoalPose = Turret.getFieldRelativeGoalPose();
        var fieldRelativeTurretPose = Turret.getFieldRelativeTurretPose();

        var robotRelativeTurretSetpoint = fieldRelativeTurretPose.getRotation()
                .minus(fieldRelativeGoalPose.getRotation()).getRotations();
        double currentRotations = turretYaw.in(Rotations);
        double revLimit = Yaw.REVERSE_SOFT_LIMIT.in(Rotations);
        double fwdLimit = Yaw.FORWARD_SOFT_LIMIT.in(Rotations);

        double bestSetpoint = currentRotations;
        double bestDistance = Double.MAX_VALUE;

        for (int i = -2; i <= 2; i++) {
            double candidate = 0 + i;

            if (candidate >= revLimit && candidate <= fwdLimit) {
                double dist = Math.abs(candidate - currentRotations);
                if (dist < bestDistance) {
                    bestDistance = dist;
                    bestSetpoint = candidate;
                }
            }
        }

        SmartDashboard.putNumber("desired rotations", bestSetpoint);
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
