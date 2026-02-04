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
    private static final Turret turret = Turret.system();
    private static final Swerve swerve = Swerve.system();
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    NetworkTable table = null;
    StructPublisher<Pose2d> fieldRelativeGoalSender = null;
    StructPublisher<Pose2d> fieldRelativeTurretSender = null;

    private Pose2d fieldRelativeGoalPose = null;
    private Pose2d fieldRelatieTurretPose = null;
    private Translation2d turretMountFromRobotCenter = new Translation2d(Inches.of(-5.106), Inches.of(-4.85));

    public Aim() {
        addRequirements(turret);
        table = nt.getTable("Aiming Table");
    }

    @Override
    public void initialize() {
        var robotPose = swerve.getState().Pose;
        var turretRelativeToRobot = robotPose.plus(turretMountFromRobotCenter);

        fieldRelatieTurretPose = new Pose2d();
        // var goalTranslation = new Translation2d(Inches.of(182.11), Inches.of(158.84));
        // fieldRelativeGoalPose = new Pose2d(new Translation2d(Inches.of(182.11), Inches.of(158.84)), fieldRelativeGoalPose.getTranslation().minus(robotPose.getTranslation()).getAngle());
        fieldRelativeGoalSender = table.getStructTopic("goalPoseAbs", Pose2d.struct).publish();
        fieldRelativeTurretSender = table.getStructTopic("turretPose", Pose2d.struct).publish();
    }

    @Override
    public void execute() {
        Angle turretYaw = turret.getCurrentYaw();
        Pose2d robotPose = swerve.getState().Pose;
        var turretFromRobot = robotPose.plus(turretMountFromRobotCenter);
        fieldRelatieTurretPose = new Pose2d(turretFromRobot.getTranslation(), turretFromRobot.getRotation().plus(new Rotation2d(turretYaw)));

        // subtract the more accurate pose: odometry or poseEstimator
        fieldRelativeGoalPose = new Pose2d(fieldRelativeGoalPose.getTranslation(), fieldRelativeGoalPose.getTranslation().minus(fieldRelatieTurretPose.getTranslation()).getAngle());
        // this line will do robot centric, but as the robot base rotatoes the goal angle rotates
        // Pose2d goalFromRobot = new Pose2d(goalPose.getTranslation(), goalPose.relativeTo(robotPose).getTranslation().getAngle());
        
        double desiredRotations = fieldRelativeGoalPose.getRotation().plus(Rotation2d.kPi).minus(robotPose.getRotation()).getRotations();
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
        
        SmartDashboard.putNumber("desired rotations", bestSetpoint);
        turret.moveYawTo(Rotations.of(bestSetpoint));
        
        fieldRelativeGoalSender.set(fieldRelativeGoalPose);
        fieldRelativeTurretSender.set(fieldRelatieTurretPose);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        turret.stop();
        fieldRelativeGoalSender.close();
    }
}
