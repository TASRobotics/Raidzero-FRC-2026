package raidzero.robot.subsystems.drive;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import raidzero.lib.LimelightHelpers;

public class Limelight extends SubsystemBase {
    private boolean ignoreFrontLime = false;
    private boolean ignoreRearLime = false;
    private boolean ignoreLeftLime = false;
    private boolean ignoreRightLime = false;
    private boolean ignoreAllLimes = false;

    private StructPublisher<Pose2d> frontNT = NetworkTableInstance.getDefault().getStructTopic("frontNT", Pose2d.struct).publish();
    private StructPublisher<Pose2d> rearNT = NetworkTableInstance.getDefault().getStructTopic("rearNT", Pose2d.struct).publish();
    private StructPublisher<Pose2d> leftNT = NetworkTableInstance.getDefault().getStructTopic("leftNT", Pose2d.struct).publish();
    private StructPublisher<Pose2d> rightNT = NetworkTableInstance.getDefault().getStructTopic("rightNT", Pose2d.struct).publish();

    private LimelightHelpers.PoseEstimate limeFront, limeRear, limeLeft, limeRight;
    private LimelightHelpers.PoseEstimate limeFrontPrev, limeRearPrev, limeLeftPrev, limeRightPrev;

    private Notifier notifier;

    private Swerve swerve = Swerve.system();
    private static Limelight instance = null;

    /**
     * Constructs a {@link Limelight} subsytem instance
     */
    private Limelight() {
        this.startThread();
    }

    /**
     * Sets the pipeline of the limelight
     *
     * @param limelightName The name of the limelight
     * @param pipeline The pipeline index
     */
    public void setPipeline(String limelightName, int pipeline) {
        LimelightHelpers.setPipelineIndex(limelightName, pipeline);
    }

    /**
     * Starts the Limelight odometry thread
     */
    private void startThread() {
        notifier = new Notifier(this::loop);
        notifier.startPeriodic(0.02);
    }

    /**
     * The main loop of the Limelight odometry thread
     */
    private void loop() {
        if (swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble() > 720) {
            ignoreAllLimes = true;
        } else {
            ignoreAllLimes = false;
        }

        // updateFront();
        updateRear();
        updateLeft();
        // updateRight();
    }

    /**
     * Updates the odometry for the front limelight
     */
    private void updateFront() {
        LimelightHelpers.SetRobotOrientation(
            "limelight-f",
            swerve.getState().Pose.getRotation().getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        limeFront = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front");

        if (limeFront != null && limeFront.pose != null) {
            ignoreFrontLime = !poseInField(limeFront.pose) ||
                (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-front").getZ()) > 0.4) ||
                (LimelightHelpers.getTA("limelight-front") < 0.1) ||
                (limeFrontPrev != null && (limeFront.pose.getTranslation().getDistance(limeFrontPrev.pose.getTranslation()) /
                    (limeFront.timestampSeconds - limeFrontPrev.timestampSeconds)) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeFrontPrev != null && (limeFront.pose.getTranslation()
                    .getDistance(limeFrontPrev.pose.getTranslation()) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.02)) ||
                (limeFront.rawFiducials.length > 0 && limeFront.rawFiducials[0].ambiguity > 0.5 &&
                    limeFront.rawFiducials[0].distToCamera > 4.0) ||
                limeFront.pose.equals(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

            if (!ignoreAllLimes && !ignoreFrontLime) {
                SmartDashboard.putBoolean("Frontpose", true);
                frontNT.set(limeFront.pose);

                swerve.addVisionMeasurement(
                    limeFront.pose,
                    limeFront.timestampSeconds,
                    VecBuilder.fill(0.5, 0.5, 5).div(LimelightHelpers.getTA("limelight-front"))
                );
            } else {
                SmartDashboard.putBoolean("Frontpose", false);
            }

            limeFrontPrev = limeFront;
        }
    }

    /**
     * Updates the odometry for the rear limelight
     */
    private void updateRear() {
        LimelightHelpers.SetRobotOrientation(
            "limelight-rear",
            swerve.getState().Pose.getRotation().getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        limeRear = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-rear");

        if (limeRear != null && limeRear.pose != null) {
            ignoreRearLime = !poseInField(limeRear.pose) ||
                (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-rear").getZ()) > 0.4) ||
                (LimelightHelpers.getTA("limelight-rear") < 0.1) ||
                (limeRearPrev != null && (limeRear.pose.getTranslation().getDistance(limeRearPrev.pose.getTranslation()) /
                    (limeRear.timestampSeconds - limeRearPrev.timestampSeconds)) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeRearPrev != null && (limeRear.pose.getTranslation()
                    .getDistance(limeRearPrev.pose.getTranslation()) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.02)) ||
                (limeRear.rawFiducials.length > 0 && limeRear.rawFiducials[0].ambiguity > 0.5 &&
                    limeRear.rawFiducials[0].distToCamera > 4.0) ||
                limeRear.pose.equals(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

            if (!ignoreAllLimes && !ignoreRearLime) {
                SmartDashboard.putBoolean("Rearpose", true);
                rearNT.set(limeRear.pose);

                swerve.addVisionMeasurement(
                    limeRear.pose,
                    limeRear.timestampSeconds,
                    VecBuilder.fill(0.5, 0.5, 5).div(LimelightHelpers.getTA("limelight-rear"))
                );
            } else {
                SmartDashboard.putBoolean("Rearpose", false);
            }

            limeRearPrev = limeRear;
        }
    }

    /**
     * Updates the odometry for the left limelight
     */
    private void updateLeft() {
        LimelightHelpers.SetRobotOrientation(
            "limelight-left",
            swerve.getState().Pose.getRotation().getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        limeLeft = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-left");

        if (limeLeft != null && limeLeft.pose != null) {
            ignoreLeftLime = !poseInField(limeLeft.pose) ||
                (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-left").getZ()) > 0.4) ||
                (LimelightHelpers.getTA("limelight-left") < 0.1) ||
                (limeLeftPrev != null && (limeLeft.pose.getTranslation().getDistance(limeLeftPrev.pose.getTranslation()) /
                    (limeLeft.timestampSeconds - limeLeftPrev.timestampSeconds)) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeLeftPrev != null && (limeLeft.pose.getTranslation()
                    .getDistance(limeLeftPrev.pose.getTranslation()) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.02)) ||
                (limeLeft.rawFiducials.length > 0 && limeLeft.rawFiducials[0].ambiguity > 0.5 &&
                    limeLeft.rawFiducials[0].distToCamera > 4.0) ||
                limeLeft.pose.equals(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

            if (!ignoreAllLimes && !ignoreLeftLime) {
                SmartDashboard.putBoolean("Leftpose", true);
                leftNT.set(limeLeft.pose);

                swerve.addVisionMeasurement(
                    limeLeft.pose,
                    limeLeft.timestampSeconds,
                    VecBuilder.fill(0.75, 0.75, 5).div(LimelightHelpers.getTA("limelight-left"))
                );
            } else {
                SmartDashboard.putBoolean("Leftpose", false);
            }

            limeLeftPrev = limeLeft;
        }
    }

    /**
     * Updates the odometry for the right limelight
     */
    private void updateRight() {
        LimelightHelpers.SetRobotOrientation(
            "limelight-right",
            swerve.getState().Pose.getRotation().getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        limeRight = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-right");

        if (limeRight != null && limeRight.pose != null) {
            ignoreRightLime = !poseInField(limeRight.pose) ||
                (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-right").getZ()) > 0.4) ||
                (LimelightHelpers.getTA("limelight-right") < 0.1) ||
                (limeRightPrev != null && (limeRight.pose.getTranslation().getDistance(limeRightPrev.pose.getTranslation()) /
                    (limeRight.timestampSeconds - limeRightPrev.timestampSeconds)) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeRightPrev != null && (limeRight.pose.getTranslation()
                    .getDistance(limeRightPrev.pose.getTranslation()) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.02)) ||
                (limeRight.rawFiducials.length > 0 && limeRight.rawFiducials[0].ambiguity > 0.5 &&
                    limeRight.rawFiducials[0].distToCamera > 4.0) ||
                limeRight.pose.equals(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

            if (!ignoreAllLimes && !ignoreRightLime) {
                SmartDashboard.putBoolean("Rightpose", true);
                rightNT.set(limeRight.pose);

                swerve.addVisionMeasurement(
                    limeRight.pose,
                    limeRight.timestampSeconds,
                    VecBuilder.fill(0.75, 0.75, 5).div(LimelightHelpers.getTA("limelight-right"))
                );
            } else {
                SmartDashboard.putBoolean("Rightpose", false);
            }

            limeRightPrev = limeRight;
        }
    }

    /**
     * Checks if a pose is inside the field dimensions
     *
     * @param pose The {@link Pose2d} to check
     * @return True if the pose is inside the field dimensions, false otherwise
     */
    private boolean poseInField(Pose2d pose) {
        return pose.getTranslation().getX() > 0 &&
            pose.getTranslation().getX() < 16.54 &&
            pose.getTranslation().getY() > 0 &&
            pose.getTranslation().getY() < 8.07;
    }

    /**
     * Gets the {@link Limelight} subsystem instance
     *
     * @return The {@link Limelight} subsystem instance
     */
    public static Limelight system() {
        if (instance == null) {
            instance = new Limelight();
        }

        return instance;
    }
}