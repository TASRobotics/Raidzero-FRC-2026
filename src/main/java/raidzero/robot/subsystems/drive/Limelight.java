package raidzero.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
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
    private boolean ignoreBlLime = false;
    private boolean ignoreBrLime = false;
    private boolean ignoreAllLimes = false;

    private StructPublisher<Pose2d> flNT = NetworkTableInstance.getDefault().getStructTopic("flNT", Pose2d.struct).publish();
    private StructPublisher<Pose2d> rearNT = NetworkTableInstance.getDefault().getStructTopic("rearNT", Pose2d.struct).publish();
    private StructPublisher<Pose2d> blNT = NetworkTableInstance.getDefault().getStructTopic("blNT", Pose2d.struct).publish();
    private StructPublisher<Pose2d> brNT = NetworkTableInstance.getDefault().getStructTopic("brNT", Pose2d.struct).publish();

    private LimelightHelpers.PoseEstimate limeFront, limeRear, limeBL, limeBR;
    private LimelightHelpers.PoseEstimate limeFrontPrev, limeRearPrev, limeBLPrev, limeBRPrev;

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
        updateBackLeft();
        // updateBackRight();
    }

    /**
     * Updates the odometry for the front left limelight
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
                flNT.set(limeFront.pose);

                swerve.addVisionMeasurement(
                    limeFront.pose,
                    Utils.fpgaToCurrentTime(limeFront.timestampSeconds),
                    VecBuilder.fill(0.5, 0.5, 5).div(LimelightHelpers.getTA("limelight-front"))
                );
            } else {
                SmartDashboard.putBoolean("Frontpose", false);
            }

            limeFrontPrev = limeFront;
        }
    }

    /**
     * Updates the odometry for the front right limelight
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
                    Utils.fpgaToCurrentTime(limeRear.timestampSeconds),
                    VecBuilder.fill(0.5, 0.5, 5).div(LimelightHelpers.getTA("limelight-rear"))
                );
            } else {
                SmartDashboard.putBoolean("Rearpose", false);
            }

            limeRearPrev = limeRear;
        }
    }

    /**
     * Updates the odometry for the back left limelight
     */
    private void updateBackLeft() {
        LimelightHelpers.SetRobotOrientation(
            "limelight-bl",
            swerve.getState().Pose.getRotation().getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        limeBL = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-bl");

        if (limeBL != null && limeBL.pose != null) {
            ignoreBlLime = !poseInField(limeBL.pose) ||
                (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-bl").getZ()) > 0.4) ||
                (LimelightHelpers.getTA("limelight-bl") < 0.1) ||
                (limeBLPrev != null && (limeBL.pose.getTranslation().getDistance(limeBLPrev.pose.getTranslation()) /
                    (limeBL.timestampSeconds - limeBLPrev.timestampSeconds)) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeBLPrev != null && (limeBL.pose.getTranslation()
                    .getDistance(limeBLPrev.pose.getTranslation()) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.02)) ||
                (limeBL.rawFiducials.length > 0 && limeBL.rawFiducials[0].ambiguity > 0.5 &&
                    limeBL.rawFiducials[0].distToCamera > 4.0) ||
                limeBL.pose.equals(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

            if (!ignoreAllLimes && !ignoreBlLime) {
                SmartDashboard.putBoolean("BLpose", true);
                blNT.set(limeBL.pose);

                swerve.addVisionMeasurement(
                    limeBL.pose,
                    Utils.fpgaToCurrentTime(limeBL.timestampSeconds),
                    VecBuilder.fill(0.75, 0.75, 5).div(LimelightHelpers.getTA("limelight-bl"))
                );
            } else {
                SmartDashboard.putBoolean("BLpose", false);
            }

            limeBLPrev = limeBL;
        }
    }

    /**
     * Updates the odometry for the back right limelight
     */
    private void updateBackRight() {
        LimelightHelpers.SetRobotOrientation(
            "limelight-br",
            swerve.getState().Pose.getRotation().getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        limeBR = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-br");

        if (limeBR != null && limeBR.pose != null) {
            ignoreBrLime = !poseInField(limeBR.pose) ||
                (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-br").getZ()) > 0.4) ||
                (LimelightHelpers.getTA("limelight-br") < 0.1) ||
                (limeBRPrev != null && (limeBR.pose.getTranslation().getDistance(limeBRPrev.pose.getTranslation()) /
                    (limeBR.timestampSeconds - limeBRPrev.timestampSeconds)) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeBRPrev != null && (limeBR.pose.getTranslation()
                    .getDistance(limeBRPrev.pose.getTranslation()) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.02)) ||
                (limeBR.rawFiducials.length > 0 && limeBR.rawFiducials[0].ambiguity > 0.5 &&
                    limeBR.rawFiducials[0].distToCamera > 4.0) ||
                limeBR.pose.equals(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

            if (!ignoreAllLimes && !ignoreBrLime) {
                SmartDashboard.putBoolean("BRpose", true);
                brNT.set(limeBR.pose);

                swerve.addVisionMeasurement(
                    limeBR.pose,
                    Utils.fpgaToCurrentTime(limeBR.timestampSeconds),
                    VecBuilder.fill(0.75, 0.75, 5).div(LimelightHelpers.getTA("limelight-br"))
                );
            } else {
                SmartDashboard.putBoolean("BRpose", false);
            }

            limeBRPrev = limeBR;
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