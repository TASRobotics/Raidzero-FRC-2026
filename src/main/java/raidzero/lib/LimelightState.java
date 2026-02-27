package raidzero.lib;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.Supplier;
import raidzero.robot.subsystems.drive.TunerConstants;

public class LimelightState {
    public enum PIPELINE {
        TAG, OBJECT,
    }

    private String limelightName;
    private LimelightHelpers.PoseEstimate currEstimate;
    private LimelightHelpers.PoseEstimate prevEstimate;
    private StructPublisher<Pose2d> ntPublisher;
    private boolean ignore;
    private PIPELINE pipeline = PIPELINE.TAG;

    private final Supplier<Double> robotYawSupplier;
    private final Supplier<Double> robotYawRateSupplier;
    private final VisionMeasurementConsumer addVisionMeasurementConsumer;

    /**
     * Constructs a LimelightState instance
     *
     * @param limelightName The name of the Limelight
     * @param robotYawSupplier A {@link Supplier} that provides the robot's yaw
     * @param robotYawRateSupplier A {@link Supplier} that provides the robot's yaw rate
     * @param addVisionMeasurementConsumer A {@link VisionMeasurementConsumer} callback that adds vision measurements into the swerve pose estimator
     */
    public LimelightState(String limelightName, PIPELINE pipeline, Supplier<Double> robotYawSupplier, Supplier<Double> robotYawRateSupplier, VisionMeasurementConsumer addVisionMeasurementConsumer) {
        this.limelightName = limelightName;
        setPipeline(pipeline);
        this.ntPublisher = NetworkTableInstance.getDefault()
            .getStructTopic(limelightName.split("-")[1] + "NT", Pose2d.struct)
            .publish();

        this.ignore = false;
        this.robotYawSupplier = robotYawSupplier;
        this.robotYawRateSupplier = robotYawRateSupplier;
        this.addVisionMeasurementConsumer = addVisionMeasurementConsumer;
    }

    /**
     * Gets the current pose estimate
     *
     * @return The current {@link LimelightHelpers.PoseEstimate}
     */
    public LimelightHelpers.PoseEstimate getEstimate() {
        return currEstimate;
    }

    /**
     * Gets the name of the limelight
     *
     * @return The name of the limelight
     */
    public String getName() {
        return limelightName;
    }

    /**
     * Gets the current pipeline
     *
     * @return The current {@link PIPELINE}
     */
    public PIPELINE getPipeline() {
        return pipeline;
    }

    /**
     * Sets the current pipeline
     *
     * @param pipeline The {@link PIPELINE} to set
     */
    public void setPipeline(PIPELINE pipeline) {
        this.pipeline = pipeline;
        LimelightHelpers.setPipelineIndex(limelightName, pipeline.ordinal());
    }

    /**
     * Updates the limelight state
     *
     * @param ignoreAll Whether to ignore all measurements
     * @param useMt2    Whether to use the MetaTag2 pose estimation method
     */
    public void update(boolean ignoreAll, boolean useMt2) {
        update(ignoreAll, 0.5, 0.5, Units.degreesToRadians(5), useMt2);
    }

    /**
     * Updates the limelight state
     *
     * @param ignoreAll Whether to ignore all measurements
     * @param stdevX    The standard deviation in the X direction in meters
     * @param stdevY    The standard deviation in the Y direction in meters
     * @param useMt2    Whether to use the MetaTag2 pose estimation method
     */
    public void update(boolean ignoreAll, double stdevX, double stdevY, boolean useMt2) {
        update(ignoreAll, stdevX, stdevY, Units.degreesToRadians(5), useMt2);
    }

    /**
     * Updates the limelight state
     *
     * @param ignoreAll Whether to ignore all measurements
     * @param stdevRot  The standard deviation for rotations in radians
     * @param useMt2    Whether to use the MetaTag2 pose estimation method
     */
    public void update(boolean ignoreAll, double stdevRot, boolean useMt2) {
        update(ignoreAll, 0.5, 0.5, stdevRot, useMt2);
    }

    /**
     * Updates the limelight state
     *
     * @param ignoreAll Whether to ignore all measurements
     * @param stdevX    The standard deviation in the X direction in meters
     * @param stdevY    The standard deviation in the Y direction in meters
     * @param stdevRot  The standard deviation for rotations in radians
     * @param useMt2    Whether to use the MetaTag2 pose estimation method
     */
    public void update(boolean ignoreAll, double stdevX, double stdevY, double stdevRot, boolean useMt2) {
        if (pipeline != PIPELINE.TAG) {
            return;
        }

        LimelightHelpers.SetRobotOrientation(
            limelightName,
            robotYawSupplier.get(),
            robotYawRateSupplier.get(),
            0,
            0,
            0,
            0
        );

        currEstimate = useMt2 ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName) :
            LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        if (currEstimate != null && currEstimate.pose != null) {
            ignore = checkIgnore();

            if (!ignore && !ignoreAll) {
                ntPublisher.set(currEstimate.pose);

                addVisionMeasurementConsumer.add(
                    currEstimate.pose,
                    currEstimate.timestampSeconds,
                    VecBuilder.fill(stdevX, stdevY, stdevRot)
                        .div(Math.max(LimelightHelpers.getTA(limelightName), 0.05))
                );
            }

            prevEstimate = currEstimate;
        } else {
            ignore = false;
        }

        SmartDashboard.putBoolean(limelightName.split("-")[1] + "Pose", !ignore && !ignoreAll);
    }

    /**
     * Returns the yaw of the current pose estimate if not null
     *
     * @return A {@link Rotation2d} of the current pose estimate, or null if the estimate is null or being ignored
     */
    public Rotation2d getYaw() {
        return (ignore || currEstimate == null || currEstimate.pose == null) ? null : currEstimate.pose.getRotation();
    }

    /**
     * Checks if the current pose estimate should be ignored
     *
     * @return True if the pose estimate should be ignored, false otherwise
     */
    private boolean checkIgnore() {
        return !poseInField(currEstimate.pose) ||
            (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue(limelightName).getZ()) > 0.4) ||
            (LimelightHelpers.getTA(limelightName) < 0.1) ||
            (prevEstimate != null && (currEstimate.pose.getTranslation().getDistance(prevEstimate.pose.getTranslation()) /
                (currEstimate.timestampSeconds - prevEstimate.timestampSeconds)) > TunerConstants.kSpeedAt12Volts
                    .baseUnitMagnitude()) ||
            (prevEstimate != null && (currEstimate.pose.getTranslation()
                .getDistance(
                    prevEstimate.pose
                        .getTranslation()
                ) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.02)) ||
            (currEstimate.rawFiducials.length > 0 && currEstimate.rawFiducials[0].ambiguity > 0.5 &&
                currEstimate.rawFiducials[0].distToCamera > 4.0) ||
            currEstimate.pose.equals(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
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
}