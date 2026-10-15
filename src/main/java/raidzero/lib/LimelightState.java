package raidzero.lib;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Arrays;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import raidzero.robot.subsystems.drive.LimelightConstants.Estimator;
import raidzero.robot.subsystems.drive.LimelightConstants.EstimatorConfig;
import raidzero.robot.subsystems.drive.TunerConstants;

public class LimelightState {
    public enum PIPELINE {
        TAG, OBJECT,
    }

    public enum IMU_MODE {
        EXTERNAL_ONLY, EXTERNAL_SEED, INTERNAL_ONLY, INTERNAL_MT1_ASSIST, INTERNAL_EXTERNAL_ASSIST,
    }

    private String limelightName;
    private LimelightHelpers.IMUData imuData;
    private LimelightHelpers.PoseEstimate currEstimate;
    private LimelightHelpers.PoseEstimate prevEstimate;
    private StructPublisher<Pose2d> ntPublisher;
    private boolean ignore;
    private PIPELINE pipeline = PIPELINE.TAG;
    private IMU_MODE imuMode = IMU_MODE.EXTERNAL_ONLY;

    private final Supplier<Double> robotYawSupplier;
    private final Supplier<Double> robotYawRateSupplier;
    private final VisionMeasurementConsumer addVisionMeasurementConsumer;
    private Supplier<Double> hoodAngleSupplier;

    private double lastValidPose = 0.0;

    private static final double RECOVERY_FACTOR = 1.25;

    private static final Pose2d[] NO_POSES = new Pose2d[0];

    private final String logKey;

    private boolean seededRobot = false;

    /**
     * Constructs a LimelightState instance
     *
     * @param limelightName The name of the Limelight
     * @param pipeline The initial pipeline to set for the Limelight
     * @param robotYawSupplier A {@link Supplier} that provides the robot's yaw
     * @param robotYawRateSupplier A {@link Supplier} that provides the robot's yaw rate
     * @param addVisionMeasurementConsumer A {@link VisionMeasurementConsumer} callback that adds vision measurements into the swerve pose estimator
     */
    public LimelightState(String limelightName, PIPELINE pipeline, Supplier<Double> robotYawSupplier, Supplier<Double> robotYawRateSupplier, VisionMeasurementConsumer addVisionMeasurementConsumer) {
        this.limelightName = limelightName;
        this.logKey = "Limelight/" + limelightName.split("-")[1] + "/";
        setPipeline(pipeline);
        this.ntPublisher = NetworkTableInstance.getDefault()
            .getStructTopic(limelightName.split("-")[1] + "NT", Pose2d.struct)
            .publish();

        this.ignore = false;
        this.robotYawSupplier = robotYawSupplier;
        this.robotYawRateSupplier = robotYawRateSupplier;
        this.addVisionMeasurementConsumer = addVisionMeasurementConsumer;
        this.imuMode = IMU_MODE.EXTERNAL_SEED;
        this.setIMUMode(imuMode);
    }

    /**
     * Sets the hood angle supplier for adjusting the vision measurement standard deviations
     *
     * @param hoodAngleSupplier A {@link Supplier} that provides the current hood angle in degrees as a double
     */
    public void setHoodSupplier(Supplier<Double> hoodAngleSupplier) {
        this.hoodAngleSupplier = hoodAngleSupplier;
    }

    /**
     * Sets whether the robot is seeded
     *
     * @param seeded Whether the robot is seeded
     */
    public void setSeededPose(boolean seeded) {
        this.seededRobot = seeded;
    }

    /**
     * Sets the IMU mode for the Limelight
     *
     * @implNote This is only supported for Limelight 4
     *
     * @param imuMode The {@link IMU_MODE} to set
     */
    public void setIMUMode(IMU_MODE imuMode) {
        this.imuMode = imuMode;
        LimelightHelpers.SetIMUMode(limelightName, this.imuMode.ordinal());
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
     * @param config    The {@link EstimatorConfig} that picks the pose estimate and how it is accepted and weighted
     * @param ignoreAll Whether to ignore all measurements
     * @param robotPose The current robot pose
     */
    public void update(EstimatorConfig config, boolean ignoreAll, Pose2d robotPose) {
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

        LimelightHelpers.PoseEstimate mt1Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        LimelightHelpers.PoseEstimate mt2Estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        currEstimate = config.estimator() == Estimator.MT2 ? mt2Estimate : mt1Estimate;

        imuData = LimelightHelpers.getIMUData(limelightName);

        Pose2d[] acceptedPose = NO_POSES;
        Pose2d[] rejectedPose = NO_POSES;
        double[] acceptedStdDevs = new double[0];

        if (currEstimate != null && currEstimate.pose != null) {
            ignore = currEstimate.tagCount < config.minTags() || (config.filter() && checkIgnore(robotPose));

            if (!ignore && !ignoreAll) {
                lastValidPose = Timer.getTimestamp();

                ntPublisher.set(currEstimate.pose);

                Matrix<N3, N1> stdDevs = VecBuilder.fill(config.xyStdDev(), config.xyStdDev(), config.rotStdDev());

                if (config.scaleStdDevs()) {
                    stdDevs = stdDevs
                        .div(Math.max(LimelightHelpers.getTA(limelightName), 0.05))
                        .times(hoodAngleSupplier != null && hoodAngleSupplier.get() > 20 ? 1.0 : 1.75);
                }

                addVisionMeasurementConsumer.add(
                    currEstimate.pose,
                    currEstimate.timestampSeconds,
                    stdDevs
                );

                acceptedPose = new Pose2d[] { currEstimate.pose };
                acceptedStdDevs = stdDevs.getData();
            } else if (currEstimate.tagCount > 0) {
                rejectedPose = new Pose2d[] { currEstimate.pose };
            }

            prevEstimate = currEstimate;
        } else {
            ignore = false;
        }

        if (imuData != null) {
            SmartDashboard.putNumber(
                limelightName.split("-")[1] + "RYaw",
                imuData.robotYaw
            );
        }

        SmartDashboard.putBoolean(limelightName.split("-")[1] + "Pose", !ignore && !ignoreAll);

        Logger.recordOutput(logKey + "MT1Pose", posesWithTags(mt1Estimate));
        Logger.recordOutput(logKey + "MT2Pose", posesWithTags(mt2Estimate));
        Logger.recordOutput(logKey + "AcceptedPose", acceptedPose);
        Logger.recordOutput(logKey + "RejectedPose", rejectedPose);
        Logger.recordOutput(logKey + "AcceptedStdDevs", acceptedStdDevs);
        Logger.recordOutput(logKey + "TagCount", currEstimate != null ? currEstimate.tagCount : 0);
        Logger.recordOutput(logKey + "AvgTagDist", currEstimate != null ? currEstimate.avgTagDist : 0.0);
        Logger.recordOutput(
            logKey + "TagIds",
            currEstimate != null && currEstimate.rawFiducials != null ?
                Arrays.stream(currEstimate.rawFiducials).mapToInt(fiducial -> fiducial.id).toArray() :
                new int[0]
        );
    }

    /**
     * Wraps an estimate's pose in an array so it can be logged and disappears from the field view when no tags are seen
     *
     * @param estimate The {@link LimelightHelpers.PoseEstimate} to wrap
     * @return An array holding the estimate's pose, or an empty array if the estimate is null or sees no tags
     */
    private static Pose2d[] posesWithTags(LimelightHelpers.PoseEstimate estimate) {
        return estimate != null && estimate.pose != null && estimate.tagCount > 0 ? new Pose2d[] { estimate.pose } : NO_POSES;
    }

    /**
     * Returns the yaw of the current pose estimate if not null
     *
     * @return A {@link Rotation2d} of the current pose estimate, or null if the estimate is null or being ignored
     */
    public Rotation2d getYaw() {
        LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        return (ignore || estimate == null || currEstimate == null || currEstimate.pose == null) ? null : estimate.pose.getRotation();
    }

    /**
     * Returns the Z height of the current pose estimate if not null or not being ignored
     *
     * @return A {@link Double} of the Z height of the current pose estimate, or {@link Double#NaN} if the estimate is null or being ignored
     */
    public double getZ() {
        Pose3d pose = LimelightHelpers.getBotPose3d_wpiBlue(limelightName);

        return (ignore || pose == null || currEstimate == null || currEstimate.pose == null) ? Double.NaN : pose.getZ();
    }

    private boolean checkIgnore(Pose2d robotPose) {
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
            (seededRobot && currEstimate != null && (currEstimate.pose.getTranslation()
                .getDistance(robotPose.getTranslation()) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.2) &&
                (lastValidPose <= 0.0 ||
                    currEstimate.pose.getTranslation()
                        .getDistance(robotPose.getTranslation()) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() *
                            (Timer.getTimestamp() - lastValidPose) * RECOVERY_FACTOR)) ||
            (currEstimate.rawFiducials.length > 0 && currEstimate.rawFiducials[0].ambiguity > 0.5 &&
                currEstimate.rawFiducials[0].distToCamera > 5.5) ||
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