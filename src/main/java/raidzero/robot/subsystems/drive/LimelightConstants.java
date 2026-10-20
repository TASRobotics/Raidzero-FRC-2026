package raidzero.robot.subsystems.drive;

import edu.wpi.first.math.util.Units;
import java.util.List;
import raidzero.lib.LimelightState.IMU_MODE;

public final class LimelightConstants {
    /**
     * Which Limelight pose estimate to read
     */
    public enum Estimator {
        MT1, MT2,
    }

    /**
     * What a heading reset does, both at enable and on the seed buttons
     */
    public enum HeadingReset {
        /** Seeds the heading from the average MT1 heading at enable; the seed buttons run MT1, then seed from the average MT1 heading */
        MT1_AVERAGE,
        /** Does nothing at enable; the seed buttons run the disabled (MT1) config so its heading is fused */
        MT1_FUSE,
        /** Does nothing at enable; the seed buttons make the robot's current facing downfield */
        DRIVER_FORWARD,
    }

    /**
     * How one estimator's measurements are accepted and weighted
     *
     * @param estimator    Which pose estimate to read
     * @param minTags      Minimum number of tags in view to accept a measurement
     * @param xyStdDev     The standard deviation in the X and Y directions in meters
     * @param rotStdDev    The standard deviation for rotations in radians, or {@link #IGNORE_ROTATION} to never fuse the vision heading
     * @param scaleStdDevs Whether to divide the standard deviations by target area and inflate them when the hood is low
     * @param filter       Whether to run the outlier filters and ignore all measurements while spinning
     */
    public record EstimatorConfig(Estimator estimator, int minTags, double xyStdDev, double rotStdDev, boolean scaleStdDevs, boolean filter) {}

    /**
     * A vision tracking strategy
     *
     * @param name            The name shown in the dashboard chooser
     * @param disabled        The {@link EstimatorConfig} used while disabled and while a heading reset is held
     * @param enabled         The {@link EstimatorConfig} used while enabled
     * @param disabledImuMode The {@link IMU_MODE} used while disabled and while a heading reset is held
     * @param enabledImuMode  The {@link IMU_MODE} used while enabled, unless overridden from the dashboard
     * @param headingReset    The {@link HeadingReset} behavior
     */
    public record Profile(String name, EstimatorConfig disabled, EstimatorConfig enabled, IMU_MODE disabledImuMode, IMU_MODE enabledImuMode, HeadingReset headingReset) {}

    /** Whether accepted vision measurements update the swerve pose while disabled */
    public static final boolean UPDATE_SWERVE_POSE_DISABLED = true;

    /** Whether accepted vision measurements update the swerve pose while enabled; when false the pose is pure odometry while enabled */
    public static final boolean UPDATE_SWERVE_POSE_ENABLED = false;

    /** Minimum tags MT1 needs to be logged as a raw pose or used for a heading reset */
    public static final int MT1_MIN_TAGS = 2;

    /** MT1 standard deviation (meters and radians) while disabled; tiny so the pose snaps onto MT1 before enabling */
    public static final double DISABLED_MT1_STD_DEV = 0.01;

    /** Rotation standard deviation that effectively never fuses the vision heading */
    public static final double IGNORE_ROTATION = 9999999;

    /** Complementary filter alpha for IMU modes 3 and 4 (the Limelight default is 0.001) */
    public static final double IMU_ASSIST_ALPHA = 0.001;

    /** Every profile's disabled config: fully trusted MT1 pose and heading with 2+ tags, with the sanity filters on */
    public static final EstimatorConfig DISABLED_MT1 = new EstimatorConfig(
        Estimator.MT1, MT1_MIN_TAGS, DISABLED_MT1_STD_DEV, DISABLED_MT1_STD_DEV, false, true
    );

    /** Fully trusted MT1 (2+ tags) while disabled, MT2 + filters while enabled, heading seeded from the average MT1 heading */
    public static final Profile CURRENT = new Profile(
        "1: Current",
        DISABLED_MT1,
        new EstimatorConfig(Estimator.MT2, 0, 0.5, Units.degreesToRadians(10), true, true),
        IMU_MODE.EXTERNAL_SEED,
        IMU_MODE.INTERNAL_EXTERNAL_ASSIST,
        HeadingReset.MT1_AVERAGE
    );

    /** Fully trusted MT1 (2+ tags) while disabled, then Pigeon heading with unfiltered MT2 X/Y while enabled */
    public static final Profile PIGEON_MT2 = new Profile(
        "2: Pigeon + MT2",
        DISABLED_MT1,
        new EstimatorConfig(Estimator.MT2, 1, 0.5, IGNORE_ROTATION, true, false),
        IMU_MODE.EXTERNAL_SEED,
        IMU_MODE.EXTERNAL_ONLY,
        HeadingReset.DRIVER_FORWARD
    );

    /** Fully trusted MT1 (2+ tags) while disabled, unfiltered MT2 X/Y while enabled */
    public static final Profile MT1_MT2 = new Profile(
        "3: MT1 (2+ tags) + MT2",
        DISABLED_MT1,
        new EstimatorConfig(Estimator.MT2, 1, 0.5, IGNORE_ROTATION, true, false),
        IMU_MODE.EXTERNAL_SEED,
        IMU_MODE.EXTERNAL_ONLY,
        HeadingReset.MT1_FUSE
    );

    public static final List<Profile> PROFILES = List.of(CURRENT, PIGEON_MT2, MT1_MT2);

    public static final Profile DEFAULT_PROFILE = CURRENT;
}
