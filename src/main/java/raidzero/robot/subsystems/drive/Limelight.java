package raidzero.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;
import org.littletonrobotics.junction.Logger;
import raidzero.lib.ElasticLib;
import raidzero.lib.ElasticLib.Notification;
import raidzero.lib.ElasticLib.NotificationLevel;
import raidzero.lib.LimelightHelpers;
import raidzero.lib.LimelightState;
import raidzero.lib.Util;
import raidzero.robot.subsystems.drive.LimelightConstants.EstimatorConfig;
import raidzero.robot.subsystems.drive.LimelightConstants.HeadingReset;
import raidzero.robot.subsystems.drive.LimelightConstants.Profile;
import raidzero.robot.subsystems.turret.Hood;

public class Limelight {
    private boolean ignoreAllLimes = false;

    private LimelightState frontState;
    private LimelightState rightState;
    private LimelightState rearState;
    private LimelightState leftState;

    private Map<String, LimelightState> limelights;

    private Collection<LimelightState> limes;

    private ArrayList<Double> headingReadings;
    private boolean seededPigeon;
    private boolean manualSeed;

    private Timer seedInterval;
    private Timer seedTimer;

    private boolean headingResetActive = false;
    private LimelightState.IMU_MODE lastImuMode = null;

    private final SendableChooser<Profile> profileChooser = new SendableChooser<>();
    private final SendableChooser<Integer> imuModeChooser = new SendableChooser<>();

    private Swerve swerve = Swerve.system();
    private Hood hood = Hood.system();
    private static Limelight instance = null;

    /**
     * Constructs a {@link Limelight} subsytem instance
     */
    private Limelight() {
        headingReadings = new ArrayList<Double>();
        seededPigeon = false;
        manualSeed = false;

        frontState = new LimelightState(
            "limelight-front",
            LimelightState.PIPELINE.OBJECT,
            () -> swerve.getState().Pose.getRotation().getDegrees(),
            () -> swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            swerve::addVisionMeasurement
        );
        rightState = new LimelightState(
            "limelight-right",
            LimelightState.PIPELINE.TAG,
            () -> swerve.getState().Pose.getRotation().getDegrees(),
            () -> swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            swerve::addVisionMeasurement
        );
        rearState = new LimelightState(
            "limelight-rear",
            LimelightState.PIPELINE.TAG,
            () -> swerve.getState().Pose.getRotation().getDegrees(),
            () -> swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            swerve::addVisionMeasurement
        );
        leftState = new LimelightState(
            "limelight-left",
            LimelightState.PIPELINE.TAG,
            () -> swerve.getState().Pose.getRotation().getDegrees(),
            () -> swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            swerve::addVisionMeasurement
        );

        frontState.setHoodSupplier(() -> hood.getAngle().in(Degrees));
        rightState.setHoodSupplier(() -> hood.getAngle().in(Degrees));
        rearState.setHoodSupplier(() -> hood.getAngle().in(Degrees));
        leftState.setHoodSupplier(() -> hood.getAngle().in(Degrees));

        limelights = Map.of(
            "limelight-front", frontState,
            "limelight-right", rightState,
            "limelight-rear", rearState,
            "limelight-left", leftState
        );

        limes = limelights.values();

        for (LimelightState state : limes) {
            LimelightHelpers.SetIMUAssistAlpha(state.getName(), LimelightConstants.IMU_ASSIST_ALPHA);
        }

        for (Profile profile : LimelightConstants.PROFILES) {
            profileChooser.addOption(profile.name(), profile);
        }
        profileChooser.setDefaultOption(LimelightConstants.DEFAULT_PROFILE.name(), LimelightConstants.DEFAULT_PROFILE);

        imuModeChooser.setDefaultOption("Profile default", -1);
        for (LimelightState.IMU_MODE mode : LimelightState.IMU_MODE.values()) {
            imuModeChooser.addOption(mode.ordinal() + ": " + mode.name(), mode.ordinal());
        }

        SmartDashboard.putData("Limelight/Profile", profileChooser);
        SmartDashboard.putData("Limelight/MT2 IMU Mode", imuModeChooser);

        seedInterval = new Timer();
        seedInterval.start();
        seedTimer = new Timer();
        seedTimer.start();
    }

    /**
     * Sets the IMU mode for all limelights
     *
     * @implNote This is only supported for Limelight 4
     *
     * @param imuMode The {@link LimelightState.IMU_MODE} to set for all limelights
     */
    public void setImuMode(LimelightState.IMU_MODE imuMode) {
        for (LimelightState state : limes) {
            state.setIMUMode(imuMode);
        }
    }

    /**
     * Toggles whether to ignore all limelight pose estimates
     */
    public void toggleLimelights() {
        ignoreAllLimes = !ignoreAllLimes;
    }

    /**
     * Seeds the heading at enable if the selected profile calls for it
     */
    public void onEnable() {
        if (profileChooser.getSelected().headingReset() == HeadingReset.MT1_AVERAGE) {
            seedFromAverageHeading();
        }
    }

    /**
     * Starts a heading reset, which runs the disabled config until {@link #finishHeadingReset()} is called
     */
    public void beginHeadingReset() {
        headingResetActive = profileChooser.getSelected().headingReset() != HeadingReset.DRIVER_FORWARD;
    }

    /**
     * Finishes a heading reset according to the selected profile's {@link HeadingReset}
     */
    public void finishHeadingReset() {
        headingResetActive = false;

        switch (profileChooser.getSelected().headingReset()) {
            case MT1_AVERAGE -> seedFromAverageHeading();
            case DRIVER_FORWARD -> swerve.seedFieldCentric();
            case MT1_FUSE -> {
            }
        }
    }

    /**
     * Seeds the swerve heading from the average MT1 heading of all limelights
     */
    private void seedFromAverageHeading() {
        swerve.seedFieldCentric(
            Rotation2d.fromDegrees(getAverageHeading())
                .plus(Util.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg)
        );
    }

    /**
     * Sets the pipeline of the limelight
     *
     * @param limelightName The name of the limelight
     * @param pipeline The Limelight {@link LimelightState.PIPELINE}
     */
    public void setPipeline(String limelightName, LimelightState.PIPELINE pipeline) {
        if (!limelights.containsKey(limelightName)) {
            return;
        }

        limelights.get(limelightName).setPipeline(pipeline);
    }

    public double getAverageHeading() {
        return limes.stream()
            .map(LimelightState::getYaw)
            .filter(java.util.Objects::nonNull)
            .map(Rotation2d::getDegrees)
            .filter(z -> !Double.isNaN(z))
            .mapToDouble(Double::doubleValue)
            .average()
            .orElse(Double.NaN);
    }

    /**
     * Gets the average Z height of all limelight pose estimates that are not null or being ignored
     *
     * @return The average Z height of all limelight pose estimates that are not null or being ignored, or {@link Double#NaN} if there are no valid estimates
     */
    public double getAverageZ() {
        return limes.stream()
            .map(LimelightState::getZ)
            .filter(z -> !Double.isNaN(z))
            .mapToDouble(Double::doubleValue)
            .average()
            .orElse(Double.NaN);
    }

    /**
     * Manually resets the seeding process
     */
    public void resetSeed() {
        ElasticLib.sendNotification(new Notification(NotificationLevel.WARNING, "LL", "RESET CALLED"));

        // seedInterval.stop();
        // seedTimer.stop();
        // seedInterval.reset();
        // seedTimer.reset();

        // seededPigeon = false;
        // manualSeed = false;
        // headingReadings.clear();

        // seedInterval.start();
        // seedTimer.start();

        // for (LimelightState state : limes) {
        // state.setSeededPose(false);
        // }
    }

    /**
     * Sets whether to manually seed the pigeon
     *
     * @param manualSeed Whether to manually seed the pigeon
     */
    public void setManualSeed(boolean manualSeed) {
        this.manualSeed = manualSeed;
    }

    /**
     * The main loop of the Limelight odometry thread
     */
    public void loop() {
        Profile profile = profileChooser.getSelected();
        boolean useDisabledConfig = DriverStation.isDisabled() || headingResetActive;
        EstimatorConfig config = useDisabledConfig ? profile.disabled() : profile.enabled();

        int imuModeOverride = imuModeChooser.getSelected();
        LimelightState.IMU_MODE imuMode = useDisabledConfig ? profile.disabledImuMode() :
            imuModeOverride >= 0 ? LimelightState.IMU_MODE.values()[imuModeOverride] : profile.enabledImuMode();

        if (imuMode != lastImuMode) {
            setImuMode(imuMode);
            lastImuMode = imuMode;
        }

        ignoreAllLimes = config.filter() && Math.abs(swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720;

        for (LimelightState state : limes) {
            state.update(config, ignoreAllLimes, swerve.getState().Pose);
        }

        // if (!seededPigeon && seedTimer.get() < 15 && seedInterval.hasElapsed(0.25)) {
        // Rotation2d rotation;

        // for (LimelightState state : limes) {
        // rotation = state.getYaw();

        // if (rotation != null) {
        // headingReadings.add(rotation.getDegrees());
        // }
        // }

        // seedInterval.restart();
        // }

        // if (!seededPigeon && (manualSeed || seedTimer.get() > 15)) {
        // swerve.getPigeon2().setYaw(headingReadings.stream().mapToDouble(Double::doubleValue).average().orElse(0.0));
        // seededPigeon = true;
        // manualSeed = false;
        // seedInterval.stop();
        // seedInterval.reset();
        // seedTimer.stop();
        // for (LimelightState state : limes) {
        // state.setSeededPose(true);
        // }
        // }

        // SmartDashboard.putBoolean("SeededRobot", seededPigeon);
        // SmartDashboard.putBoolean("ManualSeed", manualSeed);
        // SmartDashboard.putNumber("HeadingReadingsSize", headingReadings.size());
        // SmartDashboard.putNumber("seedinterval", seedInterval.get());
        // SmartDashboard.putNumber("seedtimer", seedTimer.get());
        SmartDashboard.putBoolean("Ignore Limes", ignoreAllLimes);
        SmartDashboard.putString("Limelight/Active Profile", profile.name());
        SmartDashboard.putString("Limelight/Estimator", config.estimator().name());
        SmartDashboard.putString("Limelight/IMU Mode", imuMode.ordinal() + ": " + imuMode.name());

        Logger.recordOutput("Limelight/Profile", profile.name());
        Logger.recordOutput("Limelight/Estimator", config.estimator());
        Logger.recordOutput("Limelight/IMUMode", imuMode);
        Logger.recordOutput("Limelight/HeadingResetActive", headingResetActive);
        Logger.recordOutput("Limelight/IgnoreAll", ignoreAllLimes);
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