package raidzero.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;
import raidzero.lib.ElasticLib;
import raidzero.lib.ElasticLib.Notification;
import raidzero.lib.ElasticLib.NotificationLevel;
import raidzero.lib.LimelightState;
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

    private boolean useMt2 = true;

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
            LimelightState.PIPELINE.TAG,
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

    public void setUseMt2(boolean useMt2) {
        this.useMt2 = useMt2;
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
        if (Math.abs(swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) {
            ignoreAllLimes = true;
        } else {
            ignoreAllLimes = false;
        }

        for (LimelightState state : limes) {
            state.update(ignoreAllLimes, !DriverStation.isDisabled() && useMt2, swerve.getState().Pose);
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