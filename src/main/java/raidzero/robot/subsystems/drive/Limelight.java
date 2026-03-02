package raidzero.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Map;
import raidzero.lib.LimelightState;
import raidzero.robot.subsystems.turret.Hood;
import raidzero.robot.subsystems.turret.HoodIO;

public class Limelight extends SubsystemBase {
    private boolean ignoreAllLimes = false;

    private LimelightState frontState;
    private LimelightState rightState;
    private LimelightState rearState;
    private LimelightState leftState;

    private Map<String, LimelightState> limelights;

    private Collection<LimelightState> limes;

    private ArrayList<Double> headingReadings;
    private boolean seededPigeon;

    private Notifier notifier;
    private Timer seedInterval;
    private Timer seedTimer;

    private Swerve swerve = Swerve.system();
    private Hood hood = Hood.system();
    private static Limelight instance = null;

    /**
     * Constructs a {@link Limelight} subsytem instance
     */
    private Limelight() {
        headingReadings = new ArrayList<Double>();
        seededPigeon = false;

        frontState = new LimelightState(
            "limelight-front",
            LimelightState.PIPELINE.TAG,
            () -> swerve.getState().Pose,
            () -> swerve.getPigeon2().getRotation2d().getDegrees(),
            () -> swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            swerve::addVisionMeasurement
        );
        rightState = new LimelightState(
            "limelight-right",
            LimelightState.PIPELINE.TAG,
            () -> swerve.getState().Pose,
            () -> swerve.getPigeon2().getRotation2d().getDegrees(),
            () -> swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            swerve::addVisionMeasurement
        );
        rearState = new LimelightState(
            "limelight-rear",
            LimelightState.PIPELINE.TAG,
            () -> swerve.getState().Pose,
            () -> swerve.getPigeon2().getRotation2d().getDegrees(),
            () -> swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            swerve::addVisionMeasurement
        );
        leftState = new LimelightState(
            "limelight-left",
            LimelightState.PIPELINE.TAG,
            () -> swerve.getState().Pose,
            () -> swerve.getPigeon2().getRotation2d().getDegrees(),
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

        this.startThread();
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

    /**
     * Starts the Limelight odometry thread
     */
    private void startThread() {
        notifier = new Notifier(this::loop);
        notifier.startPeriodic(0.02);
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
     * The main loop of the Limelight odometry thread
     */
    private void loop() {
        if (Math.abs(swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 720) {
            ignoreAllLimes = true;
        } else {
            ignoreAllLimes = false;
        }

        for (LimelightState state : limes) {
            state.update(ignoreAllLimes, seededPigeon);
        }

        if (!seededPigeon && seedTimer.get() < 30 && seedInterval.hasElapsed(1)) {
            Rotation2d rotation;

            for (LimelightState state : limes) {
                rotation = state.getYaw();

                if (rotation != null) {
                    headingReadings.add(rotation.getDegrees());
                }
            }

            seedInterval.restart();
        }

        if (!seededPigeon && seedTimer.get() > 30 && !headingReadings.isEmpty()) {
            swerve.getPigeon2().setYaw(headingReadings.stream().mapToDouble(Double::doubleValue).average().orElse(0.0));
            seededPigeon = true;
            seedInterval.stop();
            seedInterval.reset();
            seedTimer.stop();
            for (LimelightState state : limes) {
                state.setSeededPose(true);
            }
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("SeededRobot", seededPigeon);
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