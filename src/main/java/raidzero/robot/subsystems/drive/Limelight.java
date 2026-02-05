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
    public enum LED_MODE {
        PIPELINE, OFF, BLINK, ON
    }

    public enum STREAM_MODE {
        STANDARD, PIP_MAIN, PIP_SECOND
    }

    private boolean ignoreTurretLime = false;
    private boolean ignoreFrLime = false;
    private boolean ignoreAllLimes = false;

    private StructPublisher<Pose2d> turretNT = NetworkTableInstance.getDefault().getStructTopic("turretNT", Pose2d.struct).publish();
    private StructPublisher<Pose2d> frNT = NetworkTableInstance.getDefault().getStructTopic("frNT", Pose2d.struct).publish();

    private LimelightHelpers.PoseEstimate limeBL, limeFR;
    private LimelightHelpers.PoseEstimate limeBLPrev, limeFRPrev;

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
     * Sets the stream mode of the limelight
     *
     * @param limelightName The name of the limelight
     * @param mode {@link STREAM_MODE} of the limelight
     */
    public void setStreamMode(String limelightName, STREAM_MODE mode) {
        if (mode == STREAM_MODE.STANDARD) {
            LimelightHelpers.setStreamMode_Standard(limelightName);
        } else if (mode == STREAM_MODE.PIP_MAIN) {
            LimelightHelpers.setStreamMode_PiPMain(limelightName);
        } else if (mode == STREAM_MODE.PIP_SECOND) {
            LimelightHelpers.setStreamMode_PiPSecondary(limelightName);
        }
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
     * Sets the LED mode of the limelight
     *
     * @param limelightName The name of the limelight
     * @param mode The LED mode
     */
    public void setLedMode(String limelightName, LED_MODE mode) {
        if (mode == LED_MODE.PIPELINE) {
            LimelightHelpers.setLEDMode_PipelineControl(limelightName);
        } else if (mode == LED_MODE.OFF) {
            LimelightHelpers.setLEDMode_ForceOff(limelightName);
        } else if (mode == LED_MODE.BLINK) {
            LimelightHelpers.setLEDMode_ForceBlink(limelightName);
        } else if (mode == LED_MODE.ON) {
            LimelightHelpers.setLEDMode_ForceOn(limelightName);
        }
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

        updateBackLeft();
        updateFrontRight();
    }

    /**
     * Updates the odometry for the front left limelight
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
            ignoreTurretLime = !poseInField(limeBL.pose) ||
                (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-bl").getZ()) > 0.4) ||
                (LimelightHelpers.getTA("limelight-bl") < 0.1) ||
                (limeBLPrev != null && (limeBL.pose.getTranslation().getDistance(limeBLPrev.pose.getTranslation()) /
                    (limeBL.timestampSeconds - limeBLPrev.timestampSeconds)) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeBLPrev != null && (limeBL.pose.getTranslation()
                    .getDistance(limeBLPrev.pose.getTranslation()) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.02)) ||
                (limeBL.rawFiducials.length > 0 && limeBL.rawFiducials[0].ambiguity > 0.5 &&
                    limeBL.rawFiducials[0].distToCamera > 4.0) ||
                limeBL.pose.equals(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

            if (!ignoreAllLimes && !ignoreTurretLime) {
                SmartDashboard.putBoolean("BLPose", true);
                turretNT.set(limeBL.pose);

                swerve.addVisionMeasurement(
                    limeBL.pose,
                    limeBL.timestampSeconds,
                    VecBuilder.fill(0.5, 0.5, 5).div(LimelightHelpers.getTA("limelight-bl"))
                );
            } else {
                SmartDashboard.putBoolean("BLPose", false);
            }

            limeBLPrev = limeBL;
        }
    }

    /**
     * Updates the odometry for the front right limelight
     */
    private void updateFrontRight() {
        LimelightHelpers.SetRobotOrientation(
            "limelight-fr",
            swerve.getState().Pose.getRotation().getDegrees(),
            swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(),
            0,
            0,
            0,
            0
        );
        limeFR = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-fr");

        if (limeFR != null && limeFR.pose != null) {
            ignoreFrLime = !poseInField(limeFR.pose) ||
                (Math.abs(LimelightHelpers.getBotPose3d_wpiBlue("limelight-fr").getZ()) > 0.4) ||
                (LimelightHelpers.getTA("limelight-fr") < 0.1) ||
                (limeFRPrev != null && (limeFR.pose.getTranslation().getDistance(limeFRPrev.pose.getTranslation()) /
                    (limeFR.timestampSeconds - limeFRPrev.timestampSeconds)) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude()) ||
                (limeFRPrev != null && (limeFR.pose.getTranslation()
                    .getDistance(limeFRPrev.pose.getTranslation()) > TunerConstants.kSpeedAt12Volts.baseUnitMagnitude() * 0.02)) ||
                (limeFR.rawFiducials.length > 0 && limeFR.rawFiducials[0].ambiguity > 0.5 &&
                    limeFR.rawFiducials[0].distToCamera > 4.0) ||
                limeFR.pose.equals(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));

            if (!ignoreAllLimes && !ignoreFrLime) {
                SmartDashboard.putBoolean("FRpose", true);
                frNT.set(limeFR.pose);

                swerve.addVisionMeasurement(
                    limeFR.pose,
                    limeFR.timestampSeconds,
                    VecBuilder.fill(0.5, 0.5, 5).div(LimelightHelpers.getTA("limelight-fr"))
                );
            } else {
                SmartDashboard.putBoolean("FRpose", false);
            }

            limeFRPrev = limeFR;
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
            pose.getTranslation().getX() < 17.55 &&
            pose.getTranslation().getY() > 0 &&
            pose.getTranslation().getY() < 8.05;
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