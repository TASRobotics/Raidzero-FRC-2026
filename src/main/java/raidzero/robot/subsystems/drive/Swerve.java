package raidzero.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.Optional;
import java.util.function.Supplier;
import raidzero.lib.Util;

/**
 * Swerve subsystem with physical hardware disabled (virtual mode for bench testing).
 * Keeps pose tracking and vision measurements active without initializing unattached CAN devices.
 */
public class Swerve extends SubsystemBase {
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

    private static Swerve system;
    private final SwerveDriveState m_state = new SwerveDriveState();

    public Swerve() {
        m_state.Pose = new Pose2d();
        m_state.Speeds = new ChassisSpeeds();
        m_state.ModuleStates = new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState()
        };
        m_state.ModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()
        };
        m_state.ModuleTargets = new SwerveModuleState[] {
            new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState()
        };

        configureAutoBuilder();
    }

    public Swerve(com.ctre.phoenix6.swerve.SwerveDrivetrainConstants drivetrainConstants, com.ctre.phoenix6.swerve.SwerveModuleConstants<?, ?, ?>... modules) {
        this();
    }

    public Swerve(com.ctre.phoenix6.swerve.SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, com.ctre.phoenix6.swerve.SwerveModuleConstants<?, ?, ?>... modules) {
        this();
    }

    public Swerve(com.ctre.phoenix6.swerve.SwerveDrivetrainConstants drivetrainConstants, double odometryUpdateFrequency, Matrix<N3, N1> odometryStandardDeviation, Matrix<N3, N1> visionStandardDeviation, com.ctre.phoenix6.swerve.SwerveModuleConstants<?, ?, ?>... modules) {
        this();
    }

    public SwerveDriveState getState() {
        return m_state;
    }

    public void seedFieldCentric(Rotation2d heading) {
        m_state.Pose = new Pose2d(m_state.Pose.getTranslation(), heading);
    }

    public void resetPose(Pose2d pose) {
        m_state.Pose = pose;
    }

    public void setControl(SwerveRequest request) {}

    public void setOperatorPerspectiveForward(Rotation2d forward) {}

    public Pigeon2 getPigeon2() {
        return null;
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> {
        });
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Commands.none();
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return Commands.none();
    }

    @Override
    public void periodic() {
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        m_state.Pose = visionRobotPoseMeters;
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        m_state.Pose = visionRobotPoseMeters;
    }

    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {}

    public Optional<Pose2d> samplePoseAt(double timestampSeconds) {
        return Optional.of(m_state.Pose);
    }

    public Command stop() {
        return runOnce(() -> {
        });
    }

    public Command goToPoseAuto(Pose2d pose, LinearVelocity goalEndVelocity) {
        return defer(() -> {
            Pose2d targetPose = pose;

            if (Util.isRedAlliance()) {
                double fieldLength = 17.548;
                double fieldWidth = 8.052;

                targetPose = new Pose2d(
                    fieldLength - pose.getX(),
                    fieldWidth - pose.getY(),
                    pose.getRotation().plus(Rotation2d.kPi)
                );
            }

            return AutoBuilder.pathfindToPose(
                targetPose,
                new PathConstraints(
                    1,
                    1,
                    Units.degreesToRadians(540),
                    Units.degreesToRadians(720)
                ),
                goalEndVelocity
            );
        });
    }

    private void configureAutoBuilder() {
        try {
            AutoBuilder.configure(
                () -> this.getState().Pose,
                this::resetPose,
                () -> this.getState().Speeds,
                (speeds, feedforwards) -> {
                },
                new PPHolonomicDriveController(
                    new PIDConstants(2, 0, 0.1),
                    new PIDConstants(3.5, 0, 0.1)
                ),
                RobotConfig.fromGUISettings(),
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure Autobuilder", e.getStackTrace());
        }
    }

    public static Swerve system() {
        if (system == null) {
            system = new Swerve();
        }
        return system;
    }
}
