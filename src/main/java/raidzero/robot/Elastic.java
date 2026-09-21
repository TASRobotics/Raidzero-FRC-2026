package raidzero.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Map;
import raidzero.lib.ElasticLib;
import raidzero.lib.LimelightState;
import raidzero.lib.Util;
import raidzero.robot.subsystems.drive.Limelight;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.intake.Intake;
import raidzero.robot.subsystems.intake.Spindexer;
import raidzero.robot.subsystems.shooter.Feeder;
import raidzero.robot.subsystems.shooter.Shooter;
import raidzero.robot.subsystems.turret.Turret;

/**
 * Central manager for all telemetry and logging on Elastic / WPILib SmartDashboard.
 */
public class Elastic {
    private static boolean isBlueAlliance = true;
    private static boolean initialized = false;

    // Field visualization
    private static final Field2d field = new Field2d();

    // Bot pose publishers
    private static StructPublisher<Pose2d> botPosePub;
    private static StructPublisher<Pose2d> botPoseNTPub;
    private static StructPublisher<Pose2d> sdBotPosePub;

    // Limelight pose publishers
    private static StructPublisher<Pose2d> frontPosePub;
    private static StructPublisher<Pose2d> rightPosePub;
    private static StructPublisher<Pose2d> rearPosePub;
    private static StructPublisher<Pose2d> leftPosePub;

    private static StructPublisher<Pose2d> frontNTPub;
    private static StructPublisher<Pose2d> rightNTPub;
    private static StructPublisher<Pose2d> rearNTPub;
    private static StructPublisher<Pose2d> leftNTPub;

    // Turret pose publisher
    private static StructPublisher<Pose2d> turretPosePub;

    /**
     * Initializes Elastic tabs and logging publishers.
     */
    public static void init() {
        ElasticLib.selectTab("Setup");
        initLogging();
    }

    /**
     * Sets up CameraPublisher streams, Field2d, and NetworkTable topics for Elastic.
     */
    public static void initLogging() {
        if (initialized) {
            return;
        }
        initialized = true;

        NetworkTableInstance nt = NetworkTableInstance.getDefault();

        // Publish Field2d widget
        SmartDashboard.putData("Field", field);

        // Bot pose publishers
        botPosePub = nt.getStructTopic("botpose", Pose2d.struct).publish();
        botPoseNTPub = nt.getStructTopic("botPoseNT", Pose2d.struct).publish();
        sdBotPosePub = nt.getTable("SmartDashboard").getStructTopic("botPose", Pose2d.struct).publish();

        // Limelight pose publishers
        frontPosePub = nt.getTable("SmartDashboard/Limelights/front").getStructTopic("Pose", Pose2d.struct).publish();
        rightPosePub = nt.getTable("SmartDashboard/Limelights/right").getStructTopic("Pose", Pose2d.struct).publish();
        rearPosePub = nt.getTable("SmartDashboard/Limelights/rear").getStructTopic("Pose", Pose2d.struct).publish();
        leftPosePub = nt.getTable("SmartDashboard/Limelights/left").getStructTopic("Pose", Pose2d.struct).publish();

        frontNTPub = nt.getStructTopic("frontNT", Pose2d.struct).publish();
        rightNTPub = nt.getStructTopic("rightNT", Pose2d.struct).publish();
        rearNTPub = nt.getStructTopic("rearNT", Pose2d.struct).publish();
        leftNTPub = nt.getStructTopic("leftNT", Pose2d.struct).publish();

        // Turret pose publisher
        turretPosePub = nt.getTable("SmartDashboard/Turret").getStructTopic("Pose", Pose2d.struct).publish();

        // Setup camera publishers for all 4 Limelights
        setupCameraStream("limelight-front", "10.42.53.12");
        setupCameraStream("limelight-right", "10.42.53.13");
        setupCameraStream("limelight-rear", "10.42.53.14");
        setupCameraStream("limelight-left", "10.42.53.11");
    }

    private static void setupCameraStream(String cameraName, String ip) {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        NetworkTable cameraTable = nt.getTable("CameraPublisher/" + cameraName);

        String mjpgUrl = "mjpg:http://" + ip + ":5800/stream.mjpg";
        String localUrl = "mjpg:http://" + cameraName + ".local:5800/stream.mjpg";
        String httpUrl = "http://" + ip + ":5800/stream.mjpg";

        cameraTable.getStringArrayTopic("streams").publish().set(new String[] { mjpgUrl, localUrl });
        cameraTable.getStringTopic("description").publish().set(cameraName + " Stream");
        cameraTable.getBooleanTopic("connected").publish().set(true);

        // SmartDashboard direct stream URLs for Elastic camera widget
        SmartDashboard.putString("CameraStreams/" + cameraName, httpUrl);
    }

    /**
     * Updates all Elastic telemetry: botpose, 4 limelight poses, turret angle, and component spinning flags.
     * Call periodically from robotPeriodic().
     */
    public static void update() {
        if (!initialized) {
            initLogging();
        }

        updateBotPose();
        updateLimelights();
        updateTurret();
        updateComponentSpinning();
    }

    private static void updateBotPose() {
        try {
            Pose2d pose = Swerve.system().getState().Pose;
            if (pose != null) {
                field.setRobotPose(pose);
                botPosePub.set(pose);
                botPoseNTPub.set(pose);
                sdBotPosePub.set(pose);
            }
        } catch (Exception expected) {
            // Subsystem not ready yet
        }
    }

    private static void updateLimelights() {
        try {
            Limelight limelight = Limelight.system();
            updateSingleLimelight("front", limelight.getFront(), frontPosePub, frontNTPub);
            updateSingleLimelight("right", limelight.getRight(), rightPosePub, rightNTPub);
            updateSingleLimelight("rear", limelight.getRear(), rearPosePub, rearNTPub);
            updateSingleLimelight("left", limelight.getLeft(), leftPosePub, leftNTPub);
        } catch (Exception expected) {
            // Subsystem not ready yet
        }
    }

    private static void updateSingleLimelight(String name, LimelightState state,
                                             StructPublisher<Pose2d> posePub,
                                             StructPublisher<Pose2d> ntPub) {
        if (state == null) {
            return;
        }

        Pose2d pose = state.getPose();
        boolean valid = state.hasValidPose();

        SmartDashboard.putBoolean("Limelights/" + name + "/Valid", valid);
        SmartDashboard.putBoolean(name + "Pose", valid);

        if (pose != null) {
            posePub.set(pose);
            ntPub.set(pose);
            if (valid) {
                field.getObject("Limelight-" + name).setPose(pose);
            }
        }
    }

    private static void updateTurret() {
        try {
            Turret turret = Turret.system();
            Angle yaw = turret.getCurrentYaw();

            SmartDashboard.putNumber("Turret/AngleDeg", yaw.in(Degrees));
            SmartDashboard.putNumber("Turret/AngleRot", yaw.in(Rotations));

            Pose2d turretPose = turret.getTurretPose();
            if (turretPose != null) {
                turretPosePub.set(turretPose);
                field.getObject("Turret").setPose(turretPose);
            }
        } catch (Exception expected) {
            // Subsystem not ready yet
        }
    }

    private static void updateComponentSpinning() {
        try {
            // Shooter flywheel
            boolean shooterSpinning = Math.abs(Shooter.system().getVelocity().in(RotationsPerSecond)) > 2.0;
            SmartDashboard.putBoolean("Spinning/Shooter", shooterSpinning);
            SmartDashboard.putBoolean("Shooter Spinning", shooterSpinning);

            // Feeder
            boolean feederSpinning = Math.abs(Feeder.system().getVelocity().in(RotationsPerSecond)) > 2.0;
            SmartDashboard.putBoolean("Spinning/Feeder", feederSpinning);
            SmartDashboard.putBoolean("Feeder Spinning", feederSpinning);

            // Spindexer
            boolean spindexerSpinning = Math.abs(Spindexer.system().getVelocity().in(RotationsPerSecond)) > 0.5;
            SmartDashboard.putBoolean("Spinning/Spindexer", spindexerSpinning);
            SmartDashboard.putBoolean("Spindexer Spinning", spindexerSpinning);

            // Turret yaw
            boolean turretSpinning = Math.abs(Turret.system().getVelocity().in(RotationsPerSecond)) > 0.05;
            SmartDashboard.putBoolean("Spinning/Turret", turretSpinning);
            SmartDashboard.putBoolean("Turret Spinning", turretSpinning);

            // Intake
            boolean intakeSpinning = Math.abs(Intake.system().getVelocity().in(RotationsPerSecond)) > 1.0;
            SmartDashboard.putBoolean("Spinning/Intake", intakeSpinning);
            SmartDashboard.putBoolean("Intake Spinning", intakeSpinning);

            // Swerve drive / chassis
            ChassisSpeeds speeds = Swerve.system().getState().Speeds;
            boolean swerveSpinning = speeds != null && (
                Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > 0.05 ||
                Math.abs(speeds.omegaRadiansPerSecond) > 0.05
            );
            SmartDashboard.putBoolean("Spinning/Swerve", swerveSpinning);
            SmartDashboard.putBoolean("Swerve Spinning", swerveSpinning);
        } catch (Exception expected) {
            // Subsystem not ready yet
        }
    }

    /**
     * Gets the Field2d instance used for telemetry.
     *
     * @return the Field2d instance
     */
    public static Field2d getField() {
        return field;
    }

    /**
     * Selects the auton tab in Elastic
     */
    public static void initAuton() {
        ElasticLib.selectTab("Auton");
    }

    /**
     * Selects the teleop tab in Elastic based on alliance color
     */
    public static void initTeleop() {
        if (isBlueAlliance) {
            ElasticLib.selectTab("BTeleop");
        } else {
            ElasticLib.selectTab("RTeleop");
        }
    }

    /**
     * Checks the alliance color and sets the isBlueAlliance variable accordingly
     */
    public static void checkForAlliance() {
        if (!DriverStation.isDSAttached()) {
            return;
        }

        if (Util.isBlueAlliance()) {
            isBlueAlliance = true;
        } else {
            isBlueAlliance = false;
        }
    }
}
