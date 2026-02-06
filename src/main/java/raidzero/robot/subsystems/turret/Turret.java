package raidzero.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.R0Subsystem;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.turret.TurretConstants.Pitch;

import static raidzero.robot.subsystems.turret.TurretConstants.*;

public class Turret extends R0Subsystem<TurretIO> {
    private static Turret system;
    private static Swerve swerve = Swerve.system();

    private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
    private NetworkTable staticTable = null;
    private NetworkTable kineticTable = null;
    private StructPublisher<Pose2d> staticGoalSender = null;
    private StructPublisher<Pose2d> staticTurretSender = null;
    private StructPublisher<Pose2d> kineticGoalSender = null;
    private StructPublisher<Pose2d> kineticTurretSender = null;

    private Translation2d allianceTargetFieldPose = null;

    private AimBot staticAimBot = null;
    private AimBot kineticAimBot = null;

    public Command moveSimple(double percent) {
        return run(() -> io.runYawSimple(percent));
    }

    public Angle getYawFromTarget() {
        return io.getYawFromTarget();
    }

    public Angle getCurrentYaw() {
        return io.getYaw();
    }

    /**
     * Calculates the angle that the turret is at relative to the robot using the
     * bot's startup position & turret limelight angle
     */
    public void setStartupAngle() {
        Angle swerveAngle = swerve.getState().Pose.getRotation().getMeasure();
        Angle llAngle = io.getYawFromTarget();

        // io.setStartupYaw(swerveAngle.minus(llAngle));
        io.setStartupPitch(Pitch.STARTING_ANGLE);
    }

    /**
     * Moves the pitch and yaw
     * 
     * @param pitch
     * @param yaw
     * @return a {@link Command} to be scheduled and run
     */
    public Command moveTo(Angle pitch, Angle yaw) {
        return run(() -> io.moveTo(pitch, yaw));
    }

    /**
     * Moves the yaw to the specified angle
     * 
     * @param yaw
     */
    public void moveYawTo(Angle yaw) {
        io.setYaw(yaw);
    }

    public void setYawVelocity(AngularVelocity velocity) {
        io.setYawVelocity(velocity);
    }

    /**
     * Moves the pitch to the specified angle
     * 
     * @param pitch
     */
    public void setPitch(Angle pitch) {
        io.setPitch(pitch);
    }

    /**
     * Stops both the pitch and the yaw motors
     * 
     * @return a {@link Command} to be scheduled and run
     */
    public Command stop() {
        return run(() -> io.stopMotors());
    }

    /**
     * Gets the distance from the hub april tag to the turret
     * 
     * @return
     */
    public Distance getHubDistance() {
        return io.getHubDistance();
    }

    /**
     * Checks if the turret limelight can see the hub april tags on the scoring side
     * 
     * @return a {@link Trigger} object
     */
    public Trigger hasHubLOS() {
        return new Trigger(() -> io.hasHubLOS());
    }

    public AimBot getTurretAim(boolean kineticLead) {
        return kineticLead ? kineticAimBot : staticAimBot;
    }

    public Translation2d getAllianceTarget() {
        return allianceTargetFieldPose;
    }

    @Override
    public void periodic() {
        super.periodic();
        staticAimBot = staticAimBot.adjustStaticAim(swerve.getState().Pose, new Rotation2d(getCurrentYaw()),
                allianceTargetFieldPose);
        kineticAimBot = staticAimBot.adjustKineticLead(swerve.getState().Speeds);

        SmartDashboard.putNumber("LL Turret Angle", io.getYawFromTarget().in(Rotations));
        SmartDashboard.putNumber("LL Turret Distance", io.getHubDistance().in(Meters));

        SmartDashboard.putNumber("Turret Angle", io.getYaw().in(Rotations));

        SmartDashboard.putNumber("Turret Yaw Velocity", io.getYawVelocity().in(RotationsPerSecond));

        SmartDashboard.putBoolean("Hub LOS", io.hasHubLOS());

        staticTurretSender.set(staticAimBot.turret());
        staticGoalSender.set(staticAimBot.goal());
        kineticTurretSender.set(kineticAimBot.turret());
        kineticGoalSender.set(kineticAimBot.goal());
    }

    private Turret(TurretIO io) {
        super(io);

        staticAimBot = new AimBot(new Pose2d(), new Pose2d(), new Pose2d());
        kineticAimBot = new AimBot(new Pose2d(), new Pose2d(), new Pose2d());

        boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        allianceTargetFieldPose = isRedAlliance ? Kinetics.GOAL_LOCATION_RED
                : Kinetics.GOAL_LOCATION_BLUE;

        staticTable = nt.getTable("Turret/Static");
        kineticTable = nt.getTable("Turret/Kinetic");

        staticGoalSender = staticTable.getStructTopic("staticGoal", Pose2d.struct).publish();
        staticTurretSender = staticTable.getStructTopic("staticTurret", Pose2d.struct).publish();
        kineticGoalSender = kineticTable.getStructTopic("kineticGoal", Pose2d.struct).publish();
        kineticTurretSender = kineticTable.getStructTopic("kineticTurret", Pose2d.struct).publish();
    }

    public static Turret system() {
        if (system == null) {
            if (Utils.isSimulation()) {
                system = new Turret(new TurretIO.Sim());
            } else {
                system = new Turret(new TurretIO.Real());
            }
        }
        return system;
    }

}
