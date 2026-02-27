package raidzero.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.R0Subsystem;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.turret.TurretConstants.Kinetics;
import raidzero.robot.subsystems.turret.TurretConstants.Yaw;

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

    private final LinearFilter vxFilter = LinearFilter.movingAverage(5);
    private final LinearFilter vyFilter = LinearFilter.movingAverage(5);
    private final LinearFilter omegaFilter = LinearFilter.movingAverage(5);

    private AimBot staticAimBot = null;
    private AimBot kineticAimBot = null;

    // todo: auto set the turret to unwrap when idle (not shooting)
    // todo: hood backlash compensation (go down and back up when moving in the negative direction to compensate)

    /**
     * Moves the robot to the given setpoint
     *
     * @param setpoint The setpoint to move to
     */
    public void moveTo(Angle setpoint) {
        io.moveTo(setpoint);
    }

    /**
     * Moves the yaw at the given duty cycle percent
     *
     * @param percent the percent [0,1]
     * @return A {@link Command} to be scheduled and run
     */
    public Command movePercent(double percent) {
        return run(() -> io.runYawPercent(percent));
    }

    /**
     * Stops both the pitch and the yaw motors
     *
     * @return A {@link Command} to be scheduled and run
     */
    public Command stop() {
        return run(() -> io.stop());
    }

    public Angle getYawFromTarget() {
        return io.getYawFromTarget();
    }

    public Angle getCurrentYaw() {
        return io.getYaw();
    }

    /**
     * Sets the startup angle of the turret
     */
    public void setStartupAngle() {}

    /**
     * Gets the distance from the hub april tag to the turret
     *
     * @param kineticLead Whether or not to use the kinetic leading pose or the static aiming pose for distance calculation (kinetic leading pose is more accurate when the robot is moving)
     * @return The distance from the hub april tag to the turret in meters
     */
    public Distance getHubDistance(boolean kineticLead) {
        return Meters.of(
            kineticLead ? kineticAimBot.turret().getTranslation().getDistance(allianceTargetFieldPose) :
                staticAimBot.turret().getTranslation().getDistance(allianceTargetFieldPose)
        );
    }

    /**
     * Checks if the turret limelight can see the hub april tags on the scoring side
     *
     * @return A {@link Trigger} object
     */
    public Trigger hasHubLOS() {
        return new Trigger(() -> io.hasHubLOS());
    }

    /**
     * Gets the AimBot instance for static/kinetic aiming
     *
     * @param kineticLead wether or not to use kinetic leading for shoot-on-the-move
     * @return the AimBot instance
     */
    public AimBot getTurretAim(boolean kineticLead) {
        return kineticLead ? kineticAimBot : staticAimBot;
    }

    /**
     * Gets the goal pose on the current alliance
     *
     * @return The goal pose on the current alliance in field-relative coordinates
     */
    public Translation2d getAllianceTarget() {
        return allianceTargetFieldPose;
    }

    /**
     * Uses the Scream Remainder Theorem to calculate the turret angle
     * 
     * @param teethOnFirst The number of teeth on the first gear
     * @param teethOnSecond The number of teeth on the second gear
     * @param readingOnFirst The reading from the first gear
     * @param readingOnSecond The reading from the second gear
     * @return The calculated turret angle in rotations
     */
    private double ScreamRemainderTheorem(int teethOnFirst, int teethOnSecond, double readingOnFirst, double readingOnSecond) {
        double[] possibleReadings = new double[(int) teethOnSecond];
        for (int i = 0; i < (int) teethOnSecond; i++) {
            possibleReadings[i] = (readingOnFirst + i * teethOnFirst) % teethOnSecond;
            if (possibleReadings[i] < 0) {
                possibleReadings[i] += teethOnSecond;
            }
        }
        for (int i = 0; i < (int) teethOnSecond; i++) {
            if (Math.abs(possibleReadings[i] - readingOnSecond) < Yaw.TOLERANCE_SRT) {
                return i * teethOnFirst + readingOnFirst;
            }
        }
        DriverStation.reportWarning("Looks like the SRT failed in Turret.java", false);
        return 0.0;
    }

    @Override
    public void periodic() {
        super.periodic();

        ChassisSpeeds rawSpeeds = swerve.getState().Speeds;
        ChassisSpeeds filteredSpeeds = new ChassisSpeeds(
            vxFilter.calculate(rawSpeeds.vxMetersPerSecond),
            vyFilter.calculate(rawSpeeds.vyMetersPerSecond),
            omegaFilter.calculate(rawSpeeds.omegaRadiansPerSecond)
        );

        staticAimBot = staticAimBot.adjustStaticAim(
            swerve.getState().Pose, new Rotation2d(getCurrentYaw()),
            allianceTargetFieldPose
        );

        kineticAimBot = staticAimBot.adjustKineticLead(filteredSpeeds);

        // SmartDashboard.putNumber("LL Turret Angle",
        // io.getYawFromTarget().in(Rotations));
        // SmartDashboard.putNumber("LL Turret Distance",
        // io.getHubDistance().in(Meters));

        SmartDashboard.putNumber("Turret Angle", io.getYaw().in(Rotations));

        // SmartDashboard.putBoolean("Hub LOS", io.hasHubLOS());

        staticTurretSender.set(staticAimBot.turret());
        staticGoalSender.set(staticAimBot.goal());
        kineticTurretSender.set(kineticAimBot.turret());
        kineticGoalSender.set(kineticAimBot.goal());

        SmartDashboard.putNumber(
            "Static Hub Distance",
            staticAimBot.turret().getTranslation().getDistance(allianceTargetFieldPose)
        );
        SmartDashboard.putNumber(
            "Kinetic Hub Distance",
            kineticAimBot.turret().getTranslation().getDistance(allianceTargetFieldPose)
        );
    }

    private Turret(TurretIO io) {
        super(io);

        staticAimBot = new AimBot(new Pose2d(), new Pose2d(), new Pose2d());
        kineticAimBot = new AimBot(new Pose2d(), new Pose2d(), new Pose2d());

        boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        allianceTargetFieldPose = isRedAlliance ? Kinetics.GOAL_LOCATION_RED : Kinetics.GOAL_LOCATION_BLUE;

        staticTable = nt.getTable("Turret/Static");
        kineticTable = nt.getTable("Turret/Kinetic");

        staticGoalSender = staticTable.getStructTopic("staticGoal", Pose2d.struct).publish();
        staticTurretSender = staticTable.getStructTopic("staticTurret", Pose2d.struct).publish();
        kineticGoalSender = kineticTable.getStructTopic("kineticGoal", Pose2d.struct).publish();
        kineticTurretSender = kineticTable.getStructTopic("kineticTurret", Pose2d.struct).publish();
    }

    /**
     *  Gets the {@link Turret} subsystem instance
      *
     * @return The {@link Turret} subsystem instance
     */
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
