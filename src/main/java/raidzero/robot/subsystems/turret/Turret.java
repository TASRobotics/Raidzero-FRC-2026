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

    private boolean isPirouetteing = false;

    /**
     * Moves the robot to the given setpoint
     *
     * @param setpoint The setpoint to move to
     */
    public void moveTo(Angle setpoint) {
        io.moveTo(this.findBestWrap(setpoint));
    }

    private Angle findBestWrap(Angle desiredAngle) {
        double desiredRotations = desiredAngle.in(Rotations);
        double currentRotations = io.getYaw().in(Rotations);

        double revLimit = Yaw.REVERSE_SOFT_LIMIT.in(Rotations);
        double fwdLimit = Yaw.FORWARD_SOFT_LIMIT.in(Rotations);
        double bestSetpoint = currentRotations;
        double bestDistance = Double.MAX_VALUE;

        for (int i = -2; i <= 2; i++) {
            double candidate = desiredRotations + i;
            if (candidate >= revLimit && candidate <= fwdLimit) {
                double dist = Math.abs(candidate - currentRotations);
                if (dist < bestDistance) {
                    bestDistance = dist;
                    bestSetpoint = candidate;
                }
            }
        }
        isPirouetteing = bestDistance > 0.5;

        return Rotations.of(bestSetpoint);
    }

    public Trigger isPirouetteing() {
        return new Trigger(() -> this.isPirouetteing);
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

    /**
     * Gets the current yaw of the turret
     *
     * @return the current yaw of the turret as an {@link Angle}
     */
    public Angle getCurrentYaw() {
        return io.getYaw();
    }

    /**
     * Sets the startup angle of the turret
     */
    public void setStartupAngle() {
        Angle startupYaw = JaredModuloMagic(io.getJMM1Reading(), io.getJMM2Reading());
        io.setStartupYaw(startupYaw);
    }

    /**
     * Gets the distance from the hub april tag to the turret
     *
     * @param kineticLead whether or not to use kinetic leading for
     *                    shoot-on-the-move
     * @return the distance from the hub april tag to the turret as a
     *         {@link Distance}
     */
    public Distance getHubDistance(boolean kineticLead) {
        return Meters.of(
            kineticLead ? kineticAimBot.turret().getTranslation().getDistance(allianceTargetFieldPose) :
                staticAimBot.turret().getTranslation().getDistance(allianceTargetFieldPose)
        );
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
     * @return the goal pose on the current alliance as a {@link Translation2d}
     */
    public Translation2d getAllianceTarget() {
        return allianceTargetFieldPose;
    }

    /**
     * Calculates the absolute turret angle using the Jared Modulo Magic Method
     *
     * @param readingOnFirst  the reading from the first Jared modulo method sensor
     * @param readingOnSecond the reading from the second Jared modulo method sensor
     * @return the calculated absolute turret angle as an {@link Angle}
     */
    private Angle JaredModuloMagic(double readingOnFirst, double readingOnSecond) {
        double[] possibleReadings = new double[(int) Yaw.JARED_TEETH_1];
        for (int i = 0; i < (int) Yaw.JARED_TEETH_2; i++) {
            possibleReadings[i] = (readingOnFirst + i * Yaw.JARED_TEETH_1) % Yaw.JARED_TEETH_2;
            if (possibleReadings[i] < 0) {
                possibleReadings[i] += Yaw.JARED_TEETH_2;
            }
        }
        for (int i = 0; i < (int) Yaw.JARED_TEETH_2; i++) {
            if (Math.abs(possibleReadings[i] - readingOnSecond) < Yaw.TOLERANCE_SRT) {
                // SmartDashboard.putBoolean("JMM Success", true);

                double raw_angle = i * Yaw.JARED_TEETH_1 + readingOnFirst;

                double turret_rotations = raw_angle / Yaw.TURRET_RING_TEETH;

                SmartDashboard.putNumber("JMM Raw Angle", turret_rotations);

                if (turret_rotations > Yaw.FORWARD_SOFT_LIMIT.in(Rotations)) {
                    return Rotations
                        .of(turret_rotations - 2.278);
                }

                return Rotations.of(turret_rotations);
            }
        }

        DriverStation.reportWarning("Looks like the SRT failed in Turret.java", false);
        return Rotations.of(0.0);
    }

    public double getJmmValue() {
        return this.JaredModuloMagic(io.getJMM1Reading(), io.getJMM2Reading()).in(Rotations);
    }

    /**
     * Gets the current turret pose
     *
     * @return The current turret pose as a {@link Pose2d}
     */
    public Pose2d getTurretPose() {
        return staticAimBot.turret();
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Yaw", io.getYaw().in(Rotations));

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

        // SmartDashboard.putNumber("Turret Angle", io.getYaw().in(Rotations));
        // SmartDashboard.putNumber(
        // "Jmm Value",
        // this.JaredModuloMagic(io.getJMM1Reading(), io.getJMM2Reading()).in(Rotations)
        // );

        // SmartDashboard.putBoolean("Hub LOS", io.hasHubLOS());

        staticTurretSender.set(staticAimBot.turret());
        staticGoalSender.set(staticAimBot.goal());
        kineticTurretSender.set(kineticAimBot.turret());
        kineticGoalSender.set(kineticAimBot.goal());

        SmartDashboard.putNumber(
            "Static Hub Distance",
            staticAimBot.turret().getTranslation().getDistance(allianceTargetFieldPose)
        );
        // SmartDashboard.putNumber(
        // "Kinetic Hub Distance",
        // kineticAimBot.turret().getTranslation().getDistance(allianceTargetFieldPose)
        // );
    }

    /**
     * Constructs a Turret instance with the given IO implementation
     *
     * @param io the IO implementation to use
     */
    private Turret(TurretIO io) {
        super(io);

        staticAimBot = new AimBot(new Pose2d(), new Pose2d(), new Pose2d());
        kineticAimBot = new AimBot(new Pose2d(), new Pose2d(), new Pose2d());

        boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        allianceTargetFieldPose = isRedAlliance ? Kinetics.GOAL_LOCATION_RED : Kinetics.GOAL_LOCATION_BLUE;
        allianceTargetFieldPose = isRedAlliance ? Kinetics.GOAL_LOCATION_RED : Kinetics.GOAL_LOCATION_BLUE;

        staticTable = nt.getTable("Turret/Static");
        kineticTable = nt.getTable("Turret/Kinetic");

        staticGoalSender = staticTable.getStructTopic("staticGoal", Pose2d.struct).publish();
        staticTurretSender = staticTable.getStructTopic("staticTurret", Pose2d.struct).publish();
        kineticGoalSender = kineticTable.getStructTopic("kineticGoal", Pose2d.struct).publish();
        kineticTurretSender = kineticTable.getStructTopic("kineticTurret", Pose2d.struct).publish();
    }

    /**
     * Gets the singleton instance of the Turret subsystem, creating it if it
     * doesn't exist
     *
     * @return the singleton instance of the Turret subsystem
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
