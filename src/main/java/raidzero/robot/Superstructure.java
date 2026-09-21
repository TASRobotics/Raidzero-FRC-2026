package raidzero.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.Util;
import raidzero.robot.commands.Aim;
import raidzero.robot.subsystems.drive.Limelight;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.drive.TunerConstants;
import raidzero.robot.subsystems.intake.Extender;
import raidzero.robot.subsystems.intake.Intake;
import raidzero.robot.subsystems.intake.Spindexer;
import raidzero.robot.subsystems.shooter.Feeder;
import raidzero.robot.subsystems.shooter.Shooter;
import raidzero.robot.subsystems.turret.Hood;
import raidzero.robot.subsystems.turret.Turret;

public class Superstructure {
    public static CANBus canDoAttitude = new CANBus("canDoAttitude");

    private static final Shooter shooter = Shooter.system();
    private static final Turret turret = Turret.system();
    public static final Hood hood = Hood.system();
    private static final Feeder feeder = Feeder.system();
    private static final Spindexer spindexer = Spindexer.system();
    private static final Extender extender = Extender.system();
    private static final Intake intake = Intake.system();
    private static final Swerve swerve = Swerve.system();
    private static final Limelight limelight = Limelight.system();

    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.6;
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.50;

    private static CommandXboxController driver = new CommandXboxController(0);
    private static CommandXboxController operator = new CommandXboxController(1);

    private static final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.08).withRotationalDeadband(MaxAngularRate * 0.08)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final Auto autos = new Auto(swerve, intake, extender);

    private static SendableChooser<Command> autoChooser;

    /**
     * Runs the initialization routines for subsystems
     */
    public static void runSubsystemInitRoutines() {
        turret.setStartupAngle();
        hood.setStartupAngle();
        extender.setStartupAngle();
    }

    /**
     * Gets manual turret control input from driver or operator right joystick X.
     * Pulling left is negative, pulling right is positive.
     *
     * @return Joystick input [-1.0, 1.0]
     */
    public static double getTurretManualInput() {
        double driverX = driver.getRightX();
        if (Math.abs(driverX) > 0.08) {
            return driverX;
        }
        double operatorX = operator.getRightX();
        if (Math.abs(operatorX) > 0.08) {
            return operatorX;
        }
        return 0.0;
    }

    /**
     * Creates a command that spins up the shooter and aims the hood to the preset angle,
     * and once the shooter reaches target velocity, runs the feeder and spindexer.
     *
     * @return A {@link Command} for shooting
     */
    public static Command createShootCommand() {
        return shooter.shoot()
            .alongWith(hood.aimPreset())
            .alongWith(
                Commands.waitUntil(shooter.atTargetVelocity())
                    .andThen(feeder.go().alongWith(spindexer.go()))
            );
    }

    /**
     * Applies the default commands
     */
    public static void applyDefaultCommands() {
        swerve.setDefaultCommand(
            swerve.applyRequest(
                () -> fieldCentricDrive
                    .withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(0.0)
            )
        );

        hood.setDefaultCommand(hood.aimResting());
        // hood.setDefaultCommand(hood.manualAim());
        turret.setDefaultCommand(turret.manualDrive(Superstructure::getTurretManualInput));

        shooter.setDefaultCommand(shooter.stop());
        feeder.setDefaultCommand(feeder.stop());
        spindexer.setDefaultCommand(spindexer.stop());
        extender.setDefaultCommand(extender.stop());
        intake.setDefaultCommand(intake.stop());
    }

    /**
     * Applies driver button bindings
     */
    public static void applyButtonBindings() {
        // Turret zeroing / relative forward position
        driver.a().onTrue(Commands.runOnce(() -> turret.setStartupAngle()));
        driver.x().onTrue(Commands.runOnce(() -> turret.setStartupAngle()));

        // AimBot: hold button B to aim turret at goal (without shooting)
        driver.b().whileTrue(new Aim());
        operator.b().whileTrue(new Aim());

        // Left trigger: spin up shooter & aim hood, feed when ready
        driver.leftTrigger().whileTrue(createShootCommand());
        operator.leftTrigger().whileTrue(createShootCommand());
    }

    /**
     * Applies debug button bindings
     */
    public static void applyDebugBindings() {}

    /**
     * Binds subsystem triggers
     */
    public static void bindTriggers() {
        turret.isPirouetteing().whileTrue(spindexer.stop().alongWith(feeder.stop().andThen(shooter.stop())));
    }

    /**
     * Gets the Limelight subsystem
     *
     * @return The {@link Limelight} subsystem instance
     */
    public static Limelight getLimelight() {
        return limelight;
    }

    /**
     * Gets the Hood subsystem
     *
     * @return The {@link Hood} subsystem instance
     */
    public static Hood getHood() {
        return hood;
    }

    /**
     * Gets the Swerve subsystem
     *
     * @return The {@link Swerve} subsystem instance
     */
    public static Swerve getSwerve() {
        return swerve;
    }

    /**
     * Configures autonomous command options
     */
    public static void configureAutons() {
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Depot single", autos.getDepot1Cycle());
        autoChooser.addOption("Left Double", autos.getLeft2Cycle());
        autoChooser.addOption("Right Oupost", autos.getRightOutpostAuto());
        autoChooser.addOption("Right Double", autos.getRight2Cycle());
        autoChooser.addOption("Right Outpost Double", autos.getRightOutpostDoubleAuto());

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Gets the selected autonomous command
     *
     * @return A {@link Command} for the selected autonomous routine
     */
    public static Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}