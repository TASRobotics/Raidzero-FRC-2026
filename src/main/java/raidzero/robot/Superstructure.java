package raidzero.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    private static Shooter shooter = Shooter.system();
    private static Turret turret = Turret.system();
    private static Hood hood = Hood.system();
    private static Feeder feeder = Feeder.system();
    private static Spindexer spindexer = Spindexer.system();
    private static Extender extender = Extender.system();
    private static Intake intake = Intake.system();
    private static Limelight limelight = Limelight.system();
    private static Swerve swerve = Swerve.system();

    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.5;
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.5;

    private static final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static CommandXboxController driver = new CommandXboxController(0);

    private static final Auto autos = new Auto(Swerve.system());

    private static SendableChooser<Command> autoChooser;

    public static final Trigger hubActive = new Trigger(() -> {
        double t = Timer.getMatchTime();
        if (t < 0)
            return false;

        // >140 = auto, >130 = transition, <=30 = endgame
        // Match timing (seconds remaining)
        if (t > 140 || t > 130 || t <= 30)
            return true;
        // >140 = auto, >130 = transition, <=30 = endgame

        int shift = (t > 105) ? 1 : (t > 80) ? 2 : (t > 55) ? 3 : 4;

        String data = DriverStation.getGameSpecificMessage();
        if (data == null || data.isEmpty())
            return true;

        boolean redStartsInactive = data.charAt(0) == 'R';
        var allianceOpt = DriverStation.getAlliance();
        if (allianceOpt.isEmpty())
            return true;

        boolean isRed = allianceOpt.get() == DriverStation.Alliance.Red;
        boolean startedInactive = (isRed && redStartsInactive) || (!isRed && !redStartsInactive);

        boolean inactiveNow = ((shift - 1) % 2 == 0) ? startedInactive : !startedInactive;
        return !inactiveNow;
    });

    private static Trigger swerveApprTrench = new Trigger(() -> {
        Pose2d pose = swerve.getState().Pose;

        boolean posX = ((pose.getX() > 4.0 && pose.getX() < 5.1) || (pose.getX() > 10.94 && pose.getX() < 11.44));
        boolean negX = ((pose.getX() > 5.1 && pose.getX() < 6.1) || (pose.getX() > 11.44 && pose.getX() < 12.14));

        return (posX || negX) && ((pose.getY() < 1.4) || (pose.getY() > 6.67));
    });

    /**
     * RUns the initialization routines for subsystems
     */
    public static void runSubsystemInitRoutines() {
        turret.setStartupAngle();
        hood.setStartupAngle();
        extender.setStartupAngle();
    }

    /**
     * Applies the default commands
     */
    public static void applyDefaultCommands() {
        // CAN_SHOOT.whileTrue(shooter.shoot());

        swerve.setDefaultCommand(
            swerve.applyRequest(
                () -> fieldCentricDrive
                    .withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        shooter.setDefaultCommand(shooter.stop());
        hood.setDefaultCommand(hood.manualAim());
        turret.setDefaultCommand(new Aim());
        feeder.setDefaultCommand(feeder.stop());
        spindexer.setDefaultCommand(spindexer.stop());
        extender.setDefaultCommand(extender.stop());
        intake.setDefaultCommand(intake.stop());
    }

    /**
     * Applies driver button bindings
     */
    public static void applyButtonBindings() {
        driver.leftTrigger().whileTrue(spindexer.go().alongWith(feeder.go())).onFalse(spindexer.antiJam());
        // driver.leftTrigger().whileTrue(
        // driver.leftTrigger().and(feeder.ballDetected().negate()).whileTrue(
        // (spindexer.go()
        // .withTimeout(0.5)
        // .andThen(spindexer.antiJamBig()))
        // );
        driver.leftTrigger().whileTrue(feeder.go());

        // );
        driver.rightTrigger().whileTrue(shooter.manualShoot());

        driver.start().onTrue(swerve.runOnce(swerve::seedFieldCentric));
        driver.rightStick().onTrue(Commands.runOnce(Superstructure::runSubsystemInitRoutines, turret, hood));

        driver.leftBumper().whileTrue(intake.go());
        driver.povUp().onTrue(extender.increaseAgitate());
        driver.povDown().onTrue(extender.decreaseAgitate());
        driver.b().onTrue(extender.extend());
        driver.x().onTrue(extender.retract());

        // driver.a().onTrue(
        // extender.agitate().alongWith(spindexer.antiJam())
        // );
        driver.povUp().onTrue(shooter.incrementVelocitySetpoint(RotationsPerSecond.of(0.5)));
        driver.povDown().onTrue(shooter.incrementVelocitySetpoint(RotationsPerSecond.of(-0.5)));

        driver.povRight().onTrue(hood.incrementPitch(Degrees.of(1.0)));
        driver.povLeft().onTrue(hood.incrementPitch(Degrees.of(-1.0)));

    }

    /**
     * Binds subsystem triggers
     */
    public static void bindTriggers() {
        swerveApprTrench.whileTrue(hood.aim(Degrees.of(10)));
    }

    /**
     * Configures autonomous command options
     */
    public static void configureAutons() {
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Depot single", autos.getDepot1Cycle());
        autoChooser.addOption("Left double", autos.getLeft2Cycle());
        autoChooser.addOption("Right Oupost", autos.getRightOutpostAuto());

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