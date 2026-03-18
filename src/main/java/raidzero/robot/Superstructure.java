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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.ElasticLib;
import raidzero.lib.ElasticLib.Notification;
import raidzero.lib.ElasticLib.NotificationLevel;
import raidzero.lib.Util;
import raidzero.robot.commands.Aim;
import raidzero.robot.commands.StaticAim;
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
import raidzero.robot.subsystems.turret.TurretConstants.Pitch;

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

    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.75;
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.75;

    private static final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.FieldCentric strafeControl = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private static CommandXboxController driver = new CommandXboxController(0);
    private static CommandXboxController operator = new CommandXboxController(1);
    private static final Trigger operatorShare = operator.button(9);

    private static final Auto autos = new Auto(swerve, intake, extender);

    private static SendableChooser<Command> autoChooser;

    // TODO: Scale zone based on ChassisSpeeds (x vel)
    private static Trigger apprTrench = new Trigger(() -> {
        Pose2d pose = turret.getTurretPose();

        boolean posX = ((pose.getX() > 3.5 && pose.getX() < 4.6) || (pose.getX() > 10.4 && pose.getX() < 11.9));
        boolean negX = ((pose.getX() > 4.6 && pose.getX() < 6.1) || (pose.getX() > 11.9 && pose.getX() < 12.7));

        return (posX || negX) && ((pose.getY() < 1.4) || (pose.getY() > 6.67));
    });

    public static Trigger inAutonShootingZone = new Trigger(() -> {
        Pose2d pose = swerve.getState().Pose;
        boolean inAllianceZone = pose.getX() < 3.5 || pose.getX() > 12.7;
        return inAllianceZone && DriverStation.isAutonomousEnabled();
    });

    // 1:55 1:30 1:05 0:40
    public static Trigger timestampOne = new Trigger(() -> {
        return DriverStation.getMatchTime() <= 116;
    });
    public static Trigger timestampTwo = new Trigger(() -> {
        return DriverStation.getMatchTime() <= 91;
    });
    public static Trigger timestampThree = new Trigger(() -> {
        return DriverStation.getMatchTime() <= 66;
    });
    public static Trigger timestampFour = new Trigger(() -> {
        return DriverStation.getMatchTime() <= 41;
    });

    /**
     * Runs the initialization routines for subsystems
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
        swerve.setDefaultCommand(
            swerve.applyRequest(
                () -> fieldCentricDrive
                    .withVelocityX(-driver.getLeftY() * MaxSpeed)
                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        hood.setDefaultCommand(hood.aim().onlyIf(apprTrench.negate().or(driver.rightBumper())));
        turret.setDefaultCommand(new Aim());

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
        driver.povUp().whileTrue(
            swerve.applyRequest(
                () -> strafeControl.withVelocityX(0.75).withVelocityY(0)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        driver.povDown().whileTrue(
            swerve.applyRequest(
                () -> strafeControl.withVelocityX(-0.75).withVelocityY(0)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        driver.povLeft().whileTrue(
            swerve.applyRequest(
                () -> strafeControl.withVelocityX(0).withVelocityY(0.75)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        driver.povRight().whileTrue(
            swerve.applyRequest(
                () -> strafeControl.withVelocityX(0).withVelocityY(-0.75)
                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)
            )
        );

        driver.rightBumper().whileTrue(swerve.applyRequest(() -> brake));
        driver.b().whileTrue(hood.aim(Degrees.of(10)));

        driver.x().onTrue(Commands.runOnce(() -> turret.setStartupAngle()));

        driver.leftTrigger().and(driver.rightTrigger()).onTrue(Commands.runOnce(() -> {
            swerve.getPigeon2().setYaw(Util.isBlueAlliance() ? 0 : 180);
            ElasticLib.sendNotification(new Notification(NotificationLevel.WARNING, "LL", "SEED CALLED - DR"));
        }));

        operator.b().whileTrue(hood.aim());

        operator.b().and(feeder.ballDetected().negate()).whileTrue(
            Commands.waitUntil(shooter.atTargetVelocity()).andThen(
                feeder.go().alongWith(
                    Commands.waitUntil(feeder.atTargetVelocity()).andThen(spindexer.antiJamBig())
                )
            )
        );

        operator.b().and(feeder.ballDetected()).whileTrue(
            Commands.waitUntil(shooter.atTargetVelocity()).andThen(
                feeder.go().alongWith(
                    Commands.waitUntil(feeder.atTargetVelocity()).andThen(
                        spindexer.go()
                    )
                )
            )
        );

        operator.povDown().whileTrue(
            hood.aim(Degrees.of(10))
        );

        operator.x().whileTrue(
            hood.aim(Degrees.of(10))
        );

        operator.y().whileTrue(shooter.shoot());
        operator.y().onFalse(spindexer.stop().alongWith(feeder.stop()));

        operator.rightBumper().toggleOnTrue(intake.go());
        operator.leftBumper().whileTrue(intake.reverse());

        operator.start().and(operatorShare).onTrue(Commands.runOnce(() -> {
            limelight.toggleLimelights(); // TODO: FIX THIS
            ElasticLib.sendNotification(
                new Notification(NotificationLevel.WARNING, "LL", "LIMELIGHT TOGGLE CALLED - DR")
            );
        }));
        operator.back().onTrue(extender.retract()).onFalse(extender.extend());

        operator.a()
            .whileTrue(
                new StaticAim(
                    Rotations.of(
                        DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue ? 0.5 : 0.5
                    )
                )
                    .alongWith(hood.aim(Pitch.PASSING_ANGLE))
            );

        operator.povUp().onTrue(Commands.runOnce(() -> turret.setStartupAngle()));

        operator.leftTrigger().and(operator.rightTrigger()).onTrue(Commands.runOnce(() -> {
            swerve.getPigeon2().setYaw(Util.isBlueAlliance() ? 0 : 180);
            ElasticLib.sendNotification(new Notification(NotificationLevel.WARNING, "LL", "SEED CALLED - OP"));
        }));

        operator.leftStick().and(operator.rightStick())
            .onTrue(Commands.runOnce(() -> limelight.resetSeed()))
            .onFalse(Commands.runOnce(() -> limelight.setManualSeed(true)));

    }

    public static void applyDebugBindings() {
        driver.povRight().onTrue(hood.incrementPitch(Degrees.of(1.0)));
        driver.povLeft().onTrue(hood.incrementPitch(Degrees.of(-1.0)));

        driver.povUp().onTrue(shooter.incrementVelocitySetpoint(RotationsPerSecond.of(0.5)));
        driver.povDown().onTrue(shooter.incrementVelocitySetpoint(RotationsPerSecond.of(-0.5)));

        driver.leftTrigger().and(feeder.ballDetected().negate()).whileTrue(
            Commands.waitUntil(shooter.atManualTargetVelocity()).andThen(
                feeder.go().alongWith(
                    Commands.waitUntil(feeder.atTargetVelocity()).andThen(spindexer.antiJamBig())
                )
            )
        );

        driver.leftTrigger().and(feeder.ballDetected()).whileTrue(
            Commands.waitUntil(shooter.atManualTargetVelocity()).andThen(
                feeder.go().alongWith(
                    Commands.waitUntil(feeder.atTargetVelocity()).andThen(
                        spindexer.go()
                    )
                )
            )
        );

        driver.rightTrigger().whileTrue(shooter.manualShoot());

        driver.rightBumper().whileTrue(intake.go());
        driver.x().onTrue(extender.retract());
        driver.b().onTrue(extender.extend());

        driver.a().onTrue(Commands.runOnce(() -> turret.setStartupAngle()));
    }

    /**
     * Binds subsystem triggers
     */
    public static void bindTriggers() {
        turret.isPirouetteing().whileTrue(spindexer.stop().alongWith(feeder.stop().andThen(shooter.stop())));

        apprTrench.and(driver.rightBumper().negate()).and(operator.a().negate())
            .whileTrue(
                hood.aim(Degrees.of(10)).alongWith(
                    spindexer.stop().alongWith(feeder.stop())
                )
            );

        inAutonShootingZone.whileTrue(
            shooter.shoot()
                .until(() -> !inAutonShootingZone.getAsBoolean() && !spindexer.stopped().getAsBoolean())
        );

        inAutonShootingZone.and(feeder.ballDetected()).whileTrue(
            Commands.waitUntil(shooter.atTargetVelocity()).andThen(
                feeder.go().alongWith(spindexer.go())
            )
        );

        inAutonShootingZone.and(feeder.ballDetected().negate()).whileTrue(
            Commands.waitUntil(shooter.atTargetVelocity()).andThen(
                feeder.go().alongWith(spindexer.antiJamBig())
            )
        );

        inAutonShootingZone.negate().whileTrue(hood.aim(Degrees.of(10)));

        timestampOne.onTrue(Commands.run(() -> {
            driver.setRumble(RumbleType.kBothRumble, 1);
        }).withTimeout(1).finallyDo(() -> {
            driver.setRumble(RumbleType.kBothRumble, 0);
        }));
        timestampTwo.onTrue(Commands.run(() -> {
            driver.setRumble(RumbleType.kBothRumble, 1);
        }).withTimeout(1).finallyDo(() -> {
            driver.setRumble(RumbleType.kBothRumble, 0);
        }));
        timestampThree.onTrue(Commands.run(() -> {
            driver.setRumble(RumbleType.kBothRumble, 1);
        }).withTimeout(1).finallyDo(() -> {
            driver.setRumble(RumbleType.kBothRumble, 0);
        }));
        timestampFour.onTrue(Commands.run(() -> {
            driver.setRumble(RumbleType.kBothRumble, 1);
        }).withTimeout(1).finallyDo(() -> {
            driver.setRumble(RumbleType.kBothRumble, 0);
        }));
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
     * Configures autonomous command options
     */
    public static void configureAutons() {
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Depot single", autos.getDepot1Cycle());
        autoChooser.addOption("Left Double", autos.getLeft2Cycle());
        autoChooser.addOption("Right Oupost", autos.getRightOutpostAuto());
        autoChooser.addOption("Right Double", autos.getRight2Cycle());

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