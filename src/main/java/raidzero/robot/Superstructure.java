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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
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

    private static boolean isPassing = false;
    private static Trigger isPassingTrigger = new Trigger(() -> {
        return isPassing;
    });

    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.75;
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.75;

    private static final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.FieldCentric strafeControl = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private static CommandXboxController driver = new CommandXboxController(0);
    private static CommandPS5Controller operator = new CommandPS5Controller(1);
    private static final Trigger operatorShare = operator.button(9);

    private static final Auto autos = new Auto(swerve, intake, extender);

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

    // TODO: Scale zone based on ChassisSpeeds (x vel)
    private static Trigger apprTrench = new Trigger(() -> {
        Pose2d pose = turret.getTurretPose();

        boolean posX = ((pose.getX() > 3.5 && pose.getX() < 4.6) || (pose.getX() > 10.4 && pose.getX() < 11.9));
        boolean negX = ((pose.getX() > 4.6 && pose.getX() < 6.1) || (pose.getX() > 11.9 && pose.getX() < 12.7));

        return (posX || negX) && ((pose.getY() < 1.4) || (pose.getY() > 6.67));
    });

    private static Trigger apprPos = new Trigger(() -> {
        Pose2d pose = swerve.getState().Pose;

        return ((pose.getX() > 3.5 && pose.getX() < 4.6) || (pose.getX() > 10.4 && pose.getX() < 11.9));
    });

    private static Trigger apprNeg = new Trigger(() -> {
        Pose2d pose = swerve.getState().Pose;

        return ((pose.getX() > 4.6 && pose.getX() < 6.1) || (pose.getX() > 11.9 && pose.getX() < 12.7));
    });

    private static Trigger passingArea1 = new Trigger(() -> {
        Pose2d pose = swerve.getState().Pose;

        boolean inX = pose.getX() > 6.3 && pose.getX() < 10.8;
        boolean inY = pose.getY() > 0.28 && pose.getY() < 3.4;

        return inX && inY;
    });

    private static Trigger passingArea2 = new Trigger(() -> {
        Pose2d pose = swerve.getState().Pose;

        boolean inX = pose.getX() > 6.3 && pose.getX() < 10.8;
        boolean inY = pose.getY() > 4.6 && pose.getY() < 7.72;

        return inX && inY;
    });

    public static Trigger inAutonShootingZone = new Trigger(() -> {
        Pose2d pose = swerve.getState().Pose;
        boolean inAllianceZone = pose.getX() < 3.5 || pose.getX() > 12.7;
        return inAllianceZone && DriverStation.isAutonomousEnabled();
    });

    public static Trigger spindexerIsMoving = new Trigger(() -> spindexer.getVelocity().abs(RotationsPerSecond) > 1.0);

    // 1:55 1:30 1:05 :40
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

        hood.setDefaultCommand(hood.aim().onlyIf(swerveApprTrench.negate().or(driver.rightBumper())));
        // hood.setDefaultCommand(hood.stop());.
        // turret.setDefaultCommand(turret.stop());
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

        operator.circle().and(feeder.ballDetected().negate()).whileTrue(
            Commands.waitUntil(shooter.atTargetVelocity()).andThen(
                feeder.go().alongWith(
                    Commands.waitUntil(feeder.atTargetVelocity()).andThen(spindexer.antiJamBig())
                )
            )
        );
        operator.circle().and(feeder.ballDetected().negate()).whileTrue(
            hood.aim()// .onlyIf(isPassingTrigger.negate())
        );

        operator.circle().and(feeder.ballDetected()).whileTrue(
            Commands.waitUntil(shooter.atTargetVelocity()).andThen(
                feeder.go().alongWith(
                    Commands.waitUntil(feeder.atTargetVelocity()).andThen(
                        spindexer.go()
                    )
                )
            )
        );
        operator.circle().and(feeder.ballDetected()).whileTrue(
            hood.aim()// .onlyIf(isPassingTrigger.negate())
        );

        // * uncomment the nd comment above for manual antijam */
        // operator.circle().whileTrue(
        // Commands.waitUntil(shooter.atTargetVelocity())
        // .andThen(feeder.go().alongWith(Commands.waitUntil(feeder.atTargetVelocity()).andThen(spindexer.go())))
        // ).onFalse(spindexer.antiJam());

        operator.povDown().whileTrue(
            hood.aim(Degrees.of(10))
        );

        operator.square().whileTrue(
            hood.aim(Degrees.of(10))
        );
        // operator.povDown().whileTrue(
        // hood.aim(Degrees.of(10)).alongWith(
        // new StaticAim(
        // Rotations.of(
        // DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 0.5 : 0.0
        // )
        // )
        // )
        // );

        // operator.square().whileTrue(
        // hood.aim(Degrees.of(10)).alongWith(
        // new StaticAim(
        // Rotations.of(
        // DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 0.5 : 0.0
        // )
        // )
        // )
        // );

        operator.triangle().whileTrue(shooter.shoot());
        operator.triangle().onFalse(spindexer.stop().alongWith(feeder.stop()));

        operator.R1().toggleOnTrue(intake.go());
        operator.L1().whileTrue(intake.reverse());

        // operator.options().onTrue(extender.extend());
        // operatorShare.onTrue(extender.retract());
        // operator.options().and(operatorShare).onTrue(Commands.runOnce(() -> limelight.toggleLimelights()));
        operator.options().and(operatorShare).onTrue(Commands.runOnce(() -> {
            limelight.toggleLimelights(); // TODO: FIX THIS
            ElasticLib.sendNotification(new Notification(NotificationLevel.WARNING, "LL", "LIMELIGHT TOGGLE CALLED - DR"));
        }));
        operator.button(14).onTrue(extender.retract()).onFalse(extender.extend());

        operator.cross()
            .whileTrue(
                new StaticAim(
                    Rotations.of(
                        DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 0.5 : 0.0
                    )
                )
                    .alongWith(hood.aim(Pitch.PASSING_ANGLE))
                // .alongWith(Commands.runOnce(() -> {
                // isPassing = !isPassing;
                // }))
            );

        operator.povUp().onTrue(Commands.runOnce(() -> turret.setStartupAngle()));

        operator.L2().and(operator.R2()).onTrue(Commands.runOnce(() -> {
            // swerve.seedFieldCentric();
            swerve.getPigeon2().setYaw(Util.isBlueAlliance() ? 0 : 180);
            ElasticLib.sendNotification(new Notification(NotificationLevel.WARNING, "LL", "SEED CALLED - OP"));
        }));

        operator.L3().and(operator.R3())
            .onTrue(Commands.runOnce(() -> limelight.resetSeed()))
            .onFalse(Commands.runOnce(() -> limelight.setManualSeed(true)));

        // driver.povRight().onTrue(hood.incrementPitch(Degrees.of(1.0)));
        // driver.povLeft().onTrue(hood.incrementPitch(Degrees.of(-1.0)));

        // driver.povUp().onTrue(shooter.incrementVelocitySetpoint(RotationsPerSecond.of(0.5)));
        // driver.povDown().onTrue(shooter.incrementVelocitySetpoint(RotationsPerSecond.of(-0.5)));

        // driver.leftTrigger().and(feeder.ballDetected().negate()).whileTrue(
        // Commands.waitUntil(shooter.atManualTargetVelocity()).andThen(
        // feeder.go().alongWith(
        // Commands.waitUntil(feeder.atTargetVelocity()).andThen(spindexer.antiJamBig())
        // )
        // )
        // );

        // driver.leftTrigger().and(feeder.ballDetected()).whileTrue(
        // Commands.waitUntil(shooter.atManualTargetVelocity()).andThen(
        // feeder.go().alongWith(
        // Commands.waitUntil(feeder.atTargetVelocity()).andThen(
        // spindexer.go()
        // )
        // )
        // )
        // );

        // driver.rightTrigger().whileTrue(shooter.manualShoot());

        // driver.rightBumper().whileTrue(intake.go());
        // driver.x().onTrue(extender.retract());
        // driver.b().onTrue(extender.extend());

        // driver.a().onTrue(Commands.runOnce(() -> turret.setStartupAngle()));
    }

    /**
     * Binds subsystem triggers
     */
    public static void bindTriggers() {
        turret.isPirouetteing().whileTrue(spindexer.stop().alongWith(feeder.stop().andThen(shooter.stop())));
        // .onFalse(Commands.runOnce(() -> turret.setStartupAngle()));

        apprTrench.and(driver.rightBumper().negate()).and(operator.cross().negate())// .and(passingArea1.and(passingArea2).negate())
            .whileTrue(
                hood.aim(Degrees.of(10)).alongWith(
                    spindexer.stop().alongWith(feeder.stop())
                )
            );

        // apprPos.and(driver.rightBumper().negate()).and(passingArea1.and(passingArea2).negate())
        // .whileTrue(
        // hood.aim(Degrees.of(10)).alongWith(
        // spindexer.stop().alongWith(feeder.stop()).alongWith(
        // new StaticAim(
        // Rotations.of(
        // DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 0.0 : 0.5
        // )
        // )
        // )
        // )
        // );

        // apprNeg.and(driver.rightBumper().negate()).and(passingArea1.and(passingArea2).negate())
        // .whileTrue(
        // hood.aim(Degrees.of(10)).alongWith(
        // spindexer.stop().alongWith(feeder.stop()).alongWith(
        // new StaticAim(
        // Rotations.of(
        // DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 0.5 : 0.0
        // )
        // )
        // )
        // )
        // );

        // passingArea1.and(swerveApprTrench.negate()).and(() -> DriverStation.isTeleopEnabled())
        // .whileTrue(
        // new StaticAim(
        // Rotations.of(
        // DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 0.5 : 0.0
        // )
        // )
        // .alongWith(hood.aim(Pitch.PASSING_ANGLE))
        // )
        // .onFalse(shooter.stop());

        // passingArea2.and(swerveApprTrench.negate()).and(() -> DriverStation.isTeleopEnabled())
        // .whileTrue(
        // new StaticAim(
        // Rotations.of(
        // DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? 0.5 : 0.0
        // )
        // )
        // .alongWith(hood.aim(Pitch.PASSING_ANGLE))
        // )
        // .onFalse(shooter.stop());

        // autoSpindexTrigger.and(feeder.ballDetected().negate()).whileTrue(
        // feeder.go().alongWith(spindexer.antiJamBig())
        // );

        // autoSpindexTrigger.and(feeder.ballDetected()).whileTrue(
        // feeder.go().alongWith(spindexer.go())
        // );

        // canShootAuton.whileTrue(shooter.shoot());

        // shouldSpindexAuton.and(feeder.ballDetected().negate()).whileTrue(
        // feeder.go().alongWith(spindexer.antiJamBig()));

        // shouldSpindexAuton.and(feeder.ballDetected()).whileTrue(
        // feeder.go().alongWith(spindexer.go()));

        inAutonShootingZone.whileTrue(
            shooter.shoot().until(() -> !inAutonShootingZone.getAsBoolean() && !spindexerIsMoving.getAsBoolean())
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