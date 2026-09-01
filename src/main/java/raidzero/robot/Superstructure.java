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
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.lib.ElasticLib;
import raidzero.lib.ElasticLib.Notification;
import raidzero.lib.ElasticLib.NotificationLevel;
import raidzero.lib.LimelightState.IMU_MODE;
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

    private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.6;
    private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.50;

    private static CommandXboxController driver = new CommandXboxController(0);
    private static CommandXboxController operator = new CommandXboxController(1);

    private static final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.08).withRotationalDeadband(MaxAngularRate * 0.08)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private static final SwerveRequest.FieldCentric strafeControl = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // POV-based strafing: compute velocities from POV angle so diagonals and
    // cardinals
    // map consistently to field-centric X/Y. Use MaxSpeed scaling so feel matches
    // stick driving. POV angles are clockwise with 0 = up (forward), so convert
    // to radians with a negative sign to map to the standard unit circle.
    private static final double strafeScale = 0.25;

    // Helper inline lambda to generate a field-centric request for a given POV
    // angle
    private static final java.util.function.IntFunction<SwerveRequest.FieldCentric> povStrafe = (angle) -> {
        double rad = Math.toRadians(-angle);
        double vx = MaxSpeed * strafeScale * Math.cos(rad);
        double vy = MaxSpeed * strafeScale * Math.sin(rad);
        return strafeControl.withVelocityX(vx).withVelocityY(vy)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate);
    };

    private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private static final Auto autos = new Auto(swerve, intake, extender);

    private static SendableChooser<Command> autoChooser;

    // TODO: Scale zone based on ChassisSpeeds (x vel)
    private static Trigger apprTrench = new Trigger(() -> {
        Pose2d pose = turret.getTurretPose();

        boolean posX = ((pose.getX() > 3.5 && pose.getX() < 4.6) || (pose.getX() > 10.4 && pose.getX() < 11.9));
        boolean negX = ((pose.getX() > 4.6 && pose.getX() < 6.1) || (pose.getX() > 11.9 && pose.getX() < 12.7));

        return (posX || negX) && ((pose.getY() < 1.4) || (pose.getY() > 6.67));
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
        boolean inAllianceZone = (pose.getX() < 3.5 && pose.getX() > 3) || (pose.getX() > 12.7 && pose.getX() < 13.5);
        return inAllianceZone && DriverStation.isAutonomousEnabled();
    });

    public static Trigger inAutonReverseZone = new Trigger(() -> {
        Pose2d pose = swerve.getState().Pose;
        boolean inAllianceZone = (pose.getX() > 3.5 && pose.getX() < 5.1) || (pose.getX() > 12.7 && pose.getX() < 11.2);
        return inAllianceZone && DriverStation.isAutonomousEnabled();
    });

    // 1:55 1:30 1:05 0:40
    public static Trigger timestampOne = new Trigger(() -> {
        return DriverStation.getMatchTime() <= 118;
    });
    public static Trigger timestampTwo = new Trigger(() -> {
        return DriverStation.getMatchTime() <= 93;
    });
    public static Trigger timestampThree = new Trigger(() -> {
        return DriverStation.getMatchTime() <= 68;
    });
    public static Trigger timestampFour = new Trigger(() -> {
        return DriverStation.getMatchTime() <= 43;
    });
    public static Trigger timestampFive = new Trigger(() -> {
        return DriverStation.getMatchTime() <= 12;
    });

    /**
     * Runs the initialization routines for subsystems
     */
    public static void runSubsystemInitRoutines() {
        turret.setStartupAngle();
        // hood.setStartupAngle();
        // extender.setStartupAngle();
    }

    /**
     * Applies the default commands
     */
    public static void applyDefaultCommands() {
        // swerve.setDefaultCommand(
        // swerve.applyRequest(
        // () -> fieldCentricDrive
        // .withVelocityX(-driver.getLeftY() * MaxSpeed)
        // .withVelocityY(-driver.getLeftX() * MaxSpeed)
        // .withRotationalRate(-driver.getRightX() * MaxAngularRate)
        // )
        // );

        // hood.setDefaultCommand(hood.aim(Degrees.of(10)));
        hood.setDefaultCommand(hood.manualAim());
        // turret.setDefaultCommand(new Aim());

        shooter.setDefaultCommand(shooter.stop());
        feeder.setDefaultCommand(feeder.stop());
        spindexer.setDefaultCommand(spindexer.stop());
        // extender.setDefaultCommand(extender.stop());
        // intake.setDefaultCommand(intake.stop());
    }

    /**
     * Applies driver button bindings
     */
    public static void applyButtonBindings() {
        System.out.println("yay ggmu");
        driver.x().whileTrue(
            feeder.go()
                .alongWith(shooter.shoot())
                .alongWith(spindexer.go())
        );
        // driver.y().onTrue(hood.incrementPitch(Rotations.of(10.0)));
        // driver.a().onTrue(hood.incrementPitch(Rotations.of(-10.0)));

        // driver.povUp().whileTrue(swerve.applyRequest(() -> povStrafe.apply(0)));
        // driver.pov(45).whileTrue(swerve.applyRequest(() -> povStrafe.apply(45)));
        // driver.povRight().whileTrue(swerve.applyRequest(() -> povStrafe.apply(90)));
        // driver.pov(135).whileTrue(swerve.applyRequest(() -> povStrafe.apply(135)));
        // driver.povDown().whileTrue(swerve.applyRequest(() -> povStrafe.apply(180)));
        // driver.pov(225).whileTrue(swerve.applyRequest(() -> povStrafe.apply(225)));
        // driver.povLeft().whileTrue(swerve.applyRequest(() -> povStrafe.apply(270)));
        // driver.pov(315).whileTrue(swerve.applyRequest(() -> povStrafe.apply(315)));

        // driver.rightBumper().whileTrue(swerve.applyRequest(() -> brake));
        // driver.rightBumper().whileTrue(hood.aim());
        // driver.b().whileTrue(hood.aim(Degrees.of(10)));

        // driver.x().onTrue(Commands.runOnce(() -> turret.setStartupAngle()));

        // driver.start().and(driver.back()).onTrue(Commands.runOnce(() -> {
        // limelight.setUseMt2(false);
        // limelight.setImuMode(IMU_MODE.EXTERNAL_SEED);
        // ElasticLib.sendNotification(new Notification(NotificationLevel.WARNING, "LL", "SEED CALLED - DR"));
        // })
        // .andThen(
        // new WaitCommand(1)
        // .alongWith(swerve.stop())
        // )
        // .andThen(Commands.runOnce(() -> {
        // limelight.setUseMt2(true);
        // swerve.seedFieldCentric(
        // Rotation2d.fromDegrees(limelight.getAverageHeading())
        // .plus(Util.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg)
        // );
        // limelight.setImuMode(IMU_MODE.INTERNAL_EXTERNAL_ASSIST);
        // }))
        // );

        // // operator.b().and(operator.rightTrigger().negate()).whileTrue(hood.aim());
        // operator.b().whileTrue(spindexer.go());
        // // operator.b().whileTrue(intake.go());
        // // operator.b().and(operator.rightBumper().negate()).whileTrue(Commands.waitSeconds(1.4).andThen(extender.autoAgitate().repeatedly()));

        // // operator.x().and(operator.rightTrigger().negate()).whileTrue(hood.aim());
        // operator.x().whileTrue(spindexer.reverse());
        // // operator.x().whileTrue(intake.go());
        // // operator.x().and(operator.rightBumper().negate()).whileTrue(Commands.waitSeconds(1.4).andThen(extender.autoAgitate().repeatedly()));

        // operator.x().or(operator.b()).and(operator.y()).and(operator.rightTrigger().negate()).whileTrue(hood.aim());

        // operator.y().whileTrue(shooter.shoot());
        // operator.y().whileTrue(feeder.go());

        // operator.rightBumper().whileTrue(intake.go());
        // operator.leftBumper().whileTrue(intake.reverse());

        // operator.back().onTrue(extender.retract()).onFalse(extender.extend());

        // operator.rightTrigger().and(passingArea1)
        // .whileTrue(
        // new StaticAim(
        // Rotations.of(
        // Util.isBlueAlliance() ? 0.45 : 0.05
        // )
        // )
        // .alongWith(hood.aim(Pitch.PASSING_ANGLE))
        // .alongWith(shooter.shoot().alongWith(feeder.go()))
        // );

        // operator.rightTrigger().and(passingArea2).whileTrue(
        // new StaticAim(
        // Rotations.of(
        // Util.isBlueAlliance() ? 0.55 : 0.95
        // )
        // )
        // .alongWith(hood.aim(Pitch.PASSING_ANGLE)).alongWith(shooter.shoot().alongWith(feeder.go()))
        // );

        // operator.leftStick().onTrue(Commands.runOnce(() -> {
        // limelight.setUseMt2(false);
        // limelight.setImuMode(IMU_MODE.EXTERNAL_SEED);
        // ElasticLib.sendNotification(new Notification(NotificationLevel.WARNING, "LL", "SEED CALLED - OP"));
        // })).onFalse(Commands.runOnce(() -> {
        // limelight.setUseMt2(true);
        // swerve.seedFieldCentric(
        // Rotation2d.fromDegrees(limelight.getAverageHeading())
        // .plus(Util.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg)
        // );
        // limelight.setImuMode(IMU_MODE.INTERNAL_EXTERNAL_ASSIST);
        // }));

        // operator.leftStick().and(operator.rightStick())
        // .onTrue(Commands.runOnce(() -> limelight.resetSeed()))
        // .onFalse(Commands.runOnce(() -> limelight.setManualSeed(true)));

        // operator.povRight().onTrue(hood.incrementPitch(Degrees.of(1.0)));
        // operator.povLeft().whileTrue(autos.agitateTeleop().repeatedly());

        // operator.povUp().onTrue(shooter.incrementVelocitySetpoint(RotationsPerSecond.of(0.5)));
        // operator.povDown().onTrue(shooter.incrementVelocitySetpoint(RotationsPerSecond.of(-0.5)));
    }

    /**
     * Applies debug button bindings
     */
    public static void applyDebugBindings() {
        driver.start().onTrue(Commands.runOnce(() -> {
            limelight.setUseMt2(false);
            limelight.setImuMode(IMU_MODE.EXTERNAL_SEED);
            ElasticLib.sendNotification(new Notification(NotificationLevel.WARNING, "LL", "SEED CALLED - DR"));
        })).onFalse(Commands.runOnce(() -> {
            limelight.setUseMt2(true);
            swerve.seedFieldCentric(
                Rotation2d.fromDegrees(limelight.getAverageHeading())
                    .plus(Util.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg)
            );
            limelight.setImuMode(IMU_MODE.INTERNAL_EXTERNAL_ASSIST);
        }));

        driver.povRight().onTrue(hood.incrementPitch(Degrees.of(1.0)));
        driver.povLeft().onTrue(hood.incrementPitch(Degrees.of(-1.0)));

        driver.povUp().onTrue(shooter.incrementVelocitySetpoint(RotationsPerSecond.of(0.5)));
        driver.povDown().onTrue(shooter.incrementVelocitySetpoint(RotationsPerSecond.of(-0.5)));

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

        // driver.leftTrigger().whileTrue(
        // Commands.waitUntil(feeder.atTargetVelocity())
        // .andThen(
        // feeder.go().alongWith(
        // Commands.repeatingSequence(
        // // Run spindexer forward until feeder no longer sees a ball (ball fed out)
        // spindexer.go().until(feeder.ballDetected().negate()),
        // // Reverse until feeder no longer sees a ball (ball cleared)
        // spindexer.reverse().until(feeder.ballDetected().negate())
        // // Grace period after reversing before going forward again
        // // Commands.waitSeconds(0.15)
        // )
        // )
        // )
        // );

        driver.leftTrigger().whileTrue(spindexer.go());
        driver.leftTrigger().whileTrue(intake.go());
        driver.leftTrigger().whileTrue(Commands.waitSeconds(1.4).andThen(extender.autoAgitate().repeatedly()));

        driver.leftBumper().whileTrue(spindexer.reverse());
        driver.leftBumper().whileTrue(intake.go());
        driver.leftBumper().whileTrue(Commands.waitSeconds(1.4).andThen(extender.autoAgitate().repeatedly()));

        // driver.rightTrigger().whileTrue(shooter.manualShoot().alongWith(feeder.go()));
        // driver.rightTrigger().whileTrue(hood.manualAim());

        driver.rightTrigger().whileTrue(shooter.shoot());
        driver.rightTrigger().whileTrue(feeder.go());
        driver.rightTrigger().whileTrue(hood.aim());

        driver.rightBumper().whileTrue(intake.go());
        driver.x().onTrue(extender.retract());
        driver.b().onTrue(extender.extend());

        // driver.rightStick().toggleOnTrue(hood.aim(Degrees.of(10)));

        driver.a().onTrue(Commands.runOnce(() -> turret.setStartupAngle()));

        /**
         * Kayne's Controlssss
         *
         * driver.back().onTrue(extender.retract()).onFalse(extender.extend());
         *
         * driver.b().whileTrue(spindexer.go());
         * driver.x().whileTrue(spindexer.reverse());
         *
         * driver.y().whileTrue(shooter.manualShoot().alongWith(feeder.go()));
         * driver.y().whileTrue(hood.manualAim());
         */
    }

    /**
     * Binds subsystem triggers
     */
    public static void bindTriggers() {
        turret.isPirouetteing().whileTrue(spindexer.stop().alongWith(feeder.stop().andThen(shooter.stop())));

        // apprTrench.and(driver.rightBumper().negate()).and(operator.a().negate())
        // .whileTrue(
        // hood.aim(Degrees.of(10)).alongWith(
        // spindexer.stop().alongWith(feeder.stop())
        // )
        // );

        inAutonShootingZone.whileTrue(
            shooter.shoot()
                .until(() -> !inAutonShootingZone.getAsBoolean() && !spindexer.stopped().getAsBoolean())
        );

        inAutonReverseZone.whileTrue(
            spindexer.reverse()
        );

        inAutonShootingZone.whileTrue(
            feeder.go().alongWith(spindexer.antiJamAuto())
        );

        inAutonShootingZone.whileTrue(hood.aim());

        inAutonShootingZone.negate().whileTrue(hood.aim(Degrees.of(10)));

        timestampOne.onTrue(Commands.run(() -> {
            driver.setRumble(RumbleType.kBothRumble, 1);
        })
            .withTimeout(0.25)
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 0);
            }).withTimeout(0.25))
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 1);
            }).withTimeout(0.25))
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 0);
            }).withTimeout(0.25))
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 1);
            }).withTimeout(0.25))
            .finallyDo(() -> {
                driver.setRumble(RumbleType.kBothRumble, 0);
            })
        );

        timestampTwo.onTrue(Commands.run(() -> {
            driver.setRumble(RumbleType.kBothRumble, 1);
        })
            .withTimeout(0.25)
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 0);
            }).withTimeout(0.25))
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 1);
            }).withTimeout(0.25))
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 0);
            }).withTimeout(0.25))
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 1);
            }).withTimeout(0.25))
            .finallyDo(() -> {
                driver.setRumble(RumbleType.kBothRumble, 0);
            })
        );

        timestampThree.onTrue(Commands.run(() -> {
            driver.setRumble(RumbleType.kBothRumble, 1);
        })
            .withTimeout(0.25)
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 0);
            }).withTimeout(0.25))
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 1);
            }).withTimeout(0.25))
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 0);
            }).withTimeout(0.25))
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 1);
            }).withTimeout(0.25))
            .finallyDo(() -> {
                driver.setRumble(RumbleType.kBothRumble, 0);
            })
        );

        timestampFour.onTrue(Commands.run(() -> {
            driver.setRumble(RumbleType.kBothRumble, 1);
        })
            .withTimeout(0.25)
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 0);
            }).withTimeout(0.25))
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 1);
            }).withTimeout(0.25))
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 0);
            }).withTimeout(0.25))
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 1);
            }).withTimeout(0.25))
            .finallyDo(() -> {
                driver.setRumble(RumbleType.kBothRumble, 0);
            })
        );

        timestampFive.onTrue(Commands.run(() -> {
            driver.setRumble(RumbleType.kBothRumble, 1);
        })
            .withTimeout(0.25)
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 0);
            }).withTimeout(0.25))
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 1);
            }).withTimeout(0.25))
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 0);
            }).withTimeout(0.25))
            .andThen(Commands.run(() -> {
                driver.setRumble(RumbleType.kBothRumble, 1);
            }).withTimeout(0.25))
            .finallyDo(() -> {
                driver.setRumble(RumbleType.kBothRumble, 0);
            })
        );
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