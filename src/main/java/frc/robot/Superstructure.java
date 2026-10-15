package frc.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;

import java.util.List;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathfindingCommand;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.Timer;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.POVDirection;
import org.wpilib.driverstation.RobotState;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import frc.lib.ElasticLib;
import frc.lib.Util;
import frc.lib.ElasticLib.Notification;
import frc.lib.ElasticLib.NotificationLevel;
import frc.lib.LimelightState.IMU_MODE;
import frc.robot.commands.Aim;
import frc.robot.commands.StaticAim;
import frc.robot.subsystems.drive.Limelight;
// import frc.robot.subsystems.drive.Limelight;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.intake.Extender;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Spindexer;
import frc.robot.subsystems.shooter.Feeder;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Hood;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretConstants.Pitch;

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

    private static CommandNiDsXboxController driver = new CommandNiDsXboxController(0);
    private static CommandNiDsXboxController operator = new CommandNiDsXboxController(1);

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

    // private static final Auto autos = new Auto(swerve, intake, extender);

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
        return inAllianceZone && RobotState.isAutonomousEnabled();
    });

    public static Trigger inAutonReverseZone = new Trigger(() -> {
        Pose2d pose = swerve.getState().Pose;
        boolean inAllianceZone = (pose.getX() > 3.5 && pose.getX() < 5.1) || (pose.getX() > 12.7 && pose.getX() < 11.2);
        return inAllianceZone && RobotState.isAutonomousEnabled();
    });

    // 1:55 1:30 1:05 0:40
    public static Trigger timestampOne = new Trigger(() -> {
        return Timer.getMatchTime() <= 118;
    });
    public static Trigger timestampTwo = new Trigger(() -> {
        return Timer.getMatchTime() <= 93;
    });
    public static Trigger timestampThree = new Trigger(() -> {
        return Timer.getMatchTime() <= 68;
    });
    public static Trigger timestampFour = new Trigger(() -> {
        return Timer.getMatchTime() <= 43;
    });
    public static Trigger timestampFive = new Trigger(() -> {
        return Timer.getMatchTime() <= 12;
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
        // swerve.setDefaultCommand(
        //     swerve.applyRequest(
        //         () -> fieldCentricDrive
        //             .withVelocityX(-driver.getLeftY() * MaxSpeed)
        //             .withVelocityY(-driver.getLeftX() * MaxSpeed)
        //             .withRotationalRate(-driver.getRightX() * MaxAngularRate)
        //     )
        // );

        hood.setDefaultCommand(hood.aim(Degrees.of(10)));
        // hood.setDefaultCommand(hood.manualAim());
        turret.setDefaultCommand(new Aim());

        shooter.setDefaultCommand(shooter.stop());
        feeder.setDefaultCommand(feeder.stop());
        spindexer.setDefaultCommand(spindexer.stop());
        extender.setDefaultCommand(extender.stop());
        intake.setDefaultCommand(intake.stop());
    }

    private static Command passing(double blueRotations, double redRotations) {
        return Command.noRequirements(coroutine -> {
            double rotations = Util.isBlueAlliance() ? blueRotations : redRotations;
            coroutine.awaitAll(
                new StaticAim(Rotations.of(rotations)),
                hood.aim(Pitch.PASSING_ANGLE),
                shooter.shoot(),
                feeder.go()
            );
        }).named("Passing Shot");
    }

    /**
     * Applies driver button bindings
     */
    public static void applyButtonBindings() {
        // driver.povUp().whileTrue(swerve.applyRequest(() -> povStrafe.apply(0)));
        // driver.pov(POVDirection.UP_RIGHT).whileTrue(swerve.applyRequest(() -> povStrafe.apply(45)));
        // driver.povRight().whileTrue(swerve.applyRequest(() -> povStrafe.apply(90)));
        // driver.pov(POVDirection.UP_LEFT).whileTrue(swerve.applyRequest(() -> povStrafe.apply(135)));
        // driver.povDown().whileTrue(swerve.applyRequest(() -> povStrafe.apply(180)));
        // driver.pov(POVDirection.DOWN_LEFT).whileTrue(swerve.applyRequest(() -> povStrafe.apply(225)));
        // driver.povLeft().whileTrue(swerve.applyRequest(() -> povStrafe.apply(270)));
        // driver.pov(POVDirection.DOWN_RIGHT).whileTrue(swerve.applyRequest(() -> povStrafe.apply(315)));

        // driver.rightBumper().whileTrue(swerve.applyRequest(() -> brake));
        driver.rightBumper().whileTrue(hood.aim());
        driver.b().whileTrue(hood.aim(Degrees.of(10)));

        driver.x().onTrue(
            Command.noRequirements(coroutine -> turret.setStartupAngle())
                .named("Set Startup Angle")
        );

        driver.start().and(driver.back()).onTrue(
            Command.noRequirements(coroutine -> {
                // Step 1: switch the limelight to seed mode
                limelight.setUseMt2(false);
                limelight.setImuMode(IMU_MODE.EXTERNAL_SEED);
                ElasticLib.sendNotification(
                    new Notification(NotificationLevel.WARNING, "LL", "SEED CALLED - DR"));

                // Step 2: hold the drivetrain still for 1 second
                coroutine.await(swerve.stop().withTimeout(Seconds.of(1)));

                // Step 3: seed the heading and go back to normal
                limelight.setUseMt2(true);
                swerve.seedFieldCentric(
                    Rotation2d.fromDegrees(limelight.getAverageHeading())
                        .plus(Util.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg));
                limelight.setImuMode(IMU_MODE.INTERNAL_EXTERNAL_ASSIST);
            }).named("Seed Limelight Heading")
        );

        // operator.b().and(operator.rightTrigger().negate()).whileTrue(hood.aim());
        operator.b().whileTrue(spindexer.go());
        // operator.b().whileTrue(intake.go());
        // operator.b().and(operator.rightBumper().negate()).whileTrue(Commands.waitSeconds(1.4).andThen(extender.autoAgitate().repeatedly()));

        // operator.x().and(operator.rightTrigger().negate()).whileTrue(hood.aim());
        operator.x().whileTrue(spindexer.reverse());
        // operator.x().whileTrue(intake.go());
        // operator.x().and(operator.rightBumper().negate()).whileTrue(Commands.waitSeconds(1.4).andThen(extender.autoAgitate().repeatedly()));

        operator.x().or(operator.b()).and(operator.y()).and(operator.rightTrigger().negate()).whileTrue(hood.aim());

        operator.y().whileTrue(shooter.shoot());
        operator.y().whileTrue(feeder.go());

        operator.rightBumper().whileTrue(intake.go());
        operator.leftBumper().whileTrue(intake.reverse());

        operator.back().onTrue(extender.retract()).onFalse(extender.extend());

        operator.rightTrigger().and(passingArea1).whileTrue(passing(0.45, 0.05));
        operator.rightTrigger().and(passingArea2).whileTrue(passing(0.55, 0.95));

        operator.leftStick().onTrue(
            Command.noRequirements(coroutine -> {
                limelight.setUseMt2(false);
                limelight.setImuMode(IMU_MODE.EXTERNAL_SEED);
                ElasticLib.sendNotification(
                    new Notification(NotificationLevel.WARNING, "LL", "SEED CALLED - OP"));
            }).named("Limelight Seed Start")
        );

        operator.leftStick().onFalse(
            Command.noRequirements(coroutine -> {
                limelight.setUseMt2(true);
                swerve.seedFieldCentric(
                    Rotation2d.fromDegrees(limelight.getAverageHeading())
                        .plus(Util.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg));
                limelight.setImuMode(IMU_MODE.INTERNAL_EXTERNAL_ASSIST);
            }).named("Limelight Seed End")
        );

        operator.leftStick().and(operator.rightStick()).onTrue(
            Command.noRequirements(coroutine -> limelight.resetSeed()).named("Reset Seed")
        );
        operator.leftStick().and(operator.rightStick()).onFalse(
            Command.noRequirements(coroutine -> limelight.setManualSeed(true)).named("Manual Seed")
        );

        operator.povRight().onTrue(hood.incrementPitch(Degrees.of(1.0)));
        // operator.povLeft().whileTrue(autos.agitateTeleop().repeatedly());

        operator.povUp().onTrue(shooter.incrementVelocitySetpoint(RotationsPerSecond.of(0.5)));
        operator.povDown().onTrue(shooter.incrementVelocitySetpoint(RotationsPerSecond.of(-0.5)));
    }

    private static Command agitateAfterDelay() {
        return Command.noRequirements(coroutine -> {
            coroutine.wait(Seconds.of(1.4));
            while (true) {
                coroutine.await(extender.autoAgitate());   // fresh command each pass
                coroutine.yield();
            }
        }).named("Agitate After Delay");
    }

    /**
     * Applies debug button bindings
     */
    public static void applyDebugBindings() {
        driver.start().onTrue(
            Command.noRequirements(coroutine -> {
                limelight.setUseMt2(false);
                limelight.setImuMode(IMU_MODE.EXTERNAL_SEED);
                ElasticLib.sendNotification(
                    new Notification(NotificationLevel.WARNING, "LL", "SEED CALLED - DR"));
            }).named("Limelight Seed Start")
        );

        driver.start().onFalse(
            Command.noRequirements(coroutine -> {
                limelight.setUseMt2(true);
                swerve.seedFieldCentric(
                    Rotation2d.fromDegrees(limelight.getAverageHeading())
                        .plus(Util.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg));
                limelight.setImuMode(IMU_MODE.INTERNAL_EXTERNAL_ASSIST);
            }).named("Limelight Seed End")
        );

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
        driver.leftTrigger().whileTrue(agitateAfterDelay());

        driver.leftBumper().whileTrue(spindexer.reverse());
        driver.leftBumper().whileTrue(intake.go());
        driver.leftBumper().whileTrue(agitateAfterDelay());

        // driver.rightTrigger().whileTrue(shooter.manualShoot().alongWith(feeder.go()));
        // driver.rightTrigger().whileTrue(hood.manualAim());

        driver.rightTrigger().whileTrue(shooter.shoot());
        driver.rightTrigger().whileTrue(feeder.go());
        driver.rightTrigger().whileTrue(hood.aim());

        driver.rightBumper().whileTrue(intake.go());
        driver.x().onTrue(extender.retract());
        driver.b().onTrue(extender.extend());

        // driver.rightStick().toggleOnTrue(hood.aim(Degrees.of(10)));

        driver.a().onTrue(
            Command.noRequirements(coroutine -> turret.setStartupAngle())
                .named("Set Startup Angle")
        );

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

    private static Command rumblePulses() {
        return Command.noRequirements(coroutine -> {
            for (int i = 0; i < 5; i++) {
                driver.setRumble(RumbleType.LEFT_RUMBLE, i % 2 == 0 ? 1 : 0);   // 1,0,1,0,1
                driver.setRumble(RumbleType.RIGHT_RUMBLE, i % 2 == 0 ? 1 : 0);   // 1,0,1,0,1
                coroutine.wait(Seconds.of(0.25));
            }
            driver.setRumble(RumbleType.LEFT_RUMBLE, 0);
            driver.setRumble(RumbleType.RIGHT_RUMBLE, 0);
        })
            .whenCanceled(() -> {
                driver.setRumble(RumbleType.LEFT_RUMBLE, 0);
                driver.setRumble(RumbleType.RIGHT_RUMBLE, 0);
            })
            .named("Rumble Pulses");
    }

    /**
     * Binds subsystem triggers
     */
    public static void bindTriggers() {
        turret.isPirouetteing().whileTrue(
            Command.noRequirements(coroutine ->
                coroutine.awaitAll(
                    spindexer.stop(),
                    feeder.stop(),
                    shooter.stop()
                )
            ).named("Stop Shooting Systems")
        );

        // apprTrench.and(driver.rightBumper().negate()).and(operator.a().negate())
        // .whileTrue(
        // hood.aim(Degrees.of(10)).alongWith(
        // spindexer.stop().alongWith(feeder.stop())
        // )
        // );

        inAutonShootingZone.whileTrue(
            shooter.shoot()
                .until(() -> !inAutonShootingZone.getAsBoolean() && !spindexer.stopped().getAsBoolean())
                .named("Shooter shoot until clear")
        );

        inAutonReverseZone.whileTrue(spindexer.reverse());

        inAutonShootingZone.whileTrue(
            feeder.go()
                .alongWith(spindexer.antiJamAuto())
                .named("Feed And Antijam")
        );

        inAutonShootingZone.whileTrue(hood.aim());
        inAutonShootingZone.negate().whileTrue(hood.aim(Degrees.of(10)));

        for (Trigger timestamp : List.of(
            timestampOne, timestampTwo, timestampThree, timestampFour, timestampFive)) {
            timestamp.onTrue(rumblePulses());        // a fresh command per trigger
        }
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
        // CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
        // autoChooser = AutoBuilder.buildAutoChooser();
        // autoChooser.addOption("Depot single", autos.getDepot1Cycle());
        // autoChooser.addOption("Left Double", autos.getLeft2Cycle());
        // autoChooser.addOption("Right Oupost", autos.getRightOutpostAuto());
        // autoChooser.addOption("Right Double", autos.getRight2Cycle());
        // autoChooser.addOption("Right Outpost Double", autos.getRightOutpostDoubleAuto());

        // SmartDashboard.putData("Auto Chooser", autoChooser);
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