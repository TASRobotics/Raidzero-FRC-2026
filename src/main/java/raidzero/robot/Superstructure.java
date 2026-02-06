package raidzero.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import raidzero.robot.subsystems.drive.Limelight;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.drive.TunerConstants;
import raidzero.robot.subsystems.turret.Indexer;
import raidzero.robot.subsystems.turret.Shooter;
import raidzero.robot.subsystems.turret.Turret;
import raidzero.robot.subsystems.turret.commands.Aim;

public class Superstructure { 
      private static Turret turret = Turret.system();
      private static Shooter shooter = Shooter.system();
      private static Swerve swerve = Swerve.system();
      private static Indexer indexer = Indexer.system();
      private static Limelight limelight = Limelight.system();

      private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.25;
      private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond) * 0.5;

      private static final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
                  .withDeadband(MaxSpeed * 0.05).withRotationalDeadband(MaxAngularRate * 0.05)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

      // private static final SwerveRequest.SwerveDriveBrake brake = new
      // SwerveRequest.SwerveDriveBrake();

      private static CommandXboxController driver = new CommandXboxController(0);

      // public static Trigger driverWantsControl = driver.axisMagnitudeGreaterThan(0,
      // 0.1) // left x
      // .or(driver.axisMagnitudeGreaterThan(4, 0.1)) // right x
      // .or(driver.axisMagnitudeGreaterThan(1, .1)) // left y
      // .or(driver.axisMagnitudeGreaterThan(5, 0.1)); // right y

      // private static CommandGenericHID operator = new CommandGenericHID(1);

      // public static Trigger hoardingToggle = operator.button(0);
      // public static Trigger shootingToggle = operator.button(1);

      // public static Trigger hubActive = new Trigger(() -> false); // placeholder
      // value until repo is upgraded to 2026

      // public static Trigger CAN_SHOOT =
      // turret.hasHubLOS().and(hubActive).and(swerve.inShootingArea()).and(shootingToggle);
      // public static Trigger HOARDING_MODE =
      // turret.hasHubLOS().negate().and(swerve.inShootingArea().negate())
      // .and(hoardingToggle);

      public static void runSubsystemInitRoutines() {
            turret.setStartupAngle();
      }

      public static void applyDefaultCommands() {
            // turret.setDefaultCommand(turret.aim());
            shooter.setDefaultCommand(shooter.stop());
            // swerve.setDefaultCommand(swerve.applyRequest(() -> brake));

            // CAN_SHOOT.whileTrue(shooter.shoot());
            swerve.setDefaultCommand(swerve.applyRequest(
                        () -> fieldCentricDrive
                                    .withVelocityX(-driver.getLeftY() * MaxSpeed)
                                    .withVelocityY(-driver.getLeftX() * MaxSpeed)
                                    .withRotationalRate(-driver.getRightX() * MaxAngularRate)));

            indexer.setDefaultCommand(indexer.stop());

            turret.setDefaultCommand(new Aim());
      }

      public static void applyButtonBindings() {
            // driverWantsControl.whileTrue(swerve.applyRequest(
            // () -> fieldCentricDrive
            // .withVelocityX(-driver.getLeftY() * MaxSpeed)
            // .withVelocityY(-driver.getLeftX() * MaxSpeed)
            // .withRotationalRate(-driver.getRightX() * MaxAngularRate)));
            // }

            driver.a().whileTrue(indexer.go(RotationsPerSecond.of(45)));
            driver.y().whileTrue(turret.stop());
            driver.b().whileTrue(shooter.manualShoot());
            driver.x().whileTrue(indexer.go(RotationsPerSecond.of(-45)));
            driver.start().onTrue(swerve.runOnce(swerve::seedFieldCentric));
            driver.rightTrigger().whileTrue(shooter.shoot());


            driver.rightBumper().whileTrue(turret.moveSimple(-0.1));
            driver.leftBumper().whileTrue(turret.moveSimple(0.1));

            driver.povUp().onTrue(shooter.incrementVelocitySetpoint(RotationsPerSecond.of(1)));
            driver.povDown().onTrue(shooter.incrementVelocitySetpoint(RotationsPerSecond.of(-1)));
      }

      public static void periodic() {

      }
}