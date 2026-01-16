package raidzero.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.drive.TunerConstants;
import raidzero.robot.subsystems.turret.Shooter;
import raidzero.robot.subsystems.turret.Turret;

public class Superstructure {
   private static Turret turret = Turret.system();
   private static Swerve swerve = Swerve.system();
   private static Shooter shooter = Shooter.system();

   private static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
   private static double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

   private static final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric()
         .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
         .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

   private static final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

   private static CommandXboxController driver = new CommandXboxController(0);

   public static Trigger driverWantsControl = driver.axisMagnitudeGreaterThan(0, 0.1) // left x
         .or(driver.axisMagnitudeGreaterThan(4, 0.1)) // right x
         .or(driver.axisMagnitudeGreaterThan(1, .1)) // left y
         .or(driver.axisMagnitudeGreaterThan(5, 0.1)); // right y

   private static CommandGenericHID operator = new CommandGenericHID(1);

   public static Trigger hoardingToggle = operator.button(0);
   public static Trigger shootingToggle = operator.button(1);

   public static Trigger hubActive = new Trigger(() -> false); // placeholder value until repo is upgraded to 2026

   public static Trigger CAN_SHOOT = turret.hasHubLOS().and(hubActive).and(swerve.inShootingArea()).and(shootingToggle);
   public static Trigger HOARDING_MODE = turret.hasHubLOS().negate().and(swerve.inShootingArea().negate())
         .and(hoardingToggle);

   public static void runSubsystemInitRoutines() {
      turret.setStartupAngle();
   }

   public static void applyDefaultCommands() {
      turret.setDefaultCommand(turret.aim());
      shooter.setDefaultCommand(shooter.stop());
      swerve.setDefaultCommand(swerve.applyRequest(() -> brake));

      CAN_SHOOT.whileTrue(shooter.shoot());
   }

   public static void applyButtonBindings() {
      driverWantsControl.whileTrue(swerve.applyRequest(
            () -> fieldCentricDrive
                  .withVelocityX(-driver.getLeftY() * MaxSpeed)
                  .withVelocityY(-driver.getLeftX() * MaxSpeed)
                  .withRotationalRate(-driver.getRightX() * MaxAngularRate)));
   }
}
