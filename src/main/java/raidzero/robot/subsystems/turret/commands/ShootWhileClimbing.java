package raidzero.robot.subsystems.turret.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import raidzero.robot.Constants;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.turret.Shooter;
import raidzero.robot.subsystems.turret.Turret;
import raidzero.robot.subsystems.turret.TurretConstants;
import raidzero.robot.subsystems.turret.TurretConstants.Limelight;

public class ShootWhileClimbing extends Command {

    private Swerve swerve = Swerve.system();
    private Turret turret = Turret.system();
    private Shooter shooter = Shooter.system();
    private Translation2d actualGoal;
    private Rotation2d staticYaw;

    public ShootWhileClimbing() {
        addRequirements(Shooter.system(), Turret.system());
    }

    // Hi please ignore the red lines it should get fixed when theo merges

    @Override
    public void initialize() {
        // Goal
        actualGoal = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red 
            ? Constants.Field.GOAL_LOCATION_RED 
            : Constants.Field.GOAL_LOCATION_BLUE;

        // 2. Lock the Yaw based on current field position
        Pose2d currentPose = swerve.getState().Pose;
        staticYaw = actualGoal.minus(currentPose.getTranslation()).getAngle();
        
        turret.setYaw(staticYaw.getMeasure());
    }

    @Override
    public void execute() {
        
        double currentHeight = swerve.getRobotHeightMeters(); //method for ben to make
        double verticalVelocity = swerve.getVerticalVelocityMetersPerSecond(); //method for ben to make
        
        double futureHeight = currentHeight + (verticalVelocity * Limelight.LATENCY.in(Seconds));

        double basePitch = TurretConstants.Setpoints.CLIMB_PITCH_TABLE.get(futureHeight);
        double baseRPS = TurretConstants.Setpoints.CLIMB_RPS_TABLE.get(futureHeight);

        turret.setYaw(staticYaw.getMeasure()); 
        turret.setPitch(Degrees.of(basePitch)); // May need an adjusted pitch because of lean
        shooter.setVelocity(RotationsPerSecond.of(baseRPS)); // May need an adjusted RPM cause of shooter physics
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        turret.stop();
    }
}