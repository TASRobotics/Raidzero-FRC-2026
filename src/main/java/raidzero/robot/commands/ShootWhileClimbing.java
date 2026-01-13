package raidzero.robot.commands;

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
        
        turret.setYaw(staticYaw);
    }

    @Override
    public void execute() {
        
        double currentHeight = swerve.getRobotHeightMeters(); //method for ben to make
        double verticalVelocity = swerve.getVerticalVelocityMetersPerSecond(); //method for ben to make
        
        double latencySeconds = 0.040; // Make constant? Get from NetworkTables? Physically test?
        double futureHeight = currentHeight + (verticalVelocity * 0.040);

        double basePitch = Constants.Shooter.CLIMB_PITCH_TABLE.get(futureHeight);
        double baseRPS = Constants.Shooter.CLIMB_RPS_TABLE.get(futureHeight);

        turret.setYaw(staticYaw); 
        turret.setPitch(basePitch); // May need an adjusted pitch because of lean
        shooter.setSpeed(baseRPS); // May need an adjusted RPM cause of shooter physics
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        turret.stop();
    }
}