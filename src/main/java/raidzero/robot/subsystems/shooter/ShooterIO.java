package raidzero.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import raidzero.lib.SubsystemIO;
import raidzero.lib.wrappers.motors.LazyTalon;
import raidzero.lib.wrappers.motors.LazyTalonBuilder;
import raidzero.robot.Superstructure;
import raidzero.robot.subsystems.drive.Swerve;
import raidzero.robot.subsystems.shooter.ShooterConstants.Flywheel;
import raidzero.robot.subsystems.turret.Hood;
import raidzero.robot.subsystems.turret.Turret;
import raidzero.robot.subsystems.turret.TurretConstants.Kinetics;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

public interface ShooterIO extends SubsystemIO {

    /**
     * Sets the shooter velocity to the specified speed
     *
     * @param velocity the {@link AngularVelocity}
     */
    void setVelocity(AngularVelocity velocity);

    /**
     * Gets the flywheel's current velocity
     *
     * @return the {@link Velocity}
     */
    AngularVelocity getVelocity();

    /**
     * Sets the shooter to a speed as a percentage
     *
     * @param speedPercent the percent [0, 1]
     */
    void setPercent(double speedPercent);

    /**
     * Stops the flywheel
     */
    void stop();

    public class Real implements ShooterIO {
        LazyTalon flywheel;

        Real() {
            flywheel = new LazyTalonBuilder(
                    Flywheel.MOTOR_ID, Superstructure.canDoAttitude,
                    Flywheel.SENSOR_TO_MECHANISM_RATIO, Flywheel.INVERTED,
                    Flywheel.STATOR_CURRENT_LIMIT.in(Amps),
                    Flywheel.SUPPLY_CURRENT_LIMIT.in(Amps))
                    .withPIDFConfiguration(
                            Flywheel.P, Flywheel.I, Flywheel.D,
                            Flywheel.S, Flywheel.G, Flywheel.V, Flywheel.A,
                            Flywheel.FF_GRAVITY_TYPE, Flywheel.FF_SIGN, Flywheel.VELOCITY_SLOT)
                    .withMotionMagicConfiguration(
                            0, 0, Flywheel.CRUISE_VELOCITY,
                            Flywheel.ACCELERATION)
                    .withFollower(Flywheel.FOLLOW_ID, Superstructure.canDoAttitude, Flywheel.FOLLOWER_INVERTED)
                    .build();

            flywheel.setCoast();
        }

        @Override
        public void setVelocity(AngularVelocity velocity) {
            this.flywheel.setPIDVelocityTarget(velocity, 0);
        }

        @Override
        public void setPercent(double speedPercent) {
            flywheel.setDutyCycle(speedPercent);
        }

        @Override
        public void stop() {
            flywheel.stop();
        }

        @Override
        public AngularVelocity getVelocity() {
            return flywheel.getVelocity();
        }

        @Override
        public void updateTelemetry() {
            flywheel.updateTelemetry(Flywheel.TELEMETRY);
        }

    }

    public class Sim extends Real {
        private AngularVelocity simVelocity = RotationsPerSecond.of(0);
        private int shotCounter = 0;

        @Override
        public void setVelocity(AngularVelocity velocity) {
            this.simVelocity = velocity;
        }

        @Override
        public AngularVelocity getVelocity() {
            return simVelocity;
        }

        @Override
        public void stop() {
            this.simVelocity = RotationsPerSecond.of(0);
        }

        @Override
        public void updateTelemetry() {
            AngularVelocity rps = getVelocity();
            
            if (rps.compareTo(RotationsPerSecond.of(10)) > 0) { // Lowered threshold for testing
                if (++shotCounter % 10 == 0) {
                    System.out.println("DEBUG: Spawning Maple Ball at " + rps + " RPS");
                    spawnMapleBall(rps);
                }
            }
        }

        /**
         * Spawns a ball into the IronMaple engine using real subsystem states.
         * * @param flywheelVelocity The current velocity of the flywheel
         */
        public void spawnMapleBall(AngularVelocity flywheelVelocity) {
            Pose2d robotPose = Swerve.system().getState().Pose;
            ChassisSpeeds robotSpeeds = Swerve.system().getState().Speeds;
            Rotation2d turretYaw = new Rotation2d(Turret.system().getCurrentYaw());
            
            // 1. Non-linear Mapping
            // Robot 34 -> Physics 0
            // Robot 10 -> Physics 80
            double robotHoodDegrees = Hood.system().getAngle().in(Degrees);
            double physicsPitchDegrees = (90.0 - robotHoodDegrees);
            
            // Clamp to ensure we don't shoot backwards or into the floor if the hood overtravels
            physicsPitchDegrees = Math.max(0, Math.min(90, physicsPitchDegrees));
            Angle finalPhysicsPitch = Degrees.of(physicsPitchDegrees);

            double WHEEL_DIAMETER_METERS = Inches.of(3.125).in(Meters); // 3.125 inches in meters
            Distance RELEASE_HEIGHT = Inches.of(18.5); // 18.5 inches in meters

            // 2. Velocity Calculation
            double wheelCircumference = WHEEL_DIAMETER_METERS * Math.PI;
            double launchSpeedMps = flywheelVelocity.in(RotationsPerSecond) * wheelCircumference;


            // 3. Projectile Construction
            RebuiltFuelOnFly fuel = new RebuiltFuelOnFly(
                robotPose.getTranslation(),          
                Kinetics.TURRET_MOUNT_OFFSET,        
                robotSpeeds,                         
                robotPose.getRotation().plus(turretYaw), 
                RELEASE_HEIGHT,                      
                MetersPerSecond.of(launchSpeedMps),  
                finalPhysicsPitch 
            );

            SimulatedArena.getInstance().addGamePieceProjectile(fuel);
            
            SmartDashboard.putNumber("Shooter/SimPhysicsPitch", physicsPitchDegrees);
        }
    }
}