package raidzero.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import raidzero.robot.subsystems.turret.Shooter;
import raidzero.robot.subsystems.turret.Turret;

public class Bindings {
    private static Turret turret = Turret.system();
    private static Shooter shooter = Shooter.system();

    private static CommandXboxController driver = new CommandXboxController(0);

    public static void applyDefaultCommands() {
        turret.setDefaultCommand(turret.aim());
        shooter.setDefaultCommand(shooter.stop());

        turret.hasHubLOS().and(() -> Superstructure.hubActive).whileTrue(shooter.shoot());
    }

    public static void applyButtonBindings() {
    }
}
