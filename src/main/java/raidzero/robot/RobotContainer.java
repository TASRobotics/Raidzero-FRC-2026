// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidzero.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import raidzero.robot.subsystems.turret.Turret;

public class RobotContainer {
  private static Turret turret = Turret.system();

  public RobotContainer() {
    Bindings.applyDefaultCommands();
    Bindings.applyButtonBindings();

    turret.setStartupAngle();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
