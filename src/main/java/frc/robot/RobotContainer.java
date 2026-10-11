// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.wpilib.command2.Command;

public class RobotContainer {
    /**
     * Constructs a {@link RobotContainer} instance
     */
    public RobotContainer() {
        Superstructure.runSubsystemInitRoutines();
        Superstructure.applyDefaultCommands();
        Superstructure.applyButtonBindings();
        // Superstructure.applyDebugBindings();
        Superstructure.bindTriggers();
        Superstructure.configureAutons();
    }

    /**
     * Returns the selected autonomous command
     *
     * @return A {@link Command} representing the selected autonomous command
     */
    public Command getAutonomousCommand() {
        return Superstructure.getAutonomousCommand();
    }
}
