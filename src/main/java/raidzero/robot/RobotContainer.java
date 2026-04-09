// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidzero.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

    private static final SendableChooser<Double> taSimChooser = new SendableChooser<>();

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
        this.initializeTAChooser();
    }

    private void initializeTAChooser(){
        if (Robot.isSimulation()) {
            taSimChooser.setDefaultOption("15.0 (Far/Idle)", 15.0);
            taSimChooser.addOption("11.0 (Low Agitate)", 11.0);
            taSimChooser.addOption("9.0 (Mid Agitate)", 9.0);
            taSimChooser.addOption("7.0 (High Agitate)", 7.0);
            taSimChooser.addOption("0.0 (Ball Gone)", 0.0);

            SmartDashboard.putData("Sim TA Setter", taSimChooser);
        }
    }

    public static double getSimTA() {
        return taSimChooser.getSelected();
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
