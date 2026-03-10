// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidzero.robot;

import com.ctre.phoenix6.HootAutoReplay;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.ironmaple.simulation.SimulatedArena;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private boolean alreadyEnabled;

    private final StructArrayPublisher<Pose3d> ballLog = NetworkTableInstance.getDefault()
        .getStructArrayTopic("Sim/BallStream", Pose3d.struct).publish();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();

        Elastic.init();
        alreadyEnabled = false;
    }

    @Override
    public void robotInit() {
        if (isSimulation()) {
            SimulatedArena.getInstance().clearGamePieces();
        }
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        Superstructure.getLimelight().loop();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        if (!alreadyEnabled) {
            Elastic.checkForAlliance();
        }
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        alreadyEnabled = true;
        Elastic.initAuton();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        alreadyEnabled = true;
        Elastic.initTeleop();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();

        alreadyEnabled = true;
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

   @Override
    public void simulationPeriodic() {
        // Advance the physics engine
        SimulatedArena.getInstance().simulationPeriodic();

        // Push the balls to the 3D view
        Pose3d[] balls = SimulatedArena.getInstance().getGamePiecesArrayByType("Fuel");
        ballLog.set(balls);
    }
}
