// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidzero.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import raidzero.lib.LimelightState.PIPELINE;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private boolean alreadyEnabled;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        Logger.recordMetadata("ProjectName", "Raidzero-FRC-2026");

        if (isReal()) {
            // Log to a USB stick if one is plugged in, otherwise to the roboRIO's own storage
            Logger.addDataReceiver(new WPILOGWriter(new File("/U").canWrite() ? "/U/logs" : "/home/lvuser/logs"));
        } else {
            Logger.addDataReceiver(new WPILOGWriter());
        }

        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();

        m_robotContainer = new RobotContainer();

        Elastic.init();
        alreadyEnabled = false;
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
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
        Superstructure.getLimelight().onEnable();
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
        Superstructure.getHood().setDefaultCommand(Superstructure.getHood().aim(Degrees.of(10)));
        Superstructure.getLimelight().onEnable();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        SignalLogger.stop();
    }

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
    public void simulationPeriodic() {}
}
