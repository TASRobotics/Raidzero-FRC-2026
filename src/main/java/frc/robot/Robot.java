// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.Degrees;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import org.wpilib.framework.TimedRobot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private boolean alreadyEnabled;

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
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        Scheduler.getDefault().run();

        // SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        // SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
        // Superstructure.getLimelight().loop();
    }

    @Override
    public void disabledInit() {
        // Superstructure.getLimelight().setImuMode(IMU_MODE.EXTERNAL_SEED);
    }

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
        // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // if (m_autonomousCommand != null) {
        //     CommandScheduler.getInstance().schedule(m_autonomousCommand);
        // }

        // alreadyEnabled = true;
        // Elastic.initAuton();
        // Superstructure.getLimelight().setImuMode(IMU_MODE.INTERNAL_EXTERNAL_ASSIST);
        // Superstructure.getSwerve().seedFieldCentric(
        //     Rotation2d.fromDegrees(Superstructure.getLimelight().getAverageHeading())
        //         .plus(Util.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg)
        // );
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            Scheduler.getDefault().cancel(m_autonomousCommand);
        }

        alreadyEnabled = true;
        Elastic.initTeleop();
        Superstructure.getHood().setDefaultCommand(Superstructure.getHood().aim(Degrees.of(10)));
        // Superstructure.getLimelight().setImuMode(IMU_MODE.INTERNAL_EXTERNAL_ASSIST);

        // Superstructure.getSwerve().seedFieldCentric(
        //     Rotation2d.fromDegrees(Superstructure.getLimelight().getAverageHeading())
        //         .plus(Util.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg)
        // );
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {
        SignalLogger.stop();
    }

    // @Override
    // public void testInit() {
    //     CommandScheduler.getInstance().cancelAll();

    //     alreadyEnabled = true;
    // }

    // @Override
    // public void testPeriodic() {}

    // @Override
    // public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
