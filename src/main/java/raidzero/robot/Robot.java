// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package raidzero.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import raidzero.lib.LimelightState.IMU_MODE;
import raidzero.lib.LimelightState.PIPELINE;
import raidzero.lib.Util;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private boolean alreadyEnabled;

    private double lastSentTA = -1.0;

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
        CommandScheduler.getInstance().run();

        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("Voltage", RobotController.getBatteryVoltage());
        Superstructure.getLimelight().loop();
    }

    @Override
    public void disabledInit() {
        Superstructure.getLimelight().setImuMode(IMU_MODE.EXTERNAL_SEED);
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
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        alreadyEnabled = true;
        Elastic.initAuton();
        Superstructure.getLimelight().setImuMode(IMU_MODE.INTERNAL_EXTERNAL_ASSIST);
        Superstructure.getSwerve().seedFieldCentric(
            Rotation2d.fromDegrees(Superstructure.getLimelight().getAverageHeading())
                .plus(Util.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg)
        );
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
        Superstructure.getLimelight().setImuMode(IMU_MODE.INTERNAL_EXTERNAL_ASSIST);

        Superstructure.getSwerve().seedFieldCentric(
            Rotation2d.fromDegrees(Superstructure.getLimelight().getAverageHeading())
                .plus(Util.isBlueAlliance() ? Rotation2d.kZero : Rotation2d.k180deg)
        );
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
    public void simulationInit() {
        NetworkTableInstance.getDefault()
            .getTable("limelight-front")
            .getEntry("ta")
            .setDouble(15.0);
    }

    @Override
    public void simulationPeriodic() {
        double selectedTA = RobotContainer.getSimTA();

        // Only update NetworkTables if the value has actually changed
        if (selectedTA != lastSentTA) {
            NetworkTableInstance.getDefault()
                .getTable("limelight-front")
                .getEntry("ta")
                .setDouble(selectedTA);

            // Update the tracker so we don't send it again next loop
            lastSentTA = selectedTA;
        }
    }
}
