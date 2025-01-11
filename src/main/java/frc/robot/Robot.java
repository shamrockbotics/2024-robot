// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /** Initializes the robot. */
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  /**
   * This method is called periodically during the robot's operation. It runs the command scheduler
   * and prints the values from the robot container.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotContainer.printValues();
  }

  /**
   * This method is called when the robot enters the disabled state. It is used to initialize any
   * necessary resources or perform any cleanup tasks.
   */
  @Override
  public void disabledInit() {}

  /**
   * This method is called periodically while the robot is in disabled mode. It is used to perform
   * any necessary actions or updates during this period.
   */
  @Override
  public void disabledPeriodic() {}

  /** This method is called when the robot exits the disabled mode. */
  @Override
  public void disabledExit() {}

  /**
   * Initializes the autonomous mode. Retrieves the autonomous command from the robot container and
   * schedules it for execution.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This method is called periodically during the autonomous period. */
  @Override
  public void autonomousPeriodic() {}

  /** This method is called when the autonomous period ends. */
  @Override
  public void autonomousExit() {}

  /**
   * Initializes the teleoperated mode. If there is an autonomous command running, it will be
   * cancelled.
   */
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This method is called periodically during the teleoperated period. */
  @Override
  public void teleopPeriodic() {}

  /**
   * This method is called when the teleop period is ending. It is used to perform any necessary
   * cleanup or final actions before the teleop period ends.
   */
  @Override
  public void teleopExit() {}

  /** Initializes the test mode of the robot. Cancels all running commands. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  /** This method is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This method is used to handle the test exit functionality. */
  @Override
  public void testExit() {}
}
