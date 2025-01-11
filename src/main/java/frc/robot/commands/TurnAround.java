// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve;

public class TurnAround extends Command {
  /** Creates a new TurnAround. */
  Swerve swerve;

  boolean done = false;
  double target;
  ;

  /**
   * A command that instructs the robot to perform a turn around maneuver. This command requires the
   * Swerve subsystem.
   */
  public TurnAround(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  /**
   * Initializes the TurnAround command. Sets the initial values for the 'done' flag and the
   * 'target' angle. The 'target' angle is calculated by adding 180 degrees to the current yaw angle
   * of the swerve subsystem. If the 'target' angle is greater than 180 degrees, it is adjusted by
   * subtracting 360 degrees.
   */
  @Override
  public void initialize() {
    done = false;
    target = swerve.getYaw() + 180;
    if (target > 180) {
      target -= 360;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Executes the TurnAround command. Uses a PID controller to rotate the robot to a target angle.
   * If the rotation error is below a threshold, the rotation is set to 0 and the command is
   * considered done.
   */
  @Override
  public void execute() {

    try (PIDController rotController = new PIDController(0.1, 0, 0)) {
      rotController.enableContinuousInput(DriveConstants.MinAngle, DriveConstants.MaxAngle);

      double rotate = rotController.calculate(swerve.getYaw(), target);
      System.out.println(rotate);

      if (rotate < 0.02) {
        rotate = 0;
        done = true;
      }
      swerve.drive(new Translation2d(0, 0), -rotate, false, true, false, false);
    }
  }

  // Called once the command ends or is interrupted.
  /**
   * Called when the command ends, whether it is interrupted or finishes normally.
   *
   * @param interrupted true if the command was interrupted, false if it finished normally
   */
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  /**
   * Checks if the turn operation is finished.
   *
   * @return true if the turn operation is finished, false otherwise.
   */
  @Override
  public boolean isFinished() {
    return done;
  }
}
