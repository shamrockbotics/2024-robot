// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Wrist;

public class SetPosition extends InstantCommand {
  /** Creates a new SetPosition. */
  Wrist wrist;

  double rotation;
  boolean done;

  /**
   * A command that sets the position of the wrist subsystem.
   *
   * @param wrist The wrist subsystem.
   * @param rotation The desired rotation for the wrist.
   */
  public SetPosition(Wrist wrist, double rotation) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.wrist = wrist;
    this.rotation = rotation;
    addRequirements(wrist);
    done = false;
  }

  // Called when the command is initially scheduled.
  /** Initializes the command. */
  @Override
  public void initialize() {
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Executes the command to set the position of the wrist. It compares the current position of the
   * wrist with the desired rotation angle and adjusts it accordingly. If the current position is
   * between the pickup angle and the stow angle, it sets the rotation angle to the pickup angle. If
   * the current position is between the stow angle and the desired rotation angle, it sets the
   * rotation angle to the stow angle. It then holds the wrist at the adjusted rotation angle.
   * Finally, it checks if the difference between the adjusted rotation angle and the current
   * position is less than 0.1 degrees, and sets the 'done' flag to true if it is.
   */
  @Override
  public void execute() {
    double currentValue = wrist.getDegrees();

    if (rotation < currentValue && currentValue < WristConstants.PICKUP) {
      rotation = WristConstants.PICKUP;
    } else if (rotation > currentValue && currentValue > WristConstants.STOW) {
      rotation = WristConstants.STOW;
    }

    wrist.holdWrist(rotation);
    if (Math.abs(rotation - currentValue) < 0.1) {
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  /**
   * This method is called when the command ends. It stops the wrist movement.
   *
   * @param interrupted true if the command was interrupted, false otherwise
   */
  @Override
  public void end(boolean interrupted) {
    wrist.stop();
  }

  // Returns true when the command should end.
  /**
   * Determines whether the command has finished executing.
   *
   * @return true if the command has finished, false otherwise
   */
  @Override
  public boolean isFinished() {
    return done;
  }
}
