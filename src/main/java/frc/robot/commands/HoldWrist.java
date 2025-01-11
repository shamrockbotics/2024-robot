// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class HoldWrist extends Command {
  /**
   * Creates a new HoldWrist command.
   *
   * @param wrist the Wrist subsystem to be used by the command
   */
  double target;

  Wrist wrist;

  /**
   * A command that holds the wrist in its current position. This command requires the Wrist
   * subsystem.
   */
  public HoldWrist(Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  /**
   * Initializes the HoldWrist command. Sets the target position of the wrist to the current
   * position.
   */
  @Override
  public void initialize() {
    target = wrist.getDegrees();
    wrist.setTarget(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Executes the HoldWrist command. Retrieves the target position for the wrist and holds the wrist
   * at that position.
   */
  @Override
  public void execute() {
    target = wrist.getTarget();
    wrist.holdWrist(target);
  }
}
