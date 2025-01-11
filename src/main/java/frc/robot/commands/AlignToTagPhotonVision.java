// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CommandConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Swerve;

public class AlignToTagPhotonVision extends Command {
  /** Creates a new AlignToTagPhotonVision. */
  Swerve swerve;

  Transform3d camToTarget;

  double xError;
  double yError;
  double thetaError;
  boolean xFinished;
  boolean yFinished;
  boolean thetaFinished;

  /**
   * Constructs a new AlignToTagPhotonVision command.
   *
   * @param swerve the Swerve subsystem to align with the tag
   */
  public AlignToTagPhotonVision(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  /** Initializes the AlignToTagPhotonVision command. */
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Executes the alignment to tag PhotonVision command. Calculates the errors in x, y, and theta
   * based on the camera's position relative to the target. Drives the swerve drive system to align
   * with the target based on the calculated errors. Updates the status of xFinished, yFinished, and
   * thetaFinished based on error tolerances.
   */
  @Override
  public void execute() {
    // camToTarget = vision.getCameraToTarget();
    // xError = CommandConstants.xGoal - camToTarget.getX();
    // yError = CommandConstants.yGoal - camToTarget.getY();
    // thetaError = camToTarget.getRotation().getAngle();

    swerve.drive(
        new Translation2d(-xError, -yError).times(DriveConstants.MaxSpeed),
        thetaError,
        false,
        true,
        false,
        false);

    if (Math.abs(xError) < CommandConstants.errorTolerence) {
      xFinished = true;
    } else xFinished = false;
    if (Math.abs(yError) < CommandConstants.errorTolerence) {
      yFinished = true;
    } else yFinished = false;
    if (Math.abs(thetaError) < CommandConstants.errorTolerence) {
      thetaFinished = true;
    } else thetaFinished = false;
  }

  // Called once the command ends or is interrupted.
  /**
   * Called when the command ends, either by finishing successfully or being interrupted.
   *
   * @param interrupted true if the command was interrupted, false otherwise
   */
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  /**
   * Checks if the alignment to the target is finished.
   *
   * @return true if the alignment is finished, false otherwise.
   */
  @Override
  public boolean isFinished() {
    return xFinished && yFinished && thetaFinished;
  }
}
