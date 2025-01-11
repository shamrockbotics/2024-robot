// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class AutoShoot extends Command {
  /** Creates a new AutoIntake. */
  Intake intake;

  Feeder feeder;
  Wrist wrist;

  int count = 0;
  int feedRan = 0;
  boolean shotNote = false;

  /**
   * A command that automates the shooting process in autonomous mode. This command requires the
   * Intake, Feeder, and Wrist subsystems.
   */
  public AutoShoot(Intake intake, Feeder feeder, Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.feeder = feeder;
    this.wrist = wrist;
    addRequirements(intake, wrist, feeder);
  }

  // Called when the command is initially scheduled.
  /** Initializes the AutoShoot command. Resets the shotNote, count, and feedRan variables. */
  @Override
  public void initialize() {
    shotNote = false;
    count = 0;
    feedRan = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Executes the AutoShoot command. If the wrist angle is within 1.5 degrees of the desired angle
   * (WristConstants.SUBWOOFER), and the intake RPM is greater than or equal to the shoot RPM
   * (IntakeConstants.shootRPM), the feeder speed is set to FeederConstants.feedSpeed and the
   * feedRan counter is incremented. The intake speed is set to IntakeConstants.shootSpeed, and the
   * wrist is held at the desired angle. If the wrist angle is not within the desired range, the
   * wrist is held at the desired angle, the feeder speed is set to 0, and the intake speed is set
   * to IntakeConstants.shootSpeed. If the feedRan counter is greater than 15 and the feeder is not
   * holding a note, the shotNote flag is set to true.
   */
  @Override
  public void execute() {
    if (Math.abs(WristConstants.SUBWOOFER - wrist.getDegrees()) < 1.5) {
      if (intake.getRPM() >= IntakeConstants.shootRPM) {
        feeder.setSpeed(FeederConstants.feedSpeed);
        feedRan++;
      }
      intake.setSpeed(IntakeConstants.shootSpeed);
      wrist.holdWrist(WristConstants.SUBWOOFER);
    } else {
      wrist.holdWrist(WristConstants.SUBWOOFER);
      feeder.setSpeed(0);
      intake.setSpeed(IntakeConstants.shootSpeed);
    }

    /*
     * if (feeder.holdingNote() == false){
     * count++;
     * }
     */

    if (feedRan > 15 && !feeder.holdingNote()) {
      shotNote = true;
    }
  }

  // Called once the command ends or is interrupted.
  /**
   * This method is called when the command ends, either by finishing or being interrupted. It stops
   * the wrist, feeder, and intake subsystems.
   *
   * @param interrupted true if the command was interrupted, false otherwise
   */
  @Override
  public void end(boolean interrupted) {
    wrist.stop();
    ;
    feeder.stop();
    intake.stop();
  }

  // Returns true when the command should end.
  /**
   * Checks if the shooting operation is finished.
   *
   * @return true if the shooting operation is finished, false otherwise
   */
  @Override
  public boolean isFinished() {
    return shotNote;
  }
}
