// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class TeleopIntakeFeed extends Command {
  /** Creates a new TeleopIntake. */
  String status;

  Intake intake;
  Feeder feeder;
  boolean hasNote;
  Wrist wrist;
  int count = 0;

  /**
   * Represents a command for teleoperated intake feeding. This command is responsible for
   * controlling the intake, feeder, and wrist subsystems.
   *
   * @param intake The intake subsystem.
   * @param feeder The feeder subsystem.
   * @param status The status of the command.
   * @param wrist The wrist subsystem.
   */
  public TeleopIntakeFeed(Intake intake, Feeder feeder, String status, Wrist wrist) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.feeder = feeder;
    this.status = status;
    this.wrist = wrist;
    addRequirements(intake, feeder);
    count = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Executes the teleop intake feed command based on the current status. The status determines the
   * behavior of the intake and feeder subsystems.
   *
   * <p>Possible status values: - "INTAKE": Intakes power cells into the robot. - "EXTAKE": Extakes
   * power cells out of the robot. - "SHOOT": Shoots power cells from the robot. - "AMP": Activates
   * the intake and feeder at amp speed. - "STOP": Stops the intake and feeder. - "MANUAL": Sets the
   * intake and feeder to manual control mode.
   */
  @Override
  public void execute() {
    switch (status) {
      case "INTAKE":
        if (wrist.getDegrees() > WristConstants.INTAKEGOOD) {
          intake.setSpeed(IntakeConstants.intakeSpeed);
        } else if (!intake.hasNote()) {
          intake.setSpeed(IntakeConstants.intakeSpeed);
        } else intake.stop();

        if (feeder.holdingNote()) {
          feeder.stop();
        } else {
          feeder.setSpeed(FeederConstants.intakeSpeed);
          count = 0;
        }
        break;
      case "EXTAKE":
        intake.setSpeed(IntakeConstants.extakeSpeed);
        feeder.setSpeed(IntakeConstants.extakeSpeed);
        break;
      case "SHOOT":
        if (intake.getRPM() >= IntakeConstants.shootRPM) {
          feeder.setSpeed(FeederConstants.feedSpeed);
        }
        intake.setSpeed(IntakeConstants.shootSpeed);
        break;
      case "AMP":
        intake.setSpeed(IntakeConstants.ampSpeed);
        feeder.setSpeed(FeederConstants.ampSpeed);
        break;
      case "STOP":
        intake.stop();
        feeder.stop();
        break;
      case "MANUAL":
        intake.setSpeed(IntakeConstants.shootSpeed);
        feeder.setSpeed(FeederConstants.feedSpeed);

      default:
        intake.stop();
        feeder.stop();
        break;
    }
  }

  // Called once the command ends or is interrupted.
  /**
   * This method is called when the command ends. It stops the intake and feeder subsystems.
   *
   * @param interrupted true if the command was interrupted, false otherwise
   */
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    feeder.stop();
  }
}
