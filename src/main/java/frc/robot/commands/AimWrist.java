// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import org.photonvision.PhotonUtils;

public class AimWrist extends Command {
  /** Creates a new AimWrist. */
  private Wrist wrist;

  private Swerve swerve;
  private PhotonVision photonVision;
  private boolean isBlue;

  /**
   * A command that aims the wrist of the robot using the provided Wrist, Swerve, and PhotonVision
   * subsystems.
   */
  public AimWrist(Wrist wrist, Swerve swerve, PhotonVision photonVision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.photonVision = photonVision;
    this.swerve = swerve;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  /**
   * Initializes the AimWrist command. Sets the value of the boolean variable 'isBlue' based on the
   * alliance color.
   */
  @Override
  public void initialize() {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      isBlue = true;
    } else isBlue = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  /**
   * Executes the AimWrist command. Retrieves the target pose based on the robot's alliance color.
   * Calculates the distance to the target pose using PhotonUtils. Calculates the target wrist angle
   * based on the distance to the target pose. Holds the wrist at the calculated target wrist angle.
   */
  @Override
  public void execute() {
    Pose2d targetPose;
    if (isBlue) {
      targetPose = photonVision.getTagPose(7).get().toPose2d();
    } else {
      targetPose = photonVision.getTagPose(4).get().toPose2d();
    }
    double distanceToTarget = PhotonUtils.getDistanceToPose(swerve.getPose(), targetPose);

    double targetWrist =
        Math.toDegrees(
            Math.atan(
                (VisionConstants.SPEAKER_SCORE_HEIGHT - VisionConstants.WRIST_AXLE_HEIGHT)
                    / distanceToTarget));

    wrist.holdWrist(getShootAngle(targetWrist));
  }

  /**
   * Calculates the shoot angle based on the given angle. The shoot angle is determined by taking
   * the minimum value between the given angle and the WristConstants.SUBWOOFER constant.
   *
   * @param angle the angle to calculate the shoot angle from
   * @return the calculated shoot angle
   */
  public double getShootAngle(double angle) {
    double shootAngle = angle;
    Math.min(shootAngle, WristConstants.SUBWOOFER);
    return shootAngle;
  }

  // Called once the command ends or is interrupted.
  /**
   * Called when the command ends. Stops the wrist movement.
   *
   * @param interrupted true if the command was interrupted, false otherwise
   */
  @Override
  public void end(boolean interrupted) {
    wrist.stop();
  }
}
