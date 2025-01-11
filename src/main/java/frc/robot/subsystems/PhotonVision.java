// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.sensors.PhotonCam;
import java.io.IOException;
import java.util.Optional;

/**
 * Represents the PhotonVision subsystem, which handles vision processing and tracking using Photon
 * cameras.
 */
public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  private final PhotonCam frontPhoton;

  private final PhotonCam rightPhoton;
  private final PhotonCam leftPhoton;
  private AprilTagFieldLayout fieldLayout;

  public PhotonVision(Swerve swerve) {
    try {
      fieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    } catch (IOException e) {
      e.printStackTrace();
    }

    frontPhoton =
        new PhotonCam(
            VisionConstants.frontCamName, VisionConstants.frontRobotToCam, swerve, fieldLayout);

    rightPhoton =
        new PhotonCam(
            VisionConstants.rightCamName, VisionConstants.rightRobotToCam, swerve, fieldLayout);

    leftPhoton =
        new PhotonCam(
            VisionConstants.leftCamName, VisionConstants.rightRobotToCam, swerve, fieldLayout);
  }

  /**
   * Retrieves the pose (position and orientation) of a specific tag identified by its ID.
   *
   * @param id The ID of the tag.
   * @return An Optional containing the pose of the tag if it is found, or an empty Optional if the
   *     tag is not found.
   */
  public Optional<Pose3d> getTagPose(int id) {
    return fieldLayout.getTagPose(id);
  }

  /** Updates the front, right, and left PhotonVision cameras. */
  @Override
  public void periodic() {
    frontPhoton.update();
    rightPhoton.update();
    leftPhoton.update();
  }
}
