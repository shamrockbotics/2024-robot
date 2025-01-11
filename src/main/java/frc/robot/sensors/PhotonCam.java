package frc.robot.sensors;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Swerve;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

public class PhotonCam extends SubsystemBase {
  private PhotonCamera cam;
  private boolean hasTargets;
  private AprilTagFieldLayout fieldLayout;
  private PhotonPoseEstimator photonEstimator;
  private int tagID = -1;
  private Swerve swerve;
  private PhotonPipelineResult result;
  private double lastEstTimestamp = 0;
  private Transform3d robotToCam;

  /** Represents a Photon Camera used for vision processing on the robot. */
  public PhotonCam(
      String camName, Transform3d RobotToCam, Swerve swerve, AprilTagFieldLayout fieldLayout) {
    this.fieldLayout = fieldLayout;
    cam = new PhotonCamera(camName);
    this.swerve = swerve;
    hasTargets = false;
    this.robotToCam = RobotToCam;

    photonEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);
    photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
  }

  /**
   * Updates the PhotonCam sensor data and performs vision processing. This method retrieves the
   * latest result from the camera, estimates the robot's field-relative pose, and filters and adds
   * the vision pose to the estimation.
   */
  public void update() {
    result = cam.getLatestResult();
    var visionEst = getEstimatedGlobalPose();
    SmartDashboard.putBoolean("Estimator is Present", visionEst.isPresent());
    visionEst.ifPresent(
        est -> {
          // Calculate robot's field relative pose
          if (result.getBestTarget() != null) {
            var tar = result.getBestTarget();
            Pose3d robotPose =
                PhotonUtils.estimateFieldToRobotAprilTag(
                    tar.getBestCameraToTarget(),
                    fieldLayout.getTagPose(tar.getFiducialId()).get(),
                    robotToCam);

            var estPose = est.estimatedPose.toPose2d();
            SmartDashboard.putString("Vision Estimated Pose", estPose.toString());
            SmartDashboard.putString("Test Pose", robotPose.toPose2d().toString());

            filterAndAddVisionPose(est);
          }
        });
  }

  /**
   * Returns an optional estimated global pose of the robot based on vision estimation.
   *
   * @return An optional EstimatedRobotPose object representing the estimated global pose of the
   *     robot, or an empty optional if no estimation is available.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    var visionEstimation = photonEstimator.update(cam.getLatestResult());
    double latesttimeStamp = cam.getLatestResult().getTimestampSeconds();
    boolean newResult = Math.abs(latesttimeStamp - lastEstTimestamp) > 1e-5;
    if (newResult) {
      lastEstTimestamp = latesttimeStamp;
    }
    return visionEstimation;
  }

  /**
   * Calculates the estimated standard deviations for the given estimated pose.
   *
   * @param estimatedPose The estimated pose of the robot.
   * @return The estimated standard deviations as a matrix.
   */
  public Matrix<N3, N1> getEstimatedStdDevs(Pose2d estimatedPose) {
    var estStdDevs = VecBuilder.fill(2.0, 2.9, 2.9);
    var t = cam.getLatestResult().getTargets();
    int numTags = 0;
    double avgDistance = 0;
    for (var tar : t) {
      var tagPose = photonEstimator.getFieldTags().getTagPose(tar.getFiducialId());
      if (tagPose.isEmpty()) {
        continue;
      }
      numTags++;
      avgDistance +=
          tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
    }

    if (numTags == 0) {
      return estStdDevs;
    }

    avgDistance /= numTags;
    if (numTags > 1) {
      estStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    }
    if (numTags == 1 && avgDistance > 4) {
      estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else estStdDevs = estStdDevs.times(1 + (avgDistance * avgDistance / 30));

    return estStdDevs;
  }

  /**
   * Filters and adds a vision pose to the robot's swerve drive system.
   *
   * @param pose The estimated robot pose.
   */
  public void filterAndAddVisionPose(EstimatedRobotPose pose) {
    Matrix<N3, N1> cov = VecBuilder.fill(0.4, 0.4, 0.4);

    double ambiguity = 0;
    double distance = 0;
    for (var t : pose.targetsUsed) {
      ambiguity += t.getPoseAmbiguity() / pose.targetsUsed.size();
      distance += t.getBestCameraToTarget().getTranslation().getNorm() / pose.targetsUsed.size();
    }
    // SmartDashboard.putNumber(camName + "Ambiguity", ambiguity);
    // SmartDashboard.putNumber("Distance", distance);
    if (ambiguity > 0.25 || distance > 10) {
      return;
    } else {
      // System.out.println(ambiguity + " Excellent, Excellent");
      // System.out.println(distance + " Disisgood");
    }

    /*
     * if (pose.targetsUsed.size() > 1){
     * // has multiple tags
     * double distance2 = Math.max(Math.pow(distance * 0.4, 2), 0.7);
     * cov = VecBuilder.fill(distance2, distance2, 100);
     * }
     * if (!DriverStation.isDisabled()){
     * if (pose.targetsUsed.size() == 1){
     * if (Math.abs(pose.estimatedPose.getZ()) > 1.0
     * || pose.estimatedPose
     * .minus(new Pose3d(swerve.getPose()))
     * .getTranslation()
     * .getNorm()
     * > 1.0
     * || distance > 7.0) {
     * return;}
     * }
     * }
     */
    swerve.addVisionMeasurement(pose.estimatedPose, pose.timestampSeconds, cov);
  }

  /**
   * Checks if the PhotonCam has detected any targets.
   *
   * @return true if targets are detected, false otherwise
   */
  public boolean hasTarget() {
    return hasTargets;
  }

  /**
   * Returns the tag ID of the PhotonCam.
   *
   * @return the tag ID
   */
  public int getTagID() {
    return tagID;
  }
}
