package org.surpurdueper.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.util.VirtualSubsystem;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.surpurdueper.robot.utils.LimelightHelpers;
import org.surpurdueper.robot.utils.LimelightHelpers.LimelightResults;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class Photonvision extends VirtualSubsystem {

    private CommandSwerveDrivetrain drivetrain;
    PhotonPoseEstimator poseEstimator;
    EstimatedRobotPose lastResults;

    public Photonvision() {
        super();
        AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
        //Forward Camera
        PhotonCamera cam = new PhotonCamera("testCamera");
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));

        // Construct PhotonPoseEstimator
        poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam, robotToCam);

    }

    @Override
    public void periodic() {
        maybeApplyVisionUpdate(poseEstimator);
    }

    public void maybeApplyVisionUpdate(PhotonPoseEstimator poseEstimator) {
    var currentResults = maybeGetNewerVisionEstimate(poseEstimator);

    if (currentResults.isPresent()) {
      var estimate_timestamp = currentResults.get().timestampSeconds;
      lastResults = currentResults.get();
      if (new Transform2d(
                      drivetrain.getState().Pose,
                      currentResults.get().estimatedPose.toPose2d())
                  .getTranslation()
                  .getNorm()
              > 1
          && !DriverStation.isDisabled()) {
        return;
      }

      if (lastResults.targetsUsed.size() >= 2) {
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1, 0.1));
        drivetrain.addVisionMeasurement(
            lastResults.estimatedPose.toPose2d(),
            estimate_timestamp);
      } else {
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 1));
        drivetrain.addVisionMeasurement(
            lastResults.estimatedPose.toPose2d(),
            estimate_timestamp);
      }
    }
  }

  private Optional<EstimatedRobotPose> maybeGetNewerVisionEstimate(PhotonPoseEstimator estimator) {
    var currentResultsOptional = estimator.update();

    if (currentResultsOptional.isEmpty()) {
        return Optional.empty();
    }

    var currentResults = currentResultsOptional.get();

    if (currentResults.estimatedPose == null) {
      return Optional.empty();
    }

    if (currentResults.timestampSeconds < 0 ) {
      return Optional.empty();
    }

    if (currentResults.timestampSeconds
        <= lastResults.timestampSeconds) {
      return Optional.empty();
    }

    if (currentResults.estimatedPose.getX() == 0
        && currentResults.estimatedPose.getY() == 0
        && currentResults.estimatedPose.getZ() == 0
        && currentResults.estimatedPose.getRotation().getX() == 0
        && currentResults.estimatedPose.getRotation().getY() == 0
        && currentResults.estimatedPose.getRotation().getZ() == 0) {
      return Optional.empty();
    }

    // max 4 targets, typically 1-2
    for (var fiducial : currentResults.targetsUsed) {
      if (fiducial.getBestCameraToTarget().getTranslation().getNorm() > 7) {
        return Optional.empty();
      }
    }

    return Optional.of(currentResults);
  }
    
}
