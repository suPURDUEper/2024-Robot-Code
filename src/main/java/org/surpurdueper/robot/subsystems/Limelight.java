// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.util.VirtualSubsystem;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.surpurdueper.robot.utils.LimelightHelpers;
import org.surpurdueper.robot.utils.LimelightHelpers.LimelightResults;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;

public class Limelight extends VirtualSubsystem {

  private CommandSwerveDrivetrain drivetrain;
  private LimelightResults lastResults;

  public Limelight() {
    super();
  }

  @Override
  public void periodic() {
    maybeApplyVisionUpdate("limelight");
    maybeApplyVisionUpdate("limelight2");
  }

  
public void maybeApplyVisionUpdate(String limelightName) {
    var currentResults = maybeGetNewerVisionEstimate(lastResults, limelightName);

    if (currentResults.isPresent()) {
      var total_latency =
          currentResults.get().targetingResults.latency_capture
              + currentResults.get().targetingResults.latency_pipeline
              + currentResults.get().targetingResults.latency_jsonParse;
      var estimate_timestamp =
          currentResults.get().targetingResults.timestamp_RIOFPGA_capture - total_latency * 0.001;

      lastResults = currentResults.get();

      if (new Transform2d(
                      drivetrain.getState().Pose,
                      currentResults.get().targetingResults.getBotPose2d_wpiBlue())
                  .getTranslation()
                  .getNorm()
              > 1
          && !DriverStation.isDisabled()) {
        return;
      }

      if (lastResults.targetingResults.targets_Fiducials.length >= 2) {
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(0.1, 0.1, 0.1));
        drivetrain.addVisionMeasurement(
            lastResults.targetingResults.getBotPose2d_wpiBlue(),
            estimate_timestamp);
      } else {
        drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(1, 1, 1));
        drivetrain.addVisionMeasurement(
            lastResults.targetingResults.getBotPose2d_wpiBlue(),
            estimate_timestamp);
      }
    }
  }

  private Optional<LimelightResults> maybeGetNewerVisionEstimate(
      LimelightResults results, String limelight) {
    if (!LimelightHelpers.tableExists(limelight)) {
      return Optional.empty();
    }

    var currentResults = LimelightHelpers.getLatestResults(limelight);

    if (currentResults.targetingResults == null) {
      return Optional.empty();
    }

    if (!currentResults.targetingResults.valid) {
      return Optional.empty();
    }

    if (currentResults.targetingResults.timestamp_LIMELIGHT_publish
        <= results.targetingResults.timestamp_LIMELIGHT_publish) {
      return Optional.empty();
    }

    if (currentResults.targetingResults.botpose_wpiblue[0] == 0
        && currentResults.targetingResults.botpose_wpiblue[1] == 0
        && currentResults.targetingResults.botpose_wpiblue[2] == 0
        && currentResults.targetingResults.botpose_wpiblue[3] == 0
        && currentResults.targetingResults.botpose_wpiblue[4] == 0
        && currentResults.targetingResults.botpose_wpiblue[5] == 0) {
      return Optional.empty();
    }

    // max 4 targets, typically 1-2
    for (var fiducial : currentResults.targetingResults.targets_Fiducials) {
      if (fiducial.getRobotPose_TargetSpace().getTranslation().getNorm() > 7) {
        return Optional.empty();
      }
    }

    return Optional.of(currentResults);
  }
}
