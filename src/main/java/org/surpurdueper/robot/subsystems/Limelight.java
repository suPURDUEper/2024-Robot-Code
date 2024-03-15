// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot.subsystems;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import org.littletonrobotics.util.FieldConstants;
import org.littletonrobotics.util.VirtualSubsystem;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.surpurdueper.robot.utils.LimelightHelpers;

public class Limelight extends VirtualSubsystem {

  // https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
  private static final double a1 = Units.degreesToRadians(15.70); // Camera Lens Angle
  private static final double h1 = Units.inchesToMeters(10.566051); // Camera Lens Height
  private static final double h2 = Units.inchesToMeters(57.13); // Apriltag height

  private static double getDistance(double a2) {
    return (h2 - h1) / Math.tan(a1 + a2);
    // (57.13 - 10.566) / Math.tan(1.10 + a2) = 154.5
  }

  private static final double cameraToRobotCenterMeters = 0.1;

  private CommandSwerveDrivetrain drivetrain;
  private static final double kBufferDuration = 1.5;
  private TimeInterpolatableBuffer<Rotation2d> robotAngleBuffer;

  /** Creates a new LimeLight. */
  public Limelight(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    robotAngleBuffer = TimeInterpolatableBuffer.createBuffer(kBufferDuration);
  }

  @Override
  public void periodic() {
    // Update gyro angle buffer
    robotAngleBuffer.addSample(
        MathSharedStore.getTimestamp(), drivetrain.getState().Pose.getRotation());
    updatePose2DAprilTag();
  }

  public Optional<Double> getDistanceToGoalMeters() {
    if (!LimelightHelpers.getTV("")) {
      return Optional.empty();
    }
    double lensToTagDistanceMeters = getDistance(Units.degreesToRadians(LimelightHelpers.getTY("")));

    return Optional.of(lensToTagDistanceMeters + Units.inchesToMeters(5));
  }

  public Optional<Rotation2d> getLatencyCompensatedAngleToGoal() {
    if (!LimelightHelpers.getTV("")) {
      return Optional.empty();
    }
    Rotation2d tx = Rotation2d.fromDegrees(LimelightHelpers.getTX(""));
    double latencyMs =
        LimelightHelpers.getLatency_Capture("") + LimelightHelpers.getLatency_Pipeline("");
    if (latencyMs / 1000.0 > kBufferDuration) {
      return Optional.empty();
    }
    double timestamp = MathSharedStore.getTimestamp() - latencyMs / 1000.0;
    Optional<Rotation2d> robotAngleOptional = robotAngleBuffer.getSample(timestamp);
    if (robotAngleOptional.isEmpty()) {
      return Optional.empty();
    }
    Rotation2d angleToGoal = robotAngleOptional.get().minus(tx);
    return Optional.of(angleToGoal);
  }

  private void updatePose2DAprilTag() {
    // To do any processing we need to know what alliance we are on and if we have a valid target
    if (DriverStation.getAlliance().isEmpty() || !LimelightHelpers.getTV("")) {
      return;
    }
    double ty = LimelightHelpers.getTY("");
    double tx = LimelightHelpers.getTX("");
    double latencyMs =
        LimelightHelpers.getLatency_Capture("") + LimelightHelpers.getLatency_Pipeline("");

    if (latencyMs / 1000.0 > kBufferDuration) {
      return;
    }

    double timestamp = MathSharedStore.getTimestamp() - latencyMs / 1000.0;
    // Calculate how far we are from the apriltag
    double distance = getDistance(Units.degreesToRadians(ty));
    SmartDashboard.putNumber("Vision/Distance (in)", Units.metersToInches(distance));

    // Calculate the angle from the apriltag to the camera. Account for the delay in the camera
    // capture and processing
    Optional<Rotation2d> robotAngleOptional = robotAngleBuffer.getSample(timestamp);
    Rotation2d robotAngle =
        robotAngleOptional.orElseGet(() -> drivetrain.getState().Pose.getRotation());
    Rotation2d angle = robotAngle.minus(Rotation2d.fromDegrees(tx));
    SmartDashboard.putNumber("Vision/AngleToGoal (deg)", angle.getDegrees());

    // Make a vector using the previous distance and angle and calculate the camera position
    Translation2d apriltag =
        (DriverStation.getAlliance().get() == Alliance.Red)
            ? FieldConstants.aprilTag4
            : FieldConstants.aprilTag7;
    Translation2d tagToCameraTranslation = new Translation2d(distance, angle);
    Translation2d cameraToRobotCenterTranslation =
        new Translation2d(cameraToRobotCenterMeters, robotAngle);
    Translation2d robot =
        apriltag.plus(tagToCameraTranslation).plus(cameraToRobotCenterTranslation);
    Pose2d poseEstimate = new Pose2d(robot, robotAngle);
    SmartDashboard.putNumberArray(
        "Vision/Pose Estimate",
        new double[] {
          poseEstimate.getX(), poseEstimate.getY(), poseEstimate.getRotation().getRotations()
        });
    // drivetrain.addVisionMeasurement(poseEstimate, timestamp);
  }
}
