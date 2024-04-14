// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.util.VirtualSubsystem;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.surpurdueper.robot.utils.LimelightHelpers;

public class Limelight extends VirtualSubsystem {

  private CommandSwerveDrivetrain drivetrain;
  private int[] validIDs = {3, 4, 7, 8};
  private double[] visionPose = new double[3];

  /** Creates a new LimeLight. */
  public Limelight(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    updatePose3dAprilTag();
  }

  private void updatePose3dAprilTag() {
    boolean doRejectUpdate = false;
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
    LimelightHelpers.SetRobotOrientation(
        "limelight", drivetrain.getState().Pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    // if our angular velocity is greater than 720 degrees per second, ignore vision updates
    if (Math.abs(drivetrain.getState().speeds.omegaRadiansPerSecond)
        > Units.degreesToRadians(720)) {
      doRejectUpdate = true;
    }
    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      drivetrain.setVisionMeasurementStdDevs(VecBuilder.fill(.6, .6, 9999999));
      drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
    }
    visionPose[0] = mt2.pose.getX();
    visionPose[1] = mt2.pose.getY();
    visionPose[2] = mt2.pose.getRotation().getDegrees();
    SmartDashboard.putNumberArray("Vision/Pose (mt2)", visionPose);
  }
}
