package org.surpurdueper.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The SwerveRequest::apply function runs in a fast (250hz on CAN FD) thread that is timed on the
 * CTRE StatusSignal API. This is the same thread that updates odometry. By extending this request
 * to set the target angle to face the speaker instead of just using
 * SwerveRequest.FieldCentricFacingAngle, we can ensure that we're updating the target position at
 * 250hz and always utilizing the latest pose estimation.
 */
public class FieldCentricFacingPoint extends SwerveRequest.FieldCentricFacingAngle {

  Translation2d pointToFace;

  public void setPointToFace(Translation2d point) {
    pointToFace = point;
  }

  public Rotation2d getTargetDirection() {
    return this.TargetDirection;
  }

  @Override
  public StatusCode apply(
      SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
    this.TargetDirection = parameters.currentPose.getTranslation().minus(pointToFace).getAngle();
    return super.apply(parameters, modulesToApply);
  }

  @Override
  public FieldCentricFacingAngle withTargetDirection(Rotation2d targetDirection) {
    // Ignore this decorator, since we want to update this at 250hz in apply instead
    // of at 50hz in the main robot loop
    return this;
  }

  public FieldCentricFacingAngle withPointToFace(Translation2d point) {
    setPointToFace(point);
    return this;
  }
}
