package org.surpurdueper.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import org.littletonrobotics.util.AllianceFlipUtil;
import org.littletonrobotics.util.FieldConstants;
import org.surpurdueper.robot.Constants;
import org.surpurdueper.robot.Constants.LookupTables;
import org.surpurdueper.robot.commands.FieldCentricFacingFieldAngle;
import org.surpurdueper.robot.commands.FieldCentricFacingPoint;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Limelight;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoAutoAim extends Command {

  private CommandSwerveDrivetrain drivetrain;
  private ShooterTilt shooterTilt;
  private Elevator elevator;
  private Limelight limelight;
  private Translation2d speakerCenter;
  private FieldCentricFacingPoint poseAimRequest;
  private FieldCentricFacingFieldAngle limelightAimRequest;
  private boolean shouldElevatorFollow;

  public AutoAutoAim(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Limelight limelight) {
    this(drivetrain, shooterTilt, elevator, limelight, true);
  }

  public AutoAutoAim(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Limelight limelight,
      boolean shouldElevatorFollow) {
    this.drivetrain = drivetrain;
    this.shooterTilt = shooterTilt;
    this.elevator = elevator;
    this.limelight = limelight;
    this.shouldElevatorFollow = shouldElevatorFollow;
    addRequirements(drivetrain, elevator, shooterTilt);

    // Setup request to control drive always facing the speaker
    poseAimRequest = new FieldCentricFacingPoint();
    poseAimRequest.HeadingController.setPID(10, 0, 0);
    poseAimRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    limelightAimRequest = new FieldCentricFacingFieldAngle();
    limelightAimRequest.HeadingController.setPID(10, 0, .75);
    limelightAimRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    speakerCenter =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    SmartDashboard.putNumberArray(
        "Auto Aim/Speaker Center", new double[] {speakerCenter.getX(), speakerCenter.getY()});
    poseAimRequest.setPointToFace(speakerCenter);
  }

  @Override
  public void execute() {

    Optional<Rotation2d> targetLimelightAngle = limelight.getLatencyCompensatedAngleToGoal();
    Optional<Double> targetLimelightDistance = limelight.getDistanceToGoalMeters();

    if (targetLimelightAngle.isPresent()) {
      drivetrain.setControl(
          limelightAimRequest.withFieldCentricTargetDirection(targetLimelightAngle.get()));
      SmartDashboard.putNumber(
          "AutoAim/TargetDirection", limelightAimRequest.getTargetDirection().getDegrees());
    } else {
      drivetrain.setControl(poseAimRequest);
      SmartDashboard.putNumber(
          "AutoAim/TargetDirection", poseAimRequest.getTargetDirection().getDegrees());
    }

    // Use new pose estimation to set shooter angle
    double distanceToSpeakerMeters;
    if (targetLimelightDistance.isPresent()) {
      distanceToSpeakerMeters =
          targetLimelightDistance.get() + FieldConstants.subwooferToSpeakerCenter;
    } else {
      distanceToSpeakerMeters =
          drivetrain.getState().Pose.getTranslation().getDistance(speakerCenter)
              - Constants.kBumperToRobotCenter;
    }

    shooterTilt.setPositionRotations(
        LookupTables.distanceToShooterAngle.get(distanceToSpeakerMeters));

    if (shouldElevatorFollow) {
      elevator.followShooter(shooterTilt.getPositionRotations());
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setPositionMeters(0);
  }
}
