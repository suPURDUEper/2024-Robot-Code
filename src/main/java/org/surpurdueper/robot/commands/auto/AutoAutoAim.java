package org.surpurdueper.robot.commands.auto;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.util.AllianceFlipUtil;
import org.littletonrobotics.util.FieldConstants;
import org.surpurdueper.robot.Constants;
import org.surpurdueper.robot.Constants.LookupTables;
import org.surpurdueper.robot.commands.FieldCentricFacingPoint;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoAutoAim extends Command {

  private CommandSwerveDrivetrain drivetrain;
  private ShooterTilt shooterTilt;
  private Elevator elevator;
  private Translation2d speakerCenter;
  private FieldCentricFacingPoint poseAimRequest;
  private boolean shouldElevatorFollow;

  public AutoAutoAim(
      CommandSwerveDrivetrain drivetrain, ShooterTilt shooterTilt, Elevator elevator) {
    this(drivetrain, shooterTilt, elevator, true);
  }

  public AutoAutoAim(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      boolean shouldElevatorFollow) {
    this.drivetrain = drivetrain;
    this.shooterTilt = shooterTilt;
    this.elevator = elevator;
    this.shouldElevatorFollow = shouldElevatorFollow;
    addRequirements(drivetrain, elevator, shooterTilt);

    // Setup request to control drive always facing the speaker
    poseAimRequest = new FieldCentricFacingPoint();
    poseAimRequest.ForwardReference = ForwardReference.RedAlliance;
    poseAimRequest.HeadingController.setPID(10, 0, .75);
    poseAimRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    speakerCenter =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    poseAimRequest.setPointToFace(speakerCenter);
  }

  @Override
  public void execute() {
    drivetrain.setControl(poseAimRequest);
    SmartDashboard.putNumber(
        "AutoAim/TargetDirection", poseAimRequest.getTargetDirection().getDegrees());
    // Use new pose estimation to set shooter angle
    double distanceToSpeakerMeters =
        drivetrain.getState().Pose.getTranslation().getDistance(speakerCenter)
            - Constants.kBumperToRobotCenter;
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
