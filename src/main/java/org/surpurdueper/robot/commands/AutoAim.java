package org.surpurdueper.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.util.AllianceFlipUtil;
import org.littletonrobotics.util.FieldConstants;
import org.littletonrobotics.util.LoggedTunableNumber;
import org.surpurdueper.robot.Constants;
import org.surpurdueper.robot.Constants.LookupTables;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Limelight;
import org.surpurdueper.robot.subsystems.Shooter;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoAim extends Command {

  private static final boolean USE_LIMELIGHT = true;

  private CommandSwerveDrivetrain drivetrain;
  private ShooterTilt shooterTilt;
  private Elevator elevator;
  private Shooter shooter;
  private Limelight limelight;
  private DoubleSupplier xVelocitySupplier;
  private DoubleSupplier yVelocitySupplier;
  private Translation2d speakerCenter;
  private FieldCentricFacingPoint poseAimRequest;
  private FieldCentricFacingFieldAngle limelightAimRequest;
  private LoggedTunableNumber shooterAngle =
      new LoggedTunableNumber("ShooterTilt/AutoAim Angle", 30);

  private boolean shouldElevatorFollow;
  private double distanceToSpeakerMeters = -1;
  private Rotation2d lastSeenLimelightAngle = null;

  public AutoAim(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      Limelight limelight,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier) {
    this(
        drivetrain,
        shooterTilt,
        elevator,
        shooter,
        limelight,
        xVelocitySupplier,
        yVelocitySupplier,
        true);
  }

  public AutoAim(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      Limelight limelight,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier,
      boolean shouldElevatorFollow) {
    this.drivetrain = drivetrain;
    this.shooterTilt = shooterTilt;
    this.elevator = elevator;
    this.shooter = shooter;
    this.limelight = limelight;
    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
    this.shouldElevatorFollow = shouldElevatorFollow;
    addRequirements(drivetrain, elevator, shooter, shooterTilt);

    // Setup request to control drive always facing the speaker
    poseAimRequest = new FieldCentricFacingPoint();
    poseAimRequest.HeadingController.setPID(10, 0, 0);
    poseAimRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    limelightAimRequest = new FieldCentricFacingFieldAngle();
    limelightAimRequest.HeadingController.setPID(5, 0, 0);
    limelightAimRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    speakerCenter =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    SmartDashboard.putNumberArray(
        "Auto Aim/Speaker Center", new double[] {speakerCenter.getX(), speakerCenter.getY()});
    poseAimRequest.setPointToFace(speakerCenter);
    shooter.turnOn();
  }

  @Override
  public void execute() {
    // Update drivetrain with new joystick values
    double velocityX = xVelocitySupplier.getAsDouble();
    double velocityY = yVelocitySupplier.getAsDouble();

    Optional<Rotation2d> targetLimelightAngle = limelight.getLatencyCompensatedAngleToGoal();
    Optional<Double> targetLimelightDistance = limelight.getDistanceToGoalMeters();

    if (targetLimelightAngle.isPresent()) {
      lastSeenLimelightAngle = Rotation2d.fromDegrees(targetLimelightAngle.get().getDegrees());
    }

    if (USE_LIMELIGHT) {
      if (lastSeenLimelightAngle == null) {
        lastSeenLimelightAngle = drivetrain.getState().Pose.getRotation();
      }
      drivetrain.setControl(
          limelightAimRequest
              .withFieldCentricTargetDirection(lastSeenLimelightAngle)
              .withVelocityX(velocityX)
              .withVelocityY(velocityY));
      SmartDashboard.putNumber(
          "AutoAim/TargetDirection", limelightAimRequest.getTargetDirection().getDegrees());
    }

    // Use new pose estimation to set shooter angle
    if (USE_LIMELIGHT && targetLimelightDistance.isPresent()) {
      distanceToSpeakerMeters =
          targetLimelightDistance.get() + FieldConstants.subwooferToSpeakerCenter;
    } else if (distanceToSpeakerMeters < 0) {
      distanceToSpeakerMeters =
          drivetrain.getState().Pose.getTranslation().getDistance(speakerCenter)
              - Constants.kBumperToRobotCenter;
    }

    if (distanceToSpeakerMeters > 0) {
      shooterTilt.setPositionRotations(
          LookupTables.distanceToShooterAngle.get(distanceToSpeakerMeters));
    }

    // shooterTilt.setPositionRotations(Units.degreesToRotations(shooterAngle.getAsDouble()));
    if (shouldElevatorFollow) {
      elevator.followShooter(shooterTilt.getPositionRotations());
    }
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setPositionMeters(0);
    shooter.turnOnIdle();
  }
}
