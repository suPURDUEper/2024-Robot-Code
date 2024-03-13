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

  public AutoAim(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      Limelight limelight,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier) {
    this.drivetrain = drivetrain;
    this.shooterTilt = shooterTilt;
    this.elevator = elevator;
    this.shooter = shooter;
    this.limelight = limelight;
    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
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

    if (USE_LIMELIGHT && targetLimelightAngle.isPresent()) {
      drivetrain.setControl(
          limelightAimRequest
              .withFieldCentricTargetDirection(targetLimelightAngle.get())
              .withVelocityX(velocityX)
              .withVelocityY(velocityY));
      SmartDashboard.putNumber(
          "AutoAim/TargetDirection", limelightAimRequest.getTargetDirection().getDegrees());
    } else {
      drivetrain.setControl(
          poseAimRequest.withVelocityX(velocityX).withVelocityY(velocityY).withDeadband(0.1));
      SmartDashboard.putNumber(
          "AutoAim/TargetDirection", poseAimRequest.getTargetDirection().getDegrees());
    }

    // Use new pose estimation to set shooter angle

    double distanceToSpeakerMeters;
    if (USE_LIMELIGHT && targetLimelightDistance.isPresent()) {
      distanceToSpeakerMeters = targetLimelightDistance.get();
    } else {
      distanceToSpeakerMeters =
          drivetrain.getState().Pose.getTranslation().getDistance(speakerCenter);
    }
    shooterTilt.setPositionRotations(
        LookupTables.distanceToShooterAngle.get(distanceToSpeakerMeters));
    elevator.followShooter(shooterTilt.getPositionRotations());
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setPositionMeters(0);
    shooter.turnOnIdle();
  }
}
