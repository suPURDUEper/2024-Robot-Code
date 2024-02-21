package org.surpurdueper.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.util.FieldConstants;
import org.surpurdueper.robot.Constants.LookupTables;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoAim extends Command {
  CommandSwerveDrivetrain drivetrain;
  ShooterTilt shooterTilt;
  Elevator elevator;
  DoubleSupplier xVelocitySupplier;
  DoubleSupplier yVelocitySupplier;
  Translation2d speakerCenter;

  public AutoAim(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier) {
    this.drivetrain = drivetrain;
    this.shooterTilt = shooterTilt;
    this.elevator = elevator;
    addRequirements(drivetrain, shooterTilt, elevator);
    speakerCenter = FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d();
  }

  @Override
  public void execute() {
    double velocityX = xVelocitySupplier.getAsDouble();
    double velocityY = yVelocitySupplier.getAsDouble();
    Pose2d currentPose = drivetrain.getState().Pose;
    Rotation2d targetDirection = currentPose.getTranslation().minus(speakerCenter).getAngle();
    SwerveRequest swerveRequest =
        new SwerveRequest.FieldCentricFacingAngle()
            .withVelocityX(velocityX)
            .withVelocityY(velocityY)
            .withTargetDirection(targetDirection);
    drivetrain.setControl(swerveRequest);

    double targetDistance = currentPose.getTranslation().getDistance(speakerCenter);
    double targetShotAngle = LookupTables.distanceToShooterAngle.get(targetDistance);
    shooterTilt.setPositionDegrees(targetShotAngle);

    double elevatorHeight = LookupTables.elevatorShooterClearance.get(targetShotAngle);
    // elevator.setHeight(elevatorHeight);
  }

  @Override
  public void end(boolean interrupted) {
    // elevator.setHeight(0);
  }
}
