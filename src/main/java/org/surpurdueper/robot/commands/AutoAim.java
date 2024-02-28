package org.surpurdueper.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentric;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.util.AllianceFlipUtil;
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
  SwerveRequest.FieldCentricFacingAngle swerveRequest;

  public AutoAim(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier) {
    this.drivetrain = drivetrain;
    this.shooterTilt = shooterTilt;
    this.elevator = elevator;
    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
    addRequirements(drivetrain, elevator);
    speakerCenter = AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    swerveRequest = new SwerveRequest.FieldCentricFacingAngle();
    swerveRequest.HeadingController.setPID(10,0,0);
    swerveRequest.HeadingController.enableContinuousInput(-180.0, 180.0);

  }

  @Override
  public void execute() {
    double velocityX = xVelocitySupplier.getAsDouble();
    double velocityY = yVelocitySupplier.getAsDouble();
    Pose2d currentPose = drivetrain.getState().Pose;
    Rotation2d targetDirection = currentPose.getTranslation().minus(speakerCenter).getAngle();
    SmartDashboard.putNumber("Drive/TargetAngle", targetDirection.getDegrees());
    // drivetrain.setControl(swerveRequest
    //         .withVelocityX(velocityX)
    //         .withVelocityY(velocityY)
    //         .withDeadband(0.1)
    //         .withTargetDirection(targetDirection));

    // double targetDistance = currentPose.getTranslation().getDistance(speakerCenter);
    // double targetShotAngle = LookupTables.distanceToShooterAngle.get(targetDistance);
    // shooterTilt.setPositionDegrees(53.0);

    double elevatorHeight = LookupTables.elevatorShooterClearance.get(
      Units.rotationsToDegrees(shooterTilt.getPositionRotations()));
    elevator.setPositionMeters(Units.inchesToMeters(elevatorHeight));
  }

  @Override
  public void end(boolean interrupted) {
  }
}
