package org.surpurdueper.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.util.AllianceFlipUtil;
import org.littletonrobotics.util.FieldConstants;
import org.littletonrobotics.util.LoggedTunableNumber;
import org.surpurdueper.robot.Constants.LookupTables;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Shooter;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoAim extends Command {
  CommandSwerveDrivetrain drivetrain;
  ShooterTilt shooterTilt;
  Elevator elevator;
  private Shooter shooter;
  DoubleSupplier xVelocitySupplier;
  DoubleSupplier yVelocitySupplier;
  Translation2d speakerCenter;
  FieldCentricFacingPoint swerveRequest;
  LoggedTunableNumber shooterAngle = new LoggedTunableNumber("ShooterTilt/AutoAim Angle", 30);

  public AutoAim(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier) {
    this.drivetrain = drivetrain;
    this.shooterTilt = shooterTilt;
    this.elevator = elevator;
    this.shooter = shooter;
    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
    addRequirements(drivetrain, elevator, shooter); // TODO: Add shooterTilt back to this

    // Setup request to control drive always facing the speaker
    swerveRequest = new FieldCentricAutoAim();
    swerveRequest.HeadingController.setPID(10, 0, 0);
    swerveRequest.HeadingController.enableContinuousInput(-180.0, 180.0);
  }

  @Override
  public void initialize() {
    speakerCenter =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    swerveRequest.setPointToFace(speakerCenter);
    // elevatorSyncThread.start();
    shooter.turnOn();
  }

  @Override
  public void execute() {
    // Update drivetrain with new joystick values
    SmartDashboard.putNumber(
        "AutoAim/TargetDirection", swerveRequest.getTargetDirection().getDegrees());
    double velocityX = xVelocitySupplier.getAsDouble();
    double velocityY = yVelocitySupplier.getAsDouble();
    drivetrain.setControl(
        swerveRequest.withVelocityX(velocityX).withVelocityY(velocityY).withDeadband(0.1));

    // shooterTilt.setPositionRotations(Units.degreesToRotations(shooterAngle.getAsDouble()));
    double elevatorHeight =
        LookupTables.elevatorShooterClearance.get(shooterTilt.getPositionRotations());
    elevator.setPositionMeters(elevatorHeight);
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setPositionMeters(0);
    shooter.turnOnIdle();
  }

  public class FieldCentricAutoAim extends FieldCentricFacingPoint {

    @Override
    public StatusCode apply(
        SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
      StatusCode status = super.apply(parameters, modulesToApply);

      // Use new pose estimation to set shooter angle
      double distanceToSpeakerMeters =
          parameters.currentPose.getTranslation().getDistance(speakerCenter);
      shooterTilt.setPositionRotations(
          LookupTables.distanceToShooterAngle.get(distanceToSpeakerMeters));

      return status;
    }
  }
}
