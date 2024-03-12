package org.surpurdueper.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.FieldCentricFacingAngle;
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

  private static final boolean USE_LIMELIGHT = false;

  private CommandSwerveDrivetrain drivetrain;
  private ShooterTilt shooterTilt;
  private Elevator elevator;
  private Shooter shooter;
  private Limelight limelight;
  private DoubleSupplier xVelocitySupplier;
  private DoubleSupplier yVelocitySupplier;
  private Translation2d speakerCenter;
  private FieldCentricFacingPoint poseAimRequest;
  private FieldCentricFacingAngle limelightAimRequest;
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
    poseAimRequest.HeadingController.enableContinuousInput(-180.0, 180.0);

    limelightAimRequest = new FieldCentricFacingAngle();
    limelightAimRequest.HeadingController.setPID(10, 0, 0);
    limelightAimRequest.HeadingController.enableContinuousInput(-180.0, 180.0);
  }

  @Override
  public void initialize() {
    speakerCenter =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    poseAimRequest.setPointToFace(speakerCenter);
    shooter.turnOn();
  }

  @Override
  public void execute() {
    // Update drivetrain with new joystick values
    SmartDashboard.putNumber(
        "AutoAim/TargetDirection", poseAimRequest.getTargetDirection().getDegrees());
    double velocityX = xVelocitySupplier.getAsDouble();
    double velocityY = yVelocitySupplier.getAsDouble();

    Optional<Rotation2d> targetLimelightAngle = limelight.getLatencyCompensatedAngleToGoal();

    if (USE_LIMELIGHT && targetLimelightAngle.isPresent()) {
      drivetrain.setControl(
          limelightAimRequest
              .withTargetDirection(targetLimelightAngle.get())
              .withVelocityX(velocityX)
              .withVelocityY(velocityY));
    } else {
      drivetrain.setControl(
          poseAimRequest.withVelocityX(velocityX).withVelocityY(velocityY).withDeadband(0.1));
    }

    // Use new pose estimation to set shooter angle
    double distanceToSpeakerMeters =
        drivetrain.getState().Pose.getTranslation().getDistance(speakerCenter);
    shooterTilt.setPositionRotations(
        LookupTables.distanceToShooterAngle.get(distanceToSpeakerMeters));

    // shooterTilt.setPositionRotations(Units.degreesToRotations(shooterAngle.getAsDouble()));
    elevator.followShooter(shooterTilt.getPositionRotations());
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setPositionMeters(0);
    shooter.turnOnIdle();
  }
}
