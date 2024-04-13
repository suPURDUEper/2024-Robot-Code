package org.surpurdueper.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.ForwardReference;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.util.AllianceFlipUtil;
import org.littletonrobotics.util.FieldConstants;
import org.littletonrobotics.util.LoggedTunableNumber;
import org.surpurdueper.robot.Constants;
import org.surpurdueper.robot.Constants.LookupTables;
import org.surpurdueper.robot.Constants.TiltConstants;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Shooter;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoAim extends Command {

  private CommandSwerveDrivetrain drivetrain;
  private ShooterTilt shooterTilt;
  private Elevator elevator;
  private Shooter shooter;
  private DoubleSupplier xVelocitySupplier;
  private DoubleSupplier yVelocitySupplier;
  private Translation2d speakerCenter;
  private Translation2d feedShotTarget;
  private FieldCentricFacingPoint poseAimRequest;
  private LoggedTunableNumber shooterAngle =
      new LoggedTunableNumber("ShooterTilt/AutoAim Angle", 30);

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
    addRequirements(drivetrain, elevator, shooter, shooterTilt);

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
    feedShotTarget = AllianceFlipUtil.apply(FieldConstants.feedShotLocation);
  }

  @Override
  public void execute() {
    // Update drivetrain with new joystick values
    double velocityX = xVelocitySupplier.getAsDouble();
    double velocityY = yVelocitySupplier.getAsDouble();
    Pose2d robotPose = drivetrain.getState().Pose;

    if (AllianceFlipUtil.apply(robotPose.getX()) < FieldConstants.fieldLength / 2) {
      poseAimRequest.setPointToFace(speakerCenter);
      shooter.turnOn();
      // Use new pose estimation to set shooter angle
      double distanceToSpeakerMeters =
          drivetrain.getState().Pose.getTranslation().getDistance(speakerCenter)
              - Constants.kBumperToRobotCenter;
      shooterTilt.setPositionRotations(
          LookupTables.distanceToShooterAngle.get(distanceToSpeakerMeters));
      SmartDashboard.putNumber(
          "AutoAim/Distance to Speaker (in)", Units.metersToInches(distanceToSpeakerMeters));
    } else {
      poseAimRequest.setPointToFace(feedShotTarget);
      shooter.turnOnFeedShot();
      shooterTilt.setPositionRotations(TiltConstants.kFeedShot);
    }
    elevator.followShooter(shooterTilt.getPositionRotations());
    drivetrain.setControl(poseAimRequest.withVelocityX(velocityX).withVelocityY(velocityY));
    SmartDashboard.putNumber(
        "AutoAim/TargetDirection", poseAimRequest.getTargetDirection().getDegrees());
  }

  @Override
  public void end(boolean interrupted) {
    elevator.setPositionMeters(0);
    shooter.turnOnIdle();
  }
}
