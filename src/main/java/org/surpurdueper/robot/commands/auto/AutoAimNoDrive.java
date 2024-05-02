package org.surpurdueper.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.util.AllianceFlipUtil;
import org.littletonrobotics.util.FieldConstants;
import org.surpurdueper.robot.Constants;
import org.surpurdueper.robot.Constants.LookupTables;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoAimNoDrive extends Command {

  private CommandSwerveDrivetrain drivetrain;
  private ShooterTilt shooterTilt;
  private Elevator elevator;
  private Translation2d speakerCenter;
  private boolean shouldElevatorFollow;

  public AutoAimNoDrive(
      CommandSwerveDrivetrain drivetrain, ShooterTilt shooterTilt, Elevator elevator) {
    this(drivetrain, shooterTilt, elevator, true);
  }

  public AutoAimNoDrive(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      boolean shouldElevatorFollow) {
    this.drivetrain = drivetrain;
    this.shooterTilt = shooterTilt;
    this.elevator = elevator;
    this.shouldElevatorFollow = shouldElevatorFollow;
    addRequirements(elevator, shooterTilt);
  }

  @Override
  public void initialize() {
    speakerCenter =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
  }

  @Override
  public void execute() {

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