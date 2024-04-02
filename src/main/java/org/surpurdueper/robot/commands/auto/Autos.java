package org.surpurdueper.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Intake;
import org.surpurdueper.robot.subsystems.Limelight;
import org.surpurdueper.robot.subsystems.Shooter;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.surpurdueper.robot.utils.LimelightHelpers;

public class Autos {

  public static final double fireTimeSeconds = 0.2;

  public static Command fire(
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      Limelight limelight,
      Intake intake,
      double aimTimeSeconds) {
    return Commands.waitSeconds(aimTimeSeconds - fireTimeSeconds)
        .onlyWhile(
            () -> {
              return !elevator.isAtPosition()
                  || !shooter.isShooterAtSpeed()
                  || !shooterTilt.isAtPosition()
                  || LimelightHelpers.getTX("") > 1.0;
            })
        .andThen(intake.fire().withTimeout(fireTimeSeconds));
  }

  public static Command aimAndFireWithElevator(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      Limelight limelight,
      Intake intake) {
    return aimAndFireIfDisk(
        drivetrain, shooterTilt, elevator, shooter, limelight, intake, true, 1);
  }

  public static Command aimAndFireNoElevator(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      Limelight limelight,
      Intake intake) {
    return aimAndFireNoElevator(drivetrain, shooterTilt, elevator, shooter, limelight, intake, 1);
  }

  public static Command aimAndFireNoElevator(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      Limelight limelight,
      Intake intake,
      double aimTime) {
    return aimAndFireIfDisk(
        drivetrain, shooterTilt, elevator, shooter, limelight, intake, false, aimTime);
  }

  public static Command aimAndFireIfDisk(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      Limelight limelight,
      Intake intake,
      boolean shouldElevatorFollow,
      double aimTime) {

    return Commands.either(
        aimAndFire(
            drivetrain,
            shooterTilt,
            elevator,
            shooter,
            limelight,
            intake,
            shouldElevatorFollow,
            aimTime),
        Commands.none(),
        intake::hasDisk);
  }

  public static Command aimAndFire(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      Limelight limelight,
      Intake intake,
      boolean shouldElevatorFollow,
      double aimTime) {

    return new ParallelDeadlineGroup(
        fire(shooterTilt, elevator, shooter, limelight, intake, aimTime),
        new AutoAutoAim(drivetrain, shooterTilt, elevator, limelight, shouldElevatorFollow));
  }
}
