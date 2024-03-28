package org.surpurdueper.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Intake;
import org.surpurdueper.robot.subsystems.Limelight;
import org.surpurdueper.robot.subsystems.Shooter;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.surpurdueper.robot.utils.LimelightHelpers;

public class Autos {

  public static DoubleSupplier zero = () -> 0;

  public static Command fire(
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      Limelight limelight,
      Intake intake,
      double aimTimeSeconds) {
    return Commands.waitSeconds(aimTimeSeconds)
        .onlyWhile(
            () -> {
              return !elevator.isAtPosition()
                  || !shooter.isShooterAtSpeed()
                  || !shooterTilt.isAtPosition()
                  || LimelightHelpers.getTX("") > 1.0;
            })
        .andThen(intake.fire());
  }

  public static Command aimAndFireWithElevator(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      Limelight limelight,
      Intake intake) {
    return new AutoAutoAim(drivetrain, shooterTilt, elevator, limelight, zero, zero)
        .alongWith(fire(shooterTilt, elevator, shooter, limelight, intake, 0.75));
  }

  public static Command aimAndFireNoElevator(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      Limelight limelight,
      Intake intake) {
    return new AutoAutoAim(drivetrain, shooterTilt, elevator, limelight, zero, zero, false)
        .alongWith(fire(shooterTilt, elevator, shooter, limelight, intake, 0.75));
  }

  public static Command aimAndFireNoElevator(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      Limelight limelight,
      Intake intake,
      double aimTime) {
    return new AutoAutoAim(drivetrain, shooterTilt, elevator, limelight, zero, zero, false)
        .alongWith(fire(shooterTilt, elevator, shooter, limelight, intake, aimTime));
  }
}
