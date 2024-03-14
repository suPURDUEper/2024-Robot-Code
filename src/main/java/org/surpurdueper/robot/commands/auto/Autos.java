package org.surpurdueper.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.surpurdueper.robot.commands.AutoAim;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Intake;
import org.surpurdueper.robot.subsystems.Limelight;
import org.surpurdueper.robot.subsystems.Shooter;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class Autos {

  public static DoubleSupplier zero = () -> 0;

  public static Command aimAndFireWithElevator(CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      Limelight limelight,
      Intake intake) {
    return new AutoAutoAim(drivetrain, shooterTilt, elevator, limelight, zero, zero)
    .alongWith(Commands.waitSeconds(0.75).andThen(intake.fire()));
  }

  public static Command aimAndFireNoElevator(CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      Shooter shooter,
      Limelight limelight,
      Intake intake) {
    return new AutoAutoAim(drivetrain, shooterTilt, elevator, limelight, zero, zero, false)
    .alongWith(Commands.waitSeconds(0.75).andThen(intake.fire()));
  }
}
