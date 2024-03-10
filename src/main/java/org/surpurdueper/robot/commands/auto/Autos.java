package org.surpurdueper.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.Supplier;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Intake;
import org.surpurdueper.robot.subsystems.ShooterTilt;

public class Autos {

  public static Command aimAndFireWithElevator(
      ShooterTilt shooterTilt,
      Elevator elevator,
      Intake intake,
      Supplier<Pose2d> robotPoseSupplier) {
    return Commands.deadline(
        Commands.sequence(
            shooterTilt.goToShotAngleBlocking(robotPoseSupplier),
            elevator.waitUntilAtPosition(),
            intake.fire().withTimeout(0.3)),
        elevator.followShooter(shooterTilt::getPositionRotations));
  }

  public static Command aimAndFireNoElevator(
      ShooterTilt shooterTilt, Intake intake, Supplier<Pose2d> robotPoseSupplier) {
    return Commands.sequence(
        shooterTilt.goToShotAngleBlocking(robotPoseSupplier), intake.fire().withTimeout(0.3));
  }
}
