package org.surpurdueper.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import org.surpurdueper.robot.Constants.TiltConstants;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Intake;
import org.surpurdueper.robot.subsystems.Shooter;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class TwoDisk extends ParallelCommandGroup {

  public TwoDisk(
      CommandSwerveDrivetrain drivetrain,
      Intake intake,
      ShooterTilt shooterTilt,
      Shooter shooter,
      Elevator elevator) {

    PathPlannerPath toFirstDisk = PathPlannerPath.fromChoreoTrajectory("5 Standard.1");

    addCommands(
        Commands.runOnce(
            () -> drivetrain.seedFieldRelative(toFirstDisk.getPreviewStartingHolonomicPose())),
        elevator.followShooter(shooterTilt::getPositionRotations),
        shooter.on(),
        Commands.sequence(
            shooterTilt.goToPositionBlocking(54.5),
            intake.fire(),
            Commands.waitSeconds(0.5),
            shooterTilt.goToPosition(TiltConstants.kIntakeAngle),
            Commands.parallel(intake.load(), AutoBuilder.followPath(toFirstDisk)),
            intake.fire()));
  }
}
