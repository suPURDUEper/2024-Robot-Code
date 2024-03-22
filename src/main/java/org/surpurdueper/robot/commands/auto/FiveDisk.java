package org.surpurdueper.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Intake;
import org.surpurdueper.robot.subsystems.Limelight;
import org.surpurdueper.robot.subsystems.Shooter;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class FiveDisk extends SequentialCommandGroup {

  public FiveDisk(
      CommandSwerveDrivetrain drivetrain,
      Intake intake,
      ShooterTilt shooterTilt,
      Shooter shooter,
      Elevator elevator,
      Limelight limelight) {

    PathPlannerPath toFifthDisk = PathPlannerPath.fromChoreoTrajectory("5 Standard.5");

    addCommands(
        new FourDisk(drivetrain, intake, shooterTilt, shooter, elevator, limelight),
        Commands.deadline(AutoBuilder.followPath(toFifthDisk), intake.load()),
        Autos.aimAndFireNoElevator(
                drivetrain, shooterTilt, elevator, shooter, limelight, intake, 1.0)
            .withTimeout(1.25));
  }
}
