package org.surpurdueper.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.util.AllianceFlipUtil;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Intake;
import org.surpurdueper.robot.subsystems.Limelight;
import org.surpurdueper.robot.subsystems.Shooter;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class CenterFiveDisk extends SequentialCommandGroup {

  PathPlannerPath toCenterDisk = PathPlannerPath.fromChoreoTrajectory("front 3.1");
  PathPlannerPath backFromCenter = PathPlannerPath.fromChoreoTrajectory("front 3.2");
  PathPlannerPath toPodiumDisk = PathPlannerPath.fromChoreoTrajectory("front 3.3");
  PathPlannerPath toAmpDisk = PathPlannerPath.fromChoreoTrajectory("front 3.4");
  Rotation2d startingHeading = Rotation2d.fromRadians(Math.PI);
  Pose2d startingPose =
      new Pose2d(toCenterDisk.getPreviewStartingHolonomicPose().getTranslation(), startingHeading);

  public CenterFiveDisk(
      CommandSwerveDrivetrain drivetrain,
      Intake intake,
      ShooterTilt shooterTilt,
      Shooter shooter,
      Elevator elevator,
      Limelight limelight) {

    addCommands(
        Commands.runOnce(() -> drivetrain.seedFieldRelative(AllianceFlipUtil.apply(startingPose))),
        shooter.on(),
        Commands.deadline(
            AutoBuilder.followPath(toCenterDisk).andThen(AutoBuilder.followPath(backFromCenter)),
            Commands.sequence(
                Commands.deadline(
                    Commands.sequence(
                        Commands.waitSeconds(0.6), // Wait to shoot first shot
                        intake.runOnce(intake::runForward), // Turn on intake and feeder continously
                        Commands.waitSeconds(0.5) // Wait until second shot clears the shooter
                        ),
                    // Aim the shooter tilt continously until second shot is gone
                    new AutoAimNoDrive(drivetrain, shooterTilt, elevator)),
                intake.load())), // Change to normal intaking after second shot clears the shooter
        Autos.aimAndFireWithElevator(drivetrain, shooterTilt, elevator, shooter, limelight, intake),
        Commands.deadline(
            AutoBuilder.followPath(toPodiumDisk).andThen(Commands.waitSeconds(0.5)), intake.load()),
        Autos.aimAndFireWithElevator(drivetrain, shooterTilt, elevator, shooter, limelight, intake),
        Commands.deadline(AutoBuilder.followPath(toAmpDisk), intake.load()),
        Autos.aimAndFireWithElevator(
            drivetrain, shooterTilt, elevator, shooter, limelight, intake));
  }
}
