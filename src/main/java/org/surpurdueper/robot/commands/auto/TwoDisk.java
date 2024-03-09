package org.surpurdueper.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.Supplier;
import org.littletonrobotics.util.AllianceFlipUtil;
import org.surpurdueper.robot.Constants.TiltConstants;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Intake;
import org.surpurdueper.robot.subsystems.Shooter;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class TwoDisk extends SequentialCommandGroup {

  public TwoDisk(
      CommandSwerveDrivetrain drivetrain,
      Intake intake,
      ShooterTilt shooterTilt,
      Shooter shooter,
      Elevator elevator) {

    PathPlannerPath toFirstDisk = PathPlannerPath.fromChoreoTrajectory("5 Standard.1");
    Rotation2d startingHeading = Rotation2d.fromRadians(-2.0848597284421473);
    Pose2d startingPose =
        new Pose2d(toFirstDisk.getPreviewStartingHolonomicPose().getTranslation(), startingHeading);
    Supplier<Pose2d> robotPoseSupplier = () -> drivetrain.getState().Pose;

    addCommands(
        Commands.runOnce(() -> drivetrain.seedFieldRelative(AllianceFlipUtil.apply(startingPose))),
        Commands.deadline(
            Commands.sequence(
                shooter.on(),
                shooterTilt.goToPositionBlocking(Units.degreesToRotations(54.5)),
                elevator.waitUntilAtPosition(),
                intake.fire().withTimeout(0.5)),
            elevator.followShooter(shooterTilt::getPositionRotations)),
        elevator.goToPosition(0),
        Commands.waitSeconds(0.5),
        shooterTilt.goToPositionBlocking(TiltConstants.kIntakeAngle),
        Commands.parallel(intake.load(), AutoBuilder.followPath(toFirstDisk)),
        Autos.aimAndFireWithElevator(shooterTilt, elevator, intake, robotPoseSupplier));
  }
}
