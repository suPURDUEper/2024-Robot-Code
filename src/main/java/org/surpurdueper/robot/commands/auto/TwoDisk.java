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
import org.surpurdueper.robot.subsystems.Limelight;
import org.surpurdueper.robot.subsystems.Shooter;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class TwoDisk extends SequentialCommandGroup {

  public TwoDisk(
      CommandSwerveDrivetrain drivetrain,
      Intake intake,
      ShooterTilt shooterTilt,
      Shooter shooter,
      Elevator elevator,
      Limelight limelight) {

    PathPlannerPath lineupFirstShot = PathPlannerPath.fromChoreoTrajectory("5 Standard.1");
    PathPlannerPath toSecondDisk = PathPlannerPath.fromChoreoTrajectory("5 Standard.2");

    Rotation2d startingHeading = Rotation2d.fromRadians(-2.0848597284421473);
    Pose2d startingPose =
        new Pose2d(lineupFirstShot.getPreviewStartingHolonomicPose().getTranslation(), startingHeading);

    addCommands(
        Commands.runOnce(() -> drivetrain.seedFieldRelative(AllianceFlipUtil.apply(startingPose))),
        shooter.on(),
        AutoBuilder.followPath(lineupFirstShot),
        Autos.aimAndFireWithElevator(drivetrain, shooterTilt, elevator, shooter, limelight, intake).withTimeout(1.25),
        shooterTilt.goToPositionBlocking(TiltConstants.kIntakeAngle),
        Commands.parallel(intake.load(), AutoBuilder.followPath(toSecondDisk)),
        Autos.aimAndFireWithElevator(drivetrain, shooterTilt, elevator, shooter, limelight, intake).withTimeout(1.25));
      }
}
