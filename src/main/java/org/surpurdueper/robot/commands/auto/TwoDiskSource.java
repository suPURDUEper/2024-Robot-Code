package org.surpurdueper.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.util.AllianceFlipUtil;
import org.surpurdueper.robot.Constants.TiltConstants;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Intake;
import org.surpurdueper.robot.subsystems.Limelight;
import org.surpurdueper.robot.subsystems.Shooter;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class TwoDiskSource extends SequentialCommandGroup {
  /** Creates a new FourDiskSource. */
  public TwoDiskSource(
      CommandSwerveDrivetrain drivetrain,
      Intake intake,
      ShooterTilt shooterTilt,
      Shooter shooter,
      Elevator elevator,
      Limelight limelight) {
    PathPlannerPath lineupFirstShot = PathPlannerPath.fromChoreoTrajectory("5 source side.1");
    PathPlannerPath linupSecondShot = PathPlannerPath.fromChoreoTrajectory("5 source side.2");

    Rotation2d startingHeading = Rotation2d.fromRadians(1.5551732981225221);
    Pose2d startingPose =
        new Pose2d(
            lineupFirstShot.getPreviewStartingHolonomicPose().getTranslation(), startingHeading);

    addCommands(
        Commands.runOnce(() -> drivetrain.seedFieldRelative(AllianceFlipUtil.apply(startingPose))),
        shooter.on(),
        AutoBuilder.followPath(lineupFirstShot),
        Autos.aimAndFireWithElevator(drivetrain, shooterTilt, elevator, shooter, limelight, intake),
        shooterTilt.goToPositionBlocking(TiltConstants.kIntakeAngle),
        Commands.deadline(AutoBuilder.followPath(linupSecondShot), intake.load()),
        Autos.aimAndFireWithElevator(
            drivetrain, shooterTilt, elevator, shooter, limelight, intake));
  }
}
