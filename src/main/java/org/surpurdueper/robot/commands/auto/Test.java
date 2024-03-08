package org.surpurdueper.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.littletonrobotics.util.AllianceFlipUtil;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class Test extends SequentialCommandGroup {

  public Test(CommandSwerveDrivetrain drivetrain) {

    PathPlannerPath toFirstDisk = PathPlannerPath.fromChoreoTrajectory("5 Standard.1");
    Rotation2d startingHeading = Rotation2d.fromRadians(-2.0848597284421473);
    Pose2d startingPose =
        new Pose2d(toFirstDisk.getPreviewStartingHolonomicPose().getTranslation(), startingHeading);

    addCommands(
        Commands.runOnce(() -> drivetrain.seedFieldRelative(AllianceFlipUtil.apply(startingPose))),
        AutoBuilder.followPath(toFirstDisk));
  }
}
