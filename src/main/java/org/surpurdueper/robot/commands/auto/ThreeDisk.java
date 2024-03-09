package org.surpurdueper.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.function.Supplier;
import org.surpurdueper.robot.Constants.TiltConstants;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Intake;
import org.surpurdueper.robot.subsystems.Shooter;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class ThreeDisk extends SequentialCommandGroup {

  public ThreeDisk(
      CommandSwerveDrivetrain drivetrain,
      Intake intake,
      ShooterTilt shooterTilt,
      Shooter shooter,
      Elevator elevator) {

    PathPlannerPath toThirdDisk = PathPlannerPath.fromChoreoTrajectory("5 Standard.2");
    Supplier<Pose2d> robotPoseSupplier = () -> drivetrain.getState().Pose;

    addCommands(
        new TwoDisk(drivetrain, intake, shooterTilt, shooter, elevator),
        Commands.deadline(
            AutoBuilder.followPath(toThirdDisk),
            shooterTilt.goToPosition(TiltConstants.kIntakeAngle),
            intake.load()),
        Autos.aimAndFireNoElevator(shooterTilt, intake, robotPoseSupplier));
  }
}
