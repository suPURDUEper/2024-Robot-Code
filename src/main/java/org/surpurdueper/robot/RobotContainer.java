// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import java.util.Map;
import org.frc3005.lib.vendor.motorcontroller.SparkMax;
import org.littletonrobotics.util.AllianceFlipUtil;
import org.littletonrobotics.util.FieldConstants;
import org.surpurdueper.robot.Constants.ElevatorConstants;
import org.surpurdueper.robot.Constants.TiltConstants;
import org.surpurdueper.robot.commands.AutoAim;
import org.surpurdueper.robot.commands.auto.Test;
import org.surpurdueper.robot.commands.auto.TwoDisk;
import org.surpurdueper.robot.subsystems.Amp;
import org.surpurdueper.robot.subsystems.Blinkin;
import org.surpurdueper.robot.subsystems.Climber;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Intake;
import org.surpurdueper.robot.subsystems.Shooter;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;
import org.surpurdueper.robot.subsystems.drive.Telemetry;
import org.surpurdueper.robot.subsystems.drive.generated.TunerConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Intake intake;
  private final Amp amp;
  private final Climber climber;
  private final Elevator elevator;
  private final Shooter shooter;
  private final ShooterTilt shooterTilt;
  private final Blinkin blinkin;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private double MaxSpeed = Units.feetToMeters(13); // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate =
      1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandXboxController joystick2 = new CommandXboxController(1); // My joystick

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SparkMax.burnFlashInSync();

    intake = new Intake();
    amp = new Amp();
    climber = new Climber();
    elevator = new Elevator();
    shooter = new Shooter();
    shooterTilt = new ShooterTilt(intake);
    blinkin = new Blinkin();

    NamedCommands.registerCommands(
        Map.of(
            "Shooter On", shooter.on(),
            "Shooter Off", shooter.off(),
            "Intake", intake(),
            "Elevator Follow", elevator.followShooter(shooterTilt::getPositionRotations),
            "Fire", intake.fire()));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Drive
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain
            .applyRequest(
                () ->
                    drive
                        .withVelocityX(squareJoystick(-joystick.getLeftY()) * MaxSpeed)
                        .withVelocityY(squareJoystick(-joystick.getLeftX()) * MaxSpeed)
                        .withRotationalRate(squareJoystick(-joystick.getRightX()) * MaxAngularRate))
            .ignoringDisable(true));
    joystick
        .back()
        .onTrue(
            drivetrain.runOnce(
                () -> {
                  Pose2d resetPose =
                      new Pose2d(
                          FieldConstants.Subwoofer.centerFace.getX()
                              + Constants.kBumperToRobotCenter,
                          FieldConstants.Subwoofer.centerFace.getY(),
                          Rotation2d.fromDegrees(180));
                  drivetrain.seedFieldRelative(AllianceFlipUtil.apply(resetPose));
                }));
    drivetrain.registerTelemetry(logger::telemeterize);

    joystick
        .a()
        .whileTrue(
            new AutoAim(
                drivetrain,
                shooterTilt,
                elevator,
                shooter,
                () -> squareJoystick(-joystick.getLeftY()) * MaxSpeed,
                () -> squareJoystick(-joystick.getLeftX()) * MaxSpeed));

    // Intake
    joystick.leftBumper().onTrue(intake());

    joystick
        .b()
        .whileTrue(
            Commands.parallel(
                intake.purge(),
                shooter.purge(),
                elevator.goToPositionBlocking(0).andThen(amp.purge())));

    // Score
    joystick
        .rightBumper()
        .and(amp::isAmpLoaded)
        .onTrue(amp.score().andThen(elevator.goToPosition(0), blinkin.setLightsOff()));

    joystick
        .rightBumper()
        .and(amp::isAmpNotLoaded)
        .onTrue(intake.fire().andThen(blinkin.setLightsOff()));

    // Load Amp
    joystick
        .rightTrigger()
        .onTrue(
            elevator
                .goToPosition(0)
                .andThen(shooterTilt.goToPositionBlocking(TiltConstants.kAmpHandOff))
                .andThen(Commands.deadline(amp.load(), intake.feedAmp(), shooter.feedAmp()))
                .andThen(shooterTilt.goToPositionBlocking(TiltConstants.kSafeElevator))
                .andThen(elevator.goToPosition(ElevatorConstants.kAmpScoreHeight)));

    joystick.start().onTrue(Commands.run(() -> CommandScheduler.getInstance().cancelAll()));

    // Temporary buttons
    joystick
        .povUp()
        .whileTrue(
            Commands.startEnd(
                () -> shooterTilt.setVoltage(4), () -> shooterTilt.stop(), shooterTilt));
    joystick
        .povDown()
        .whileTrue(
            Commands.startEnd(
                () -> shooterTilt.setVoltage(-4), () -> shooterTilt.stop(), shooterTilt));

    joystick.leftTrigger().whileTrue(amp.startEnd(() -> amp.setVoltage(10), () -> amp.stop()));

    joystick
        .x()
        .whileTrue(Commands.startEnd(() -> climber.setVoltage(-12), () -> climber.stop(), climber));

    joystick
        .y()
        .whileTrue(Commands.startEnd(() -> climber.setVoltage(12), () -> climber.stop(), climber));

    joystick.povRight().onTrue(elevator.goToPosition(Units.inchesToMeters(20)));
    joystick.povLeft().onTrue(elevator.goToPosition(Units.inchesToMeters(0)));

    // shooterTilt.setDefaultCommand(
    //     Commands.run(
    //         () -> shooterTilt.setVoltage(8 * applyDeadband(joystick2.getRightY())),
    // shooterTilt));
    // joystick2.a().onTrue(intake.load());
    // joystick2.b().whileTrue(intake.purge());

    /* Bindings for characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */
    joystick2.back().and(joystick2.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick2.back().and(joystick2.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick2.start().and(joystick2.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick2.start().and(joystick2.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  }

  public Command intake() {
    return elevator
        .goToPosition(0)
        .alongWith(
            shooterTilt
                .goToPositionBlocking(TiltConstants.kIntakeAngle)
                .onlyIf(shooterTilt::isNotAtIntakeHeight)
                .andThen(
                    intake.load(),
                    new ScheduleCommand(
                        Commands.sequence(
                            blinkin.setLightsStrobeGold(),
                            Commands.waitSeconds(1),
                            blinkin.setLightsOrange()))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new Test(drivetrain);
  }

  public double applyDeadband(double value) {
    double deadband = 0.1;
    if (Math.abs(value) < deadband) {
      return 0.0;
    }
    return value;
  }

  public double squareJoystick(double value) {
    double sign = Math.signum(value);
    return value * value * sign;
  }
}
