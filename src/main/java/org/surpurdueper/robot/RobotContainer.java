// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.function.DoubleSupplier;
import org.frc3005.lib.vendor.motorcontroller.SparkMax;
import org.littletonrobotics.util.AllianceFlipUtil;
import org.littletonrobotics.util.FieldConstants;
import org.surpurdueper.robot.Constants.ClimberConstants;
import org.surpurdueper.robot.Constants.ElevatorConstants;
import org.surpurdueper.robot.Constants.TiltConstants;
import org.surpurdueper.robot.commands.AutoAim;
import org.surpurdueper.robot.commands.WheelRadiusCharacterization;
import org.surpurdueper.robot.commands.auto.CenterFiveDisk;
import org.surpurdueper.robot.commands.auto.FiveDisk;
import org.surpurdueper.robot.commands.auto.FourDisk;
import org.surpurdueper.robot.commands.auto.FourDiskSource;
import org.surpurdueper.robot.commands.auto.OneDiskSkip;
import org.surpurdueper.robot.commands.auto.ThreeDisk;
import org.surpurdueper.robot.commands.auto.ThreeDiskSkip;
import org.surpurdueper.robot.commands.auto.ThreeDiskSource;
import org.surpurdueper.robot.commands.auto.TwoDisk;
import org.surpurdueper.robot.commands.auto.TwoDiskSkip;
import org.surpurdueper.robot.commands.auto.TwoDiskSource;
import org.surpurdueper.robot.subsystems.Amp;
import org.surpurdueper.robot.subsystems.Blinkin;
import org.surpurdueper.robot.subsystems.Climber;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.Intake;
import org.surpurdueper.robot.subsystems.Limelight;
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
  final Intake intake;
  final Amp amp;
  final Climber climber;
  final Elevator elevator;
  final Shooter shooter;
  final ShooterTilt shooterTilt;
  final Blinkin blinkin;
  final Limelight limelight;

  /* Setting up bindings for necessary control of the swerve drive platform */
  private double MaxSpeed = Units.feetToMeters(16); // desired top speed
  private double MaxAngularRate =
      1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  private final CommandXboxController joystick2 = new CommandXboxController(1); // My joystick
  private final CommandXboxController joystick3 = new CommandXboxController(2); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
  // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    SparkMax.burnFlashInSync();

    intake = new Intake();
    amp = new Amp();
    climber = new Climber();
    elevator = new Elevator();
    shooter = new Shooter();
    shooterTilt = new ShooterTilt(intake);
    blinkin = new Blinkin(intake, shooter);
    limelight = new Limelight(drivetrain);

    Command doNothingAuto = Commands.none();
    Command twoDisk = new TwoDisk(drivetrain, intake, shooterTilt, shooter, elevator, limelight);
    Command threeDisk =
        new ThreeDisk(drivetrain, intake, shooterTilt, shooter, elevator, limelight);
    Command fourDisk = new FourDisk(drivetrain, intake, shooterTilt, shooter, elevator, limelight);
    Command fiveDisk = new FiveDisk(drivetrain, intake, shooterTilt, shooter, elevator, limelight);
    Command oneDiskSkip =
        new OneDiskSkip(drivetrain, intake, shooterTilt, shooter, elevator, limelight);
    Command twoDiskSkip =
        new TwoDiskSkip(drivetrain, intake, shooterTilt, shooter, elevator, limelight);
    Command threeDiskSkip =
        new ThreeDiskSkip(drivetrain, intake, shooterTilt, shooter, elevator, limelight);
    Command TwoDiskSource =
        new TwoDiskSource(drivetrain, intake, shooterTilt, shooter, elevator, limelight);
    Command ThreeDiskSource =
        new ThreeDiskSource(drivetrain, intake, shooterTilt, shooter, elevator, limelight);
    Command fourDiskSource =
        new FourDiskSource(drivetrain, intake, shooterTilt, shooter, elevator, limelight);
    Command fiveDiskCenter =
        new CenterFiveDisk(drivetrain, intake, shooterTilt, shooter, elevator, limelight);

    m_chooser.setDefaultOption("Do Nothing", doNothingAuto);
    m_chooser.addOption("Two Disk", twoDisk);
    m_chooser.addOption("Three Disk", threeDisk);
    m_chooser.addOption("Four Disk", fourDisk);
    m_chooser.addOption("Five Disk", fiveDisk);
    m_chooser.addOption("One Disk (Skip)", oneDiskSkip);
    m_chooser.addOption("Two Disk (Skip)", twoDiskSkip);
    m_chooser.addOption("Three Disk (Skip)", threeDiskSkip);
    m_chooser.addOption("Two Disk (Source)", TwoDiskSource);
    m_chooser.addOption("Three Disk (Source)", ThreeDiskSource);
    m_chooser.addOption("Four Disk (Source)", fourDiskSource);
    m_chooser.addOption("Five Disk (Center)", fiveDiskCenter);

    SmartDashboard.putData("Autonomous Routine", m_chooser);

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
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

    /*******************/
    /* DRIVER CONTROLS */
    /*******************/
    // Drive
    drivetrain.registerTelemetry(logger::telemeterize);
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
    joystick.start().onTrue(Commands.run(() -> CommandScheduler.getInstance().cancelAll()));

    joystick
        .a()
        .whileTrue(
            new AutoAim(
                drivetrain,
                shooterTilt,
                elevator,
                shooter,
                limelight,
                () -> squareJoystick(-joystick.getLeftY()) * MaxSpeed,
                () -> squareJoystick(-joystick.getLeftX()) * MaxSpeed));

    // Intake
    joystick.leftBumper().onTrue(intake());

    // Score
    joystick.rightBumper().onTrue(amp.score().andThen(elevator.goToPosition(0)));

    joystick.rightBumper().onTrue(intake.fire());

    joystick
        .leftTrigger()
        .onTrue(climber.run(() -> climber.setPositionRotations(ClimberConstants.kClimbPosition)));

    /*********************/
    /* OPERATOR CONTROLS */
    /*********************/
    // Load Amp
    joystick2
        .rightTrigger()
        .onTrue(
            elevator
                .goToPosition(0)
                .andThen(shooterTilt.goToPositionBlocking(TiltConstants.kAmpHandOff))
                .andThen(Commands.deadline(amp.load(), intake.feedAmp(), shooter.feedAmp()))
                .andThen(shooterTilt.goToPositionBlocking(TiltConstants.kSafeElevator))
                .andThen(elevator.goToPosition(ElevatorConstants.kAmpScoreHeight)));

    // Load for Trap
    joystick2
        .leftTrigger()
        .onTrue(
            elevator
                .goToPosition(0)
                .andThen(shooterTilt.goToPositionBlocking(TiltConstants.kAmpHandOff))
                .andThen(Commands.deadline(amp.load(), intake.feedAmp(), shooter.feedAmp()))
                .andThen(Commands.deadline(amp.trapLoad())));

    // Climb
    joystick2
        .a()
        .onTrue(
            shooterTilt
                .goToPosition(TiltConstants.kSafeElevator)
                .andThen(
                    elevator
                        .goToPositionBlocking(ElevatorConstants.kClimbHeight)
                        .andThen(climber.climb())
                        .andThen(Commands.waitSeconds(1))
                        .andThen(amp.trapScore().withTimeout(2))));

    // Purge
    joystick2
        .b()
        .whileTrue(
            Commands.parallel(
                intake.purge(),
                shooter.purge(),
                elevator.goToPositionBlocking(0).andThen(amp.purge())));

    // Manual shot locations
    joystick2
        .y()
        .whileTrue(
            shooterTilt
                .goToPosition(Constants.TiltConstants.kWallShot)
                .alongWith(elevator.followShooter(shooterTilt::getPositionRotations)));
    joystick2.y().onFalse(elevator.goToPosition(0));

    joystick2
        .x()
        .whileTrue(
            shooterTilt
                .goToPosition(Constants.TiltConstants.kStageShot)
                .alongWith(elevator.goToPosition(0)));
    joystick2
        .y()
        .or(joystick2.x())
        .whileTrue(shooter.startEnd(shooter::turnOn, shooter::turnOnIdle));

    joystick2.leftBumper().onTrue(shooterTilt.goToPosition(Constants.TiltConstants.kStageShot));
    joystick2.leftBumper().whileTrue(shooterTilt.startEnd(shooter::onfeedShot, shooter::idle));

    joystick2.rightBumper().onTrue(amp.trapLoad().andThen(amp.trapScore().withTimeout(2)));

    // Manual shooter tilt and climber control
    overDeadband(joystick2::getLeftY)
        .whileTrue(
            shooterTilt.runEnd(
                () -> shooterTilt.setVoltage(12 * applyDeadband(-joystick2.getLeftY())),
                shooterTilt::stop));
    overDeadband(joystick2::getRightY)
        .whileTrue(
            climber.runEnd(
                () -> climber.setVoltage(12 * applyDeadband(-joystick2.getRightY())),
                climber::stop));

    // Manual elevator up and down
    joystick2
        .povUp()
        .onTrue(
            shooterTilt
                .goToPositionBlocking(TiltConstants.kSafeElevator)
                .andThen(elevator.goToPosition(Units.inchesToMeters(20))));
    joystick2
        .povDown()
        .onTrue(
            shooterTilt
                .goToPositionBlocking(TiltConstants.kSafeElevator)
                .andThen(elevator.goToPosition(Units.inchesToMeters(0))));

    // Home shooter tilt
    joystick2.start().onTrue(shooterTilt.home());

    // Manual shooter elevator unjam
    joystick2
        .povLeft()
        .whileTrue(
            Commands.startEnd(
                () -> {
                  shooterTilt.setVoltage(3);
                  elevator.setVoltage(3);
                },
                () -> {
                  shooterTilt.stop();
                  elevator.stop();
                },
                shooterTilt,
                elevator));

    /* Bindings for characterization */
    /* These bindings require multiple buttons pushed to swap between quastatic and dynamic */
    /* Back/Start select dynamic/quasistatic, Y/X select forward/reverse direction */

    // Temporary buttons
    joystick3
        .rightTrigger()
        .onTrue(
            elevator
                .goToPosition(0)
                .andThen(shooterTilt.goToPositionBlocking(TiltConstants.kAmpHandOff))
                .andThen(Commands.deadline(amp.load(), intake.feedAmp(), shooter.feedAmp()))
                .andThen(shooterTilt.goToPositionBlocking(TiltConstants.kSafeElevator))
                .andThen(amp.trapLoad()));
    joystick3.start().and(joystick2.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick3.start().and(joystick2.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    joystick3
        .a()
        .whileTrue(
            new WheelRadiusCharacterization(
                drivetrain, WheelRadiusCharacterization.Direction.CLOCKWISE));
    joystick3
        .b()
        .whileTrue(
            new WheelRadiusCharacterization(
                drivetrain, WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE));
  }

  public Command intake() {
    return elevator
        .goToPosition(0)
        .alongWith(
            shooterTilt
                .goToPositionBlocking(TiltConstants.kIntakeAngle)
                .onlyIf(shooterTilt::isNotAtIntakeHeight)
                .andThen(intake.load())
                .andThen(rumbleDriverController()));
  }

  private Command rumbleDriverController() {
    return new ScheduleCommand(
        Commands.runOnce(() -> joystick.getHID().setRumble(RumbleType.kBothRumble, 1))
            .andThen(Commands.waitSeconds(0.75))
            .andThen(
                Commands.runOnce(() -> joystick.getHID().setRumble(RumbleType.kBothRumble, 0))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
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

  public Trigger overDeadband(DoubleSupplier joystick) {
    return new Trigger(() -> applyDeadband(Math.abs(joystick.getAsDouble())) > 0.0);
  }
}
