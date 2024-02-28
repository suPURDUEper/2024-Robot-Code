package org.surpurdueper.robot.commands;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.util.AllianceFlipUtil;
import org.littletonrobotics.util.FieldConstants;
import org.surpurdueper.robot.Constants.LookupTables;
import org.surpurdueper.robot.subsystems.Elevator;
import org.surpurdueper.robot.subsystems.ShooterTilt;
import org.surpurdueper.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoAim extends Command {
  CommandSwerveDrivetrain drivetrain;
  ShooterTilt shooterTilt;
  Elevator elevator;
  DoubleSupplier xVelocitySupplier;
  DoubleSupplier yVelocitySupplier;
  Translation2d speakerCenter;
  FieldCentricFacingPoint swerveRequest;

  protected ElevatorSyncThread elevatorSyncThread;

  public AutoAim(
      CommandSwerveDrivetrain drivetrain,
      ShooterTilt shooterTilt,
      Elevator elevator,
      DoubleSupplier xVelocitySupplier,
      DoubleSupplier yVelocitySupplier) {
    this.drivetrain = drivetrain;
    this.shooterTilt = shooterTilt;
    this.elevator = elevator;
    this.xVelocitySupplier = xVelocitySupplier;
    this.yVelocitySupplier = yVelocitySupplier;
    addRequirements(drivetrain, elevator); // TODO: Add shooterTilt back to this

    // Setup request to control drive always facing the speaker
    swerveRequest = new FieldCentricFacingPoint();
    swerveRequest.HeadingController.setPID(10, 0, 0);
    swerveRequest.HeadingController.enableContinuousInput(-180.0, 180.0);

    elevatorSyncThread = new ElevatorSyncThread();
  }

  @Override
  public void initialize() {
    speakerCenter =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d());
    swerveRequest.setPointToFace(speakerCenter);
    elevatorSyncThread.start();
  }

  @Override
  public void execute() {
    // Update drivetrain with new joystick values
    SmartDashboard.putNumber(
        "AutoAim/TargetDirection", swerveRequest.getTargetDirection().getDegrees());
    double velocityX = xVelocitySupplier.getAsDouble();
    double velocityY = yVelocitySupplier.getAsDouble();
    drivetrain.setControl(
        swerveRequest.withVelocityX(velocityX).withVelocityY(velocityY).withDeadband(0.1));

    // Update shooter angle from current pose
    double distanceToSpeakerMeters =
        drivetrain.getState().Pose.getTranslation().getDistance(speakerCenter);
    shooterTilt.setPositionRotations(
        LookupTables.distanceToShooterAngle.get(distanceToSpeakerMeters));
    shooterTilt.setPositionDegrees(53.0); // TODO: Remove
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSyncThread.stop();
    elevator.setPositionMeters(0);
  }

  public class ElevatorSyncThread {
    protected static final int START_THREAD_PRIORITY =
        1; // Testing shows 1 (minimum realtime) is sufficient for tighter
    // odometry loops.
    // If the odometry period is far away from the desired frequency,
    // increasing this may help
    protected final Thread m_thread;
    protected volatile boolean m_running = false;
    protected final StatusSignal<Double>[] m_signals;

    public ElevatorSyncThread() {
      m_thread = new Thread(this::run);
      /* Mark this thread as a "daemon" (background) thread
       * so it doesn't hold up program shutdown */
      m_thread.setDaemon(true);

      /* 2 signals for position and velocity */
      m_signals = new StatusSignal[2];
      m_signals[0] = shooterTilt.getPositionSignal();
      m_signals[1] = shooterTilt.getVelocitySignal();
    }

    /** Starts the odometry thread. */
    public void start() {
      m_running = true;
      m_thread.start();
    }

    /** Stops the odometry thread. */
    public void stop() {
      stop(0);
    }

    /**
     * Stops the odometry thread with a timeout.
     *
     * @param millis The time to wait in milliseconds
     */
    public void stop(long millis) {
      m_running = false;
      try {
        m_thread.join(millis);
      } catch (final InterruptedException ex) {
        Thread.currentThread().interrupt();
      }
    }

    public void run() {
      /* Run as fast as possible, our signals will control the timing */
      double UpdateFrequency = 250;
      BaseStatusSignal.setUpdateFrequencyForAll(UpdateFrequency, m_signals);
      Threads.setCurrentThreadPriority(true, START_THREAD_PRIORITY);
      while (m_running) {
        /* Synchronously wait for all signals */
        /* Wait up to twice the period of the update frequency */
        StatusCode status = BaseStatusSignal.waitForAll(2.0 / UpdateFrequency, m_signals);
        if (status.isOK()) {
          double shooterAngleRotations =
              BaseStatusSignal.getLatencyCompensatedValue(m_signals[0], m_signals[1]);
          double elevatorHeight = LookupTables.elevatorShooterClearance.get(shooterAngleRotations);
          elevator.setPositionMeters(elevatorHeight);
        }
      }
    }
  }
}
