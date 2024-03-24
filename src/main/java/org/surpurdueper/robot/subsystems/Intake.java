package org.surpurdueper.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxExtensions;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.UUID;

import org.frc3005.lib.vendor.motorcontroller.SparkMax;
import org.frc3005.lib.vendor.motorcontroller.SparkMaxUtils;
import org.surpurdueper.robot.Constants.CANIDs;
import org.surpurdueper.robot.Constants.DIOPorts;
import org.surpurdueper.robot.utils.LimelightHelpers;

public class Intake extends SubsystemBase {

  public CANSparkMax intakeMotor;
  CANSparkMax feederMotor;
  DigitalInput feederBreakBeam1;
  DigitalInput feederBreakBeam2;

  public Intake() {
    intakeMotor =
        new SparkMax(CANIDs.kIntakeMotor).withInitializer(Intake::sparkMaxIntakeInitializer);
    feederMotor =
        new SparkMax(CANIDs.kFeederMotor).withInitializer(Intake::sparkMaxFeederInitializer);
    feederBreakBeam1 = new DigitalInput(DIOPorts.kFeederBreakBeam1);
    feederBreakBeam2 = new DigitalInput(DIOPorts.kFeederBreakBeam2);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake/BreakBeam1", feederBreakBeam1.get());
    SmartDashboard.putBoolean("Intake/BreakBeam2", feederBreakBeam2.get());
  }

  public boolean isFeederBreakBeamTriggered() {
    return !feederBreakBeam1.get() || !feederBreakBeam2.get();
  }

  public void runForward() {
    intakeMotor.setVoltage(6);
    feederMotor.setVoltage(8);
  }

  public void runBackwards() {
    intakeMotor.setVoltage(-3);
    feederMotor.setVoltage(-3);
  }

  public void stop() {
    intakeMotor.stopMotor();
    feederMotor.stopMotor();
  }

  public Command load() {
    return Commands.startEnd(this::runForward, this::stop, this)
        .until(this::isFeederBreakBeamTriggered);
  }

  public Command fire() {
    return Commands.startEnd(() -> {
      feederMotor.setVoltage(12);
      // Start async limelight capture
      String snapshotName = "";
      if (DriverStation.isFMSAttached()) {
        try {
          StringBuilder builder = new StringBuilder();
          builder.append("Match ");
          builder.append(DriverStation.getMatchNumber());
          builder.append(": ");
          builder.append(DriverStation.getMatchTime());
          snapshotName = builder.toString();
        } catch (Exception e) {
          snapshotName = UUID.randomUUID().toString();
        }
      } else {
        snapshotName = UUID.randomUUID().toString();
      }
      LimelightHelpers.takeSnapshot("limelight", snapshotName);
      
    }, feederMotor::stopMotor, this);
  }

  public Command feedAmp() {
    return Commands.startEnd(() -> feederMotor.setVoltage(4), feederMotor::stopMotor, this);
  }

  public Command purge() {
    return Commands.startEnd(this::runBackwards, this::stop, this);
  }

  private static Boolean sparkMaxFeederInitializer(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo(sparkMax));
    errors += SparkMaxUtils.check(CANSparkMaxExtensions.setInverted(sparkMax, false));
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kBrake));
    // SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kNoFeedback);
    return errors == 0;
  }

  private static Boolean sparkMaxIntakeInitializer(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo(sparkMax));
    errors += SparkMaxUtils.check(CANSparkMaxExtensions.setInverted(sparkMax, false));
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kCoast));
    // SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kNoFeedback);
    return errors == 0;
  }
}
