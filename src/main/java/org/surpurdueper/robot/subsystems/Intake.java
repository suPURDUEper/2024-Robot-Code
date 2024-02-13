package org.surpurdueper.robot.subsystems;

import static org.surpurdueper.robot.Constants.CANIDs;
import static org.surpurdueper.robot.Constants.DIOPorts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxExtensions;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc3005.lib.vendor.motorcontroller.SparkMax;
import org.frc3005.lib.vendor.motorcontroller.SparkMax.FrameStrategy;
import org.frc3005.lib.vendor.motorcontroller.SparkMaxUtils;
import org.surpurdueper.robot.Constants.CANIDs;
import org.surpurdueper.robot.Constants.DIOPorts;

public class Intake extends SubsystemBase {

  CANSparkMax intakeMotor;
  CANSparkMax feederMotor;
  DigitalInput feederBreakBeam;

  public Intake() {
    intakeMotor = new SparkMax(CANIDs.kIntakeMotor).withInitializer(Intake::sparkMaxInitializer);
    feederMotor = new SparkMax(CANIDs.kFeederMotor).withInitializer(Intake::sparkMaxInitializer);
    feederBreakBeam = new DigitalInput(DIOPorts.kFeederBreakBeam);
  }

  public void runForward() {
    intakeMotor.setVoltage(12);
    feederMotor.setVoltage(12);
  }

  public void runBackwards() {
    intakeMotor.setVoltage(-12);
    feederMotor.setVoltage(-12);
  }

  public void stop() {
    intakeMotor.stopMotor();
    feederMotor.stopMotor();
  }

  public Command load() {
    return Commands.startEnd(this::runForward, this::stop, this).until(feederBreakBeam::get);
  }

  public Command fire() {
    return Commands.startEnd(() -> feederMotor.setVoltage(12), feederMotor::stopMotor, this);
  }

  public Command purge() {
    return Commands.startEnd(this::runBackwards, this::stop, this);
  }

  private static Boolean sparkMaxInitializer(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo(sparkMax));
    errors += SparkMaxUtils.check(CANSparkMaxExtensions.setInverted(sparkMax, false));
    SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kNoFeedback);
    return errors == 0;
  }
}
