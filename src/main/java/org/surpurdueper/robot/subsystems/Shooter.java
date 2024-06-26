// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.util.LoggedTunableNumber;
import org.surpurdueper.robot.Constants.CANIDs;
import org.surpurdueper.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  // Class variables
  private TalonFX shooterLeft;
  private TalonFX shooterRight;
  private TalonFXConfiguration config;
  public double shooterLeftTargetRps = 0;
  private double shooterRightTargetRps = 0;

  // Tunable numbers
  private static final LoggedTunableNumber kp = new LoggedTunableNumber("Shooter/Kp");
  private static final LoggedTunableNumber ki = new LoggedTunableNumber("Shooter/Ki");
  private static final LoggedTunableNumber kd = new LoggedTunableNumber("Shooter/Kd");
  private static final LoggedTunableNumber ks = new LoggedTunableNumber("Shooter/Ks");
  private static final LoggedTunableNumber kv = new LoggedTunableNumber("Shooter/Kv");
  private static final LoggedTunableNumber ka = new LoggedTunableNumber("Shooter/Ka");
  private static final List<LoggedTunableNumber> pidGains = new ArrayList<>();

  // Control requests
  private final ControlRequest stopRequest = new NeutralOut();
  private final TorqueCurrentFOC voltagRequest = new TorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0);

  // SysID Setup
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(
              Volts.of(10).per(Seconds.of(1)),
              Volts.of(40),
              null,
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                shooterRight.setControl(voltagRequest.withOutput(volts.in(Units.Volts)));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              null, // Using the CTRE SignalLogger API instead
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

  static {
    kp.initDefault(ShooterConstants.kp);
    ki.initDefault(ShooterConstants.ki);
    kd.initDefault(ShooterConstants.kd);
    ks.initDefault(ShooterConstants.ks);
    kv.initDefault(ShooterConstants.kv);
    ka.initDefault(ShooterConstants.ka);
    pidGains.addAll(List.of(kp, ki, kd, ks, kv, ka));
  }

  /** Creates a new shooter. */
  public Shooter() {
    shooterRight = new TalonFX(CANIDs.kShooterRightMotor, "canivore");
    shooterLeft = new TalonFX(CANIDs.kShooterLeftMotor, "canivore");
    configureTalonFx();
  }

  private void setupSysIdTiming(TalonFX motorToTest) {
    /* Speed up signals for better charaterization data */
    BaseStatusSignal.setUpdateFrequencyForAll(
        250, motorToTest.getPosition(), motorToTest.getVelocity(), motorToTest.getMotorVoltage());

    /* Optimize out the other signals, since they're not particularly helpful for us */
    motorToTest.optimizeBusUtilization();
  }

  @Override
  public void periodic() {
    // Update PID values if they've changed
    for (LoggedTunableNumber gain : pidGains) {
      if (gain.hasChanged(hashCode())) {
        // Send new PID gains to talon
        Slot0Configs slot0config =
            new Slot0Configs()
                .withKP(kp.get())
                .withKI(ki.get())
                .withKD(kd.get())
                .withKS(ks.get())
                .withKV(kv.get())
                .withKA(ka.get());
        shooterLeft.getConfigurator().apply(slot0config);
        shooterRight.getConfigurator().apply(slot0config);
        break;
      }
    }

    // Log out velocity and target to Glass so we can measure performance
    double shooterLeftTarget = shooterLeft.getClosedLoopReference().getValueAsDouble();
    double shooterLeftActual = shooterLeft.getVelocity().getValueAsDouble();
    double shooterRightTarget = shooterRight.getClosedLoopReference().getValueAsDouble();
    double shooterRightActual = shooterRight.getVelocity().getValueAsDouble();
    SmartDashboard.putNumber("Shooter/Shooter Left Target (RPM)", shooterLeftTarget * 60.0);
    SmartDashboard.putNumber("Shooter/Shooter Left (RPM)", shooterLeftActual * 60.0);
    SmartDashboard.putNumber("Shooter/Shooter Right Target (RPM)", shooterRightTarget * 60.0);
    SmartDashboard.putNumber("Shooter/Shooter Right (RPM)", shooterRightActual * 60.0);
  }

  public void setShooterRps(double shooterLeftRps, double shooterRightRps) {
    shooterLeftTargetRps = shooterLeftRps;
    shooterRightTargetRps = shooterRightRps;
    shooterLeft.setControl(velocityRequest.withVelocity(shooterLeftTargetRps));
    shooterRight.setControl(velocityRequest.withVelocity(shooterRightTargetRps));
  }

  public void stop() {
    shooterLeftTargetRps = 0;
    shooterRightTargetRps = 0;
    shooterLeft.setControl(stopRequest);
    shooterRight.setControl(stopRequest);
  }

  public boolean isShooterAtSpeed() {
    return Math.abs(shooterLeft.getVelocity().getValue() - shooterLeftTargetRps)
            < ShooterConstants.kShooterRpsTolerance
        && Math.abs(shooterLeft.getVelocity().getValue() - shooterLeftTargetRps)
            < ShooterConstants.kShooterRpsTolerance;
  }

  public void setVoltage(double leftVoltage, double rightVoltage) {
    shooterLeft.setControl(voltagRequest.withOutput(leftVoltage));
    shooterRight.setControl(voltagRequest.withOutput(rightVoltage));
  }

  public void turnOn() {
    setShooterRps(ShooterConstants.kLeftShooterSpeedRps, ShooterConstants.kRightShooterSpeedRps);
  }

  public void turnOnIdle() {
    setShooterRps(ShooterConstants.kLeftShooterIdleRps, ShooterConstants.kRightShooterIdleRps);
  }

  public void turnOnFeedShot() {
    setShooterRps(ShooterConstants.kLeftShooterFeedRps, ShooterConstants.kRightShooterFeedRps);
  }

  public Command on() {
    return Commands.runOnce(this::turnOn, this);
  }

  public Command idle() {
    return Commands.runOnce(this::turnOnIdle, this);
  }

  public Command off() {
    return Commands.runOnce(this::stop, this);
  }

  public Command feedAmp() {
    return Commands.startEnd(
        () ->
            setShooterRps(
                ShooterConstants.kLeftShooterAmpRps, ShooterConstants.kRightShooterAmpRps),
        () -> stop(),
        this);
  }

  public Command onfeedShot() {
    return Commands.runOnce(this::turnOnFeedShot, this);
  }

  public Command purge() {
    return Commands.startEnd(() -> setVoltage(-2, -2), this::stop, this);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public void configureTalonFx() {
    MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive);
    CurrentLimitsConfigs currentConfig =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(ShooterConstants.kStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true);
    TorqueCurrentConfigs torqueCurrentConfigs =
        new TorqueCurrentConfigs()
            .withPeakForwardTorqueCurrent(120)
            .withPeakReverseTorqueCurrent(-120);
    FeedbackConfigs feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(ShooterConstants.kGearRatio);
    Slot0Configs slot0config =
        new Slot0Configs()
            .withKP(kp.get())
            .withKI(ki.get())
            .withKD(kd.get())
            .withKS(ks.get())
            .withKV(kv.get())
            .withKA(ka.get());
    config =
        new TalonFXConfiguration()
            .withSlot0(slot0config)
            .withCurrentLimits(currentConfig)
            .withFeedback(feedbackConfig)
            .withTorqueCurrent(torqueCurrentConfigs);
    shooterLeft.getConfigurator().apply(config.withMotorOutput(motorOutputConfigs));
    shooterRight
        .getConfigurator()
        .apply(
            config.withMotorOutput(
                motorOutputConfigs.withInverted(InvertedValue.Clockwise_Positive)));
  }
}
