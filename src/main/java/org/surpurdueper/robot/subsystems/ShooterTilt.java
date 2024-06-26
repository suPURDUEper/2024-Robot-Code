// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.util.LoggedTunableNumber;
import org.surpurdueper.robot.Constants.CANIDs;
import org.surpurdueper.robot.Constants.TiltConstants;

public class ShooterTilt extends SubsystemBase {

  // Class variables
  private TalonFX tiltMotor;
  private TalonFXConfiguration tiltConfig;
  private double targetRotations = -1;
  private boolean isHomed = false;
  private double lowestPosition = Double.MAX_VALUE;

  // Tunable numbers
  private static final LoggedTunableNumber kp = new LoggedTunableNumber("ShooterTilt/Kp");
  private static final LoggedTunableNumber ki = new LoggedTunableNumber("ShooterTilt/Ki");
  private static final LoggedTunableNumber kd = new LoggedTunableNumber("ShooterTilt/Kd");
  private static final LoggedTunableNumber ks = new LoggedTunableNumber("ShooterTilt/Ks");
  private static final LoggedTunableNumber kv = new LoggedTunableNumber("ShooterTilt/Kv");
  private static final LoggedTunableNumber ka = new LoggedTunableNumber("ShooterTilt/Ka");
  private static final LoggedTunableNumber kg = new LoggedTunableNumber("ShooterTilt/Kg");
  private static final LoggedTunableNumber profileKv =
      new LoggedTunableNumber("ShooterTilt/profileKv");
  private static final LoggedTunableNumber profileKa =
      new LoggedTunableNumber("ShooterTilt/profileKa");
  private static final List<LoggedTunableNumber> pidGains = new ArrayList<>();

  // Control requests
  private final StaticBrake stopRequest = new StaticBrake();
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0);

  private static final Measure<Velocity<Voltage>> sysIdRampRate =
      edu.wpi.first.units.Units.Volts.of(1).per(Seconds.of(1));
  private static final Measure<Voltage> sysIdStepAmps = edu.wpi.first.units.Units.Volts.of(7);
  // SysID Setup
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(
              sysIdRampRate,
              sysIdStepAmps,
              null,
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                tiltMotor.setControl(
                    voltageRequest.withOutput(volts.in(edu.wpi.first.units.Units.Volts)));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              null, // Using the CTRE SignalLogger API instead
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

  static {
    kp.initDefault(TiltConstants.kp);
    ki.initDefault(TiltConstants.ki);
    kd.initDefault(TiltConstants.kd);
    ks.initDefault(TiltConstants.ks);
    kv.initDefault(TiltConstants.kv);
    ka.initDefault(TiltConstants.ka);
    kg.initDefault(TiltConstants.kg);
    profileKv.initDefault(TiltConstants.profileKv);
    profileKa.initDefault(TiltConstants.profileKa);
    pidGains.addAll(List.of(kp, ki, kd, ks, kv, ka, kg, profileKa, profileKv));
  }

  public ShooterTilt(Intake intake) {
    tiltMotor = new TalonFX(CANIDs.kTiltMotor, "canivore");

    configureTalonFx();
    // setupSysIdTiming(tiltMotor);

    // Elevator reacts to the shooter angle during auto aim. Increase the frequency at which this is
    // updated
    BaseStatusSignal.setUpdateFrequencyForAll(
        250, tiltMotor.getPosition(), tiltMotor.getVelocity());
    SmartDashboard.putBoolean("ShooterTilt/isHomed", isHomed);
  }

  @Override
  public void periodic() {

    // Update tunable numbers
    for (LoggedTunableNumber gain : pidGains) {
      if (gain.hasChanged(hashCode())) {
        // Send new PID gains to talon
        Slot0Configs slot0config =
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKP(kp.get())
                .withKI(ki.get())
                .withKD(kd.get())
                .withKS(ks.get())
                .withKV(kv.get())
                .withKA(ka.get())
                .withKG(kg.get());
        MotionMagicConfigs motionMagicConfigs =
            new MotionMagicConfigs()
                .withMotionMagicExpo_kA(profileKa.get())
                .withMotionMagicExpo_kV(profileKv.get());
        tiltMotor
            .getConfigurator()
            .apply(tiltConfig.withSlot0(slot0config).withMotionMagic(motionMagicConfigs));
        break;
      }
    }

    if (DriverStation.isDisabled() && !isHomed) {
      double motorPosition = tiltMotor.getPosition().getValueAsDouble();
      if (motorPosition < lowestPosition) {
        lowestPosition = motorPosition;
      } else if (motorPosition > lowestPosition + Units.degreesToRotations(10)) {
        tiltMotor.setPosition(TiltConstants.kHardStopPosition + (motorPosition - lowestPosition));
        tiltMotor
            .getConfigurator()
            .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
        isHomed = true;
        SmartDashboard.putBoolean("ShooterTilt/isHomed", isHomed);
      }
    }

    // Log out to Glass for debugging
    double armPositionMotor = Units.rotationsToDegrees(tiltMotor.getPosition().getValueAsDouble());
    double armPositionSetpoint =
        Units.rotationsToDegrees(tiltMotor.getClosedLoopReference().getValueAsDouble());
    SmartDashboard.putNumber("ShooterTilt/Position (Motor)", armPositionMotor);
    SmartDashboard.putNumber("ShooterTilt/Setpoint", targetRotations);
    SmartDashboard.putNumber("ShooterTilt/Profile Target", armPositionSetpoint);
    SmartDashboard.putNumber("ShooterTilt/Lowest Position (Homing)", lowestPosition);
    SmartDashboard.putBoolean("ShooterTilt/AtIntakeAngle", !isNotAtIntakeHeight());
    SmartDashboard.putBoolean("ShooterTilt/AtPosition", isAtPosition());
  }

  private void setupSysIdTiming(TalonFX motorToTest) {
    /* Speed up signals for better charaterization data */
    BaseStatusSignal.setUpdateFrequencyForAll(
        250, motorToTest.getPosition(), motorToTest.getVelocity(), motorToTest.getMotorVoltage());

    /* Optimize out the other signals, since they're not particularly helpful for us */
    motorToTest.optimizeBusUtilization();
  }

  public void setVoltage(double volts) {
    tiltMotor.setControl(voltageRequest.withOutput(volts));
  }

  public void setPositionRads(double position) {
    setPositionRotations(Units.radiansToRotations(position));
  }

  public void setPositionDegrees(double position) {
    setPositionRotations(Units.degreesToRotations(position));
  }

  public void setPositionRotations(double position) {
    targetRotations = position;
    tiltMotor.setControl(positionRequest.withPosition(targetRotations));
  }

  public double getPositionRotations() {
    return tiltMotor.getPosition().getValueAsDouble();
  }

  public StatusSignal<Double> getPositionSignal() {
    return tiltMotor.getPosition();
  }

  public StatusSignal<Double> getVelocitySignal() {
    return tiltMotor.getVelocity();
  }

  public void stop() {
    tiltMotor.setControl(stopRequest);
  }

  public boolean isAtPosition() {
    return Math.abs(targetRotations - tiltMotor.getPosition().getValueAsDouble())
            < TiltConstants.kPositionTolerance
        && tiltMotor.getVelocity().getValueAsDouble() < 0.1;
  }

  public Command goToPosition(double rotations) {
    return runOnce(() -> setPositionRotations(rotations));
  }

  public Command goToPositionBlocking(double rotations) {
    return goToPosition(rotations).andThen(Commands.waitUntil(this::isAtPosition));
  }

  public Command home() {
    return runOnce(() -> setVoltage(-3))
        .andThen(Commands.waitUntil(() -> tiltMotor.getStatorCurrent().getValueAsDouble() > 11.5))
        .andThen(runOnce(this::stop))
        .andThen(
            Commands.waitUntil(
                () -> Math.abs(tiltMotor.getStatorCurrent().getValueAsDouble()) < 0.2))
        .andThen(
            runOnce(
                () -> {
                  tiltMotor.setPosition(TiltConstants.kHardStopPosition);
                  isHomed = true;
                  SmartDashboard.putBoolean("ShooterTilt/isHomed", isHomed);
                }));
  }

  public boolean isNotAtIntakeHeight() {
    return TiltConstants.kIntakeAngle < tiltMotor.getPosition().getValueAsDouble();
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
            .withStatorCurrentLimit(TiltConstants.kStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true);
    FeedbackConfigs feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(TiltConstants.kGearRatio);
    Slot0Configs slot0config =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Arm_Cosine)
            .withKP(kp.get())
            .withKI(ki.get())
            .withKD(kd.get())
            .withKS(ks.get())
            .withKV(kv.get())
            .withKA(ka.get())
            .withKG(kg.get());
    MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicExpo_kA(profileKv.get())
            .withMotionMagicExpo_kV(profileKa.get());
    SoftwareLimitSwitchConfigs softlimitConfig =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(TiltConstants.kForwardSoftLimit)
            .withForwardSoftLimitEnable(false);
    tiltConfig =
        new TalonFXConfiguration()
            .withMotorOutput(motorOutputConfigs)
            .withSoftwareLimitSwitch(softlimitConfig)
            .withSlot0(slot0config)
            .withCurrentLimits(currentConfig)
            .withFeedback(feedbackConfig)
            .withMotionMagic(motionMagicConfigs);
    tiltMotor.getConfigurator().apply(tiltConfig);
  }
}
