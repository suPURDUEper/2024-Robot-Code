// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.util.LoggedTunableNumber;
import org.surpurdueper.robot.Constants.CANIDs;
import org.surpurdueper.robot.Constants.ElevatorConstants;
import org.surpurdueper.robot.Constants.LookupTables;

public class Elevator extends SubsystemBase {

  // Class variables
  private TalonFX elevatorMotor;
  private TalonFXConfiguration elevatorConfig;
  private double targetHeight = -1;

  // Tunable numbers
  private static final LoggedTunableNumber kp = new LoggedTunableNumber("Elevator/Kp");
  private static final LoggedTunableNumber ki = new LoggedTunableNumber("Elevator/Ki");
  private static final LoggedTunableNumber kd = new LoggedTunableNumber("Elevator/Kd");
  private static final LoggedTunableNumber ks = new LoggedTunableNumber("Elevator/Ks");
  private static final LoggedTunableNumber kv = new LoggedTunableNumber("Elevator/Kv");
  private static final LoggedTunableNumber ka = new LoggedTunableNumber("Elevator/Ka");
  private static final LoggedTunableNumber kg = new LoggedTunableNumber("Elevator/Kg");
  private static final LoggedTunableNumber profileKv =
      new LoggedTunableNumber("Elevator/profileKv");
  private static final LoggedTunableNumber profileKa =
      new LoggedTunableNumber("Elevator/profileKa");
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
                this.setVoltage(volts.in(edu.wpi.first.units.Units.Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              null, // Using the CTRE SignalLogger API instead
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

  static {
    kp.initDefault(ElevatorConstants.kp);
    ki.initDefault(ElevatorConstants.ki);
    kd.initDefault(ElevatorConstants.kd);
    ks.initDefault(ElevatorConstants.kd);
    kv.initDefault(ElevatorConstants.kv);
    ka.initDefault(ElevatorConstants.ka);
    kg.initDefault(ElevatorConstants.kg);
    profileKv.initDefault(ElevatorConstants.profileKv);
    profileKa.initDefault(ElevatorConstants.profileKa);
    pidGains.addAll(List.of(kp, ki, kd, ks, kv, ka, kg, profileKa, profileKv));
  }

  public Elevator() {
    elevatorMotor = new TalonFX(CANIDs.kElevatorMotor, "canivore");
    configureTalonFx();
    elevatorMotor.setPosition(0);
    // setupSysIdTiming(elevatorMotor);
  }

  @Override
  public void periodic() {
    for (LoggedTunableNumber gain : pidGains) {
      if (gain.hasChanged(hashCode())) {
        // Send new PID gains to talon
        Slot0Configs slot0config =
            new Slot0Configs()
                .withGravityType(GravityTypeValue.Elevator_Static)
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
        elevatorMotor
            .getConfigurator()
            .apply(elevatorConfig.withSlot0(slot0config).withMotionMagic(motionMagicConfigs));
        break;
      }
    }

    // Log out to Glass for debugging
    double armPositionMotor = Units.metersToInches(elevatorMotor.getPosition().getValueAsDouble());
    double armPositionSetpoint =
        Units.metersToInches(elevatorMotor.getClosedLoopReference().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Position (Motor)", armPositionMotor);
    SmartDashboard.putNumber("Elevator/Target Position", armPositionSetpoint);
    SmartDashboard.putBoolean("Elevator/At Position", isAtPosition());
  }

  private void setupSysIdTiming(TalonFX motorToTest) {
    /* Speed up signals for better charaterization data */
    BaseStatusSignal.setUpdateFrequencyForAll(
        250, motorToTest.getPosition(), motorToTest.getVelocity(), motorToTest.getMotorVoltage());

    /* Optimize out the other signals, since they're not particularly helpful for us */
    motorToTest.optimizeBusUtilization();
  }

  public void setVoltage(double volts) {
    elevatorMotor.setControl(voltageRequest.withOutput(volts));
  }

  public void setPositionMeters(double position) {
    targetHeight = position;
    elevatorMotor.setControl(positionRequest.withPosition(targetHeight));
  }

  public void stop() {
    elevatorMotor.setControl(stopRequest);
  }

  public boolean isAtPosition() {
    return Math.abs(targetHeight - elevatorMotor.getPosition().getValueAsDouble())
        < ElevatorConstants.kPositionTolerance;
  }

  public Command goToPosition(double meters) {
    return Commands.runOnce(() -> setPositionMeters(meters));
  }

  public void followShooter(double shooterAngle) {
    double elevatorHeight = LookupTables.elevatorShooterClearance.get(shooterAngle);
    setPositionMeters(elevatorHeight);
  }

  public Command followShooter(DoubleSupplier shooterAngle) {
    return Commands.run(() -> followShooter(shooterAngle.getAsDouble()), this);
  }

  public Command goToPositionBlocking(double meters) {
    return goToPosition(meters).andThen(Commands.waitUntil(this::isAtPosition));
  }

  public Command waitUntilAtPosition() {
    return Commands.waitUntil(this::isAtPosition);
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
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
    CurrentLimitsConfigs currentConfig =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(ElevatorConstants.kStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true);
    FeedbackConfigs feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(ElevatorConstants.kMetersPerRotation);
    Slot0Configs slot0config =
        new Slot0Configs()
            .withGravityType(GravityTypeValue.Elevator_Static)
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
            .withForwardSoftLimitThreshold(ElevatorConstants.kForwardSoftLimit)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ElevatorConstants.kReverseSoftLimit)
            .withReverseSoftLimitEnable(true);
    elevatorConfig =
        new TalonFXConfiguration()
            .withMotorOutput(motorOutputConfigs)
            .withSoftwareLimitSwitch(softlimitConfig)
            .withSlot0(slot0config)
            .withCurrentLimits(currentConfig)
            .withFeedback(feedbackConfig)
            .withMotionMagic(motionMagicConfigs);
    elevatorMotor.getConfigurator().apply(elevatorConfig);
  }
}
