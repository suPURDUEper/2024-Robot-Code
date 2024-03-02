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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.util.LoggedTunableNumber;
import org.surpurdueper.robot.Constants.CANIDs;
import org.surpurdueper.robot.Constants.ClimberConstants;
import org.surpurdueper.robot.Constants.DIOPorts;

public class Climber extends SubsystemBase {

  // Class variables
  private TalonFX climberMotor;
  private TalonFX climberFollower;
  private DutyCycleEncoder tiltAbsoluteEncoder;
  private TalonFXConfiguration climberConfig;
  private double targetRotations = -1;

  // Tunable numbers
  private static final LoggedTunableNumber kp = new LoggedTunableNumber("Climber/Kp");
  private static final LoggedTunableNumber ki = new LoggedTunableNumber("Climber/Ki");
  private static final LoggedTunableNumber kd = new LoggedTunableNumber("Climber/Kd");
  private static final LoggedTunableNumber ks = new LoggedTunableNumber("Climber/Ks");
  private static final LoggedTunableNumber kv = new LoggedTunableNumber("Climber/Kv");
  private static final LoggedTunableNumber ka = new LoggedTunableNumber("Climber/Ka");
  private static final LoggedTunableNumber kg = new LoggedTunableNumber("Climber/Kg");
  private static final LoggedTunableNumber profileKv = new LoggedTunableNumber("Climber/profileKv");
  private static final LoggedTunableNumber profileKa = new LoggedTunableNumber("Climber/profileKa");
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
                climberMotor.setControl(
                    voltageRequest.withOutput(volts.in(edu.wpi.first.units.Units.Volts)));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              null, // Using the CTRE SignalLogger API instead
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

  static {
    kp.initDefault(ClimberConstants.kp);
    ki.initDefault(ClimberConstants.ki);
    kd.initDefault(ClimberConstants.kd);
    ks.initDefault(ClimberConstants.ks);
    kv.initDefault(ClimberConstants.kv);
    ka.initDefault(ClimberConstants.ka);
    kg.initDefault(ClimberConstants.kg);
    profileKv.initDefault(ClimberConstants.profileKv);
    profileKa.initDefault(ClimberConstants.profileKa);
    pidGains.addAll(List.of(kp, ki, kd, ks, kv, ka, kg, profileKa, profileKv));
  }

  public Climber() {
    climberMotor = new TalonFX(CANIDs.kClimberMotor, "canivore");
    climberFollower = new TalonFX(CANIDs.kClimber2Motor, "canivore");
    tiltAbsoluteEncoder = new DutyCycleEncoder(DIOPorts.kTiltEncoder);
    tiltAbsoluteEncoder.setDistancePerRotation(ClimberConstants.kAbsoluteEncoderInverted ? -1 : 1);
    tiltAbsoluteEncoder.setPositionOffset(ClimberConstants.kAbsoluteEncoderOffset);
    configureTalonFx();
    // setupSysIdTiming(climberMotor);
  }

  @Override
  public void periodic() {
    // Update tunable numbers
    if (Math.abs(getAbsoluteSensorAngle() - climberMotor.getPosition().getValueAsDouble())
        > Units.degreesToRotations(1.0)) {
      climberMotor.setPosition(getAbsoluteSensorAngle());
    }

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
        climberMotor
            .getConfigurator()
            .apply(climberConfig.withSlot0(slot0config).withMotionMagic(motionMagicConfigs));
        break;
      }
    }

    // Log out to Glass for debugging
    double armPositionAbs = Units.rotationsToDegrees(getAbsoluteSensorAngle());
    double armPositionMotor =
        Units.rotationsToDegrees(climberMotor.getPosition().getValueAsDouble());
    double armPositionSetpoint =
        Units.rotationsToDegrees(climberMotor.getClosedLoopReference().getValueAsDouble());
    SmartDashboard.putNumber("Climber/Position (Abs)", armPositionAbs);
    SmartDashboard.putNumber("Climber/Position (Motor)", armPositionMotor);
    SmartDashboard.putNumber("Climber/Setpoint", targetRotations);
    SmartDashboard.putNumber("Climber/Profile Target", armPositionSetpoint);
    SmartDashboard.putBoolean("Climber/AtPosition", isAtPosition());
  }

  private void setupSysIdTiming(TalonFX motorToTest) {
    /* Speed up signals for better charaterization data */
    BaseStatusSignal.setUpdateFrequencyForAll(
        250, motorToTest.getPosition(), motorToTest.getVelocity(), motorToTest.getMotorVoltage());

    /* Optimize out the other signals, since they're not particularly helpful for us */
    motorToTest.optimizeBusUtilization();
  }

  public double getAbsoluteSensorAngle() {
    double wrappedAngle =
        MathUtil.angleModulus(Units.rotationsToRadians(tiltAbsoluteEncoder.getDistance()));
    return Units.radiansToRotations(wrappedAngle);
  }

  public void setVoltage(double volts) {
    climberMotor.setControl(voltageRequest.withOutput(volts));
  }

  public void setPositionRads(double position) {
    setPositionRotations(Units.radiansToRotations(position));
  }

  public void setPositionDegrees(double position) {
    setPositionRotations(Units.degreesToRotations(position));
  }

  public void setPositionRotations(double position) {
    targetRotations = position;
    climberMotor.setControl(positionRequest.withPosition(targetRotations));
  }

  public double getPositionRotations() {
    return climberMotor.getPosition().getValueAsDouble();
  }

  public StatusSignal<Double> getPositionSignal() {
    return climberMotor.getPosition();
  }

  public StatusSignal<Double> getVelocitySignal() {
    return climberMotor.getVelocity();
  }

  public void stop() {
    climberMotor.setControl(stopRequest);
  }

  public boolean isAtPosition() {
    return Math.abs(targetRotations - climberMotor.getPosition().getValueAsDouble())
        < ClimberConstants.kPositionTolerance;
  }

  public Command goToPosition(double rotations) {
    return Commands.runOnce(() -> setPositionRotations(rotations), this);
  }

  public Command goToPositionBlocking(double rotations) {
    return Commands.runOnce(() -> setPositionRotations(rotations), this)
        .andThen(Commands.waitUntil(this::isAtPosition));
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
            .withInverted(InvertedValue.CounterClockwise_Positive);
    CurrentLimitsConfigs currentConfig =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(ClimberConstants.kStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true);
    FeedbackConfigs feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(ClimberConstants.kGearRatio);
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
            .withForwardSoftLimitThreshold(ClimberConstants.kForwardSoftLimit)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ClimberConstants.kReverseSoftLimit)
            .withReverseSoftLimitEnable(true);
    climberConfig =
        new TalonFXConfiguration()
            .withMotorOutput(motorOutputConfigs)
            .withSoftwareLimitSwitch(softlimitConfig)
            .withSlot0(slot0config)
            .withCurrentLimits(currentConfig)
            .withFeedback(feedbackConfig)
            .withMotionMagic(motionMagicConfigs);
    climberMotor.getConfigurator().apply(climberConfig);
    climberFollower.getConfigurator().apply(new TalonFXConfiguration());
    climberFollower.setControl(
        new Follower(climberMotor.getDeviceID(), true).withUpdateFreqHz(250));
  }
}
