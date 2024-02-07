// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.util.LoggedTunableNumber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.DIOPorts;
import frc.robot.Constants.TiltConstants;

public class ShooterTilt extends SubsystemBase {

  TalonFX tiltMotor;
  DutyCycleEncoder tiltAbsoluteEncoder;
  TalonFXConfiguration tiltConfig;

  private static final LoggedTunableNumber kp = new LoggedTunableNumber("ShooterTilt/Kp");
  private static final LoggedTunableNumber ki = new LoggedTunableNumber("ShooterTilt/Ki");
  private static final LoggedTunableNumber kd = new LoggedTunableNumber("ShooterTilt/Kd");
  private static final LoggedTunableNumber ks = new LoggedTunableNumber("ShooterTilt/Ks");
  private static final LoggedTunableNumber kv = new LoggedTunableNumber("ShooterTilt/Kv");
  private static final LoggedTunableNumber ka = new LoggedTunableNumber("ShooterTilt/Ka");
  private static final LoggedTunableNumber kg = new LoggedTunableNumber("ShooterTilt/Kg");
  private static final List<LoggedTunableNumber> pidGains = new ArrayList<>();

  static {
    kp.initDefault(0.0);
    ki.initDefault(0.0);
    kd.initDefault(0.0);
    ks.initDefault(0.0);
    kv.initDefault(0.0);
    ka.initDefault(0.0);
    kg.initDefault(0.0);
    pidGains.addAll(List.of(kp, ki, kd, ks, kv, ka, kg));
  }

  public ShooterTilt() {
    tiltMotor = new TalonFX(CANIDs.kTiltMotor);
    configureTalonFx(tiltMotor);
    tiltAbsoluteEncoder = new DutyCycleEncoder(DIOPorts.kTiltEncoder);
    // tilt returns radians :)
    tiltAbsoluteEncoder.setDistancePerRotation(
        (Constants.TiltConstants.kAbsoluteEncoderInverted ? -1 : 1) * 2 * Math.PI);
    tiltAbsoluteEncoder.setPositionOffset(TiltConstants.kAbsoluteEncoderOffset);
  }

  @Override
  public void periodic() {
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
        tiltMotor.getConfigurator().apply(slot0config);
        break;
      }
    }
    // This method will be called once per scheduler run
  }

  public void configureTalonFx(TalonFX tiltMotor) {
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
    SoftwareLimitSwitchConfigs softlimitConfig =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(TiltConstants.kForwardSoftLimit)
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitThreshold(TiltConstants.kReverseSoftLimit)
            .withReverseSoftLimitEnable(false);
    tiltConfig = new TalonFXConfiguration();
  }
}
