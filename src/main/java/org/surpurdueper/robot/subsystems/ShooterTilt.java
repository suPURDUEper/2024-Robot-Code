// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.util.LoggedTunableNumber;
import org.surpurdueper.robot.Constants;
import org.surpurdueper.robot.Constants.CANIDs;
import org.surpurdueper.robot.Constants.DIOPorts;
import org.surpurdueper.robot.Constants.TiltConstants;

public class ShooterTilt extends SubsystemBase {

  // Class variables
  private TalonFX tiltMotor;
  private DutyCycleEncoder tiltAbsoluteEncoder;
  private TalonFXConfiguration tiltConfig;

  // Tunable numbers
  private static final LoggedTunableNumber kp = new LoggedTunableNumber("ShooterTilt/Kp");
  private static final LoggedTunableNumber ki = new LoggedTunableNumber("ShooterTilt/Ki");
  private static final LoggedTunableNumber kd = new LoggedTunableNumber("ShooterTilt/Kd");
  private static final LoggedTunableNumber ks = new LoggedTunableNumber("ShooterTilt/Ks");
  private static final LoggedTunableNumber kv = new LoggedTunableNumber("ShooterTilt/Kv");
  private static final LoggedTunableNumber ka = new LoggedTunableNumber("ShooterTilt/Ka");
  private static final LoggedTunableNumber kg = new LoggedTunableNumber("ShooterTilt/Kg");
  private static final LoggedTunableNumber profileKv = new LoggedTunableNumber("ShooterTilt/Kv");
  private static final LoggedTunableNumber profileKa = new LoggedTunableNumber("ShooterTilt/Ka");
  private static final List<LoggedTunableNumber> pidGains = new ArrayList<>();

  // Control requests
  private final ControlRequest voltagRequest = new VoltageOut(0);
  private final ControlRequest torqueRequest =
      new MotionMagicExpoTorqueCurrentFOC(0, 0, 0, false, false, false);

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

  public ShooterTilt() {
    tiltMotor = new TalonFX(CANIDs.kTiltMotor);
    configureTalonFx(tiltMotor);
    tiltAbsoluteEncoder = new DutyCycleEncoder(DIOPorts.kTiltEncoder);
    // tilt returns rotations :)
    tiltAbsoluteEncoder.setDistancePerRotation(
        Constants.TiltConstants.kAbsoluteEncoderInverted ? -1 : 1);
    tiltAbsoluteEncoder.setPositionOffset(TiltConstants.kAbsoluteEncoderOffset);
    tiltMotor.setPosition(tiltAbsoluteEncoder.getAbsolutePosition());
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
    MotionMagicConfigs motionMagicConfigs =
        new MotionMagicConfigs()
            .withMotionMagicExpo_kA(profileKv.get())
            .withMotionMagicExpo_kV(profileKa.get());
    SoftwareLimitSwitchConfigs softlimitConfig =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(TiltConstants.kForwardSoftLimit)
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitThreshold(TiltConstants.kReverseSoftLimit)
            .withReverseSoftLimitEnable(false);
    tiltConfig =
        new TalonFXConfiguration()
            .withSoftwareLimitSwitch(softlimitConfig)
            .withSlot0(slot0config)
            .withCurrentLimits(currentConfig)
            .withFeedback(feedbackConfig)
            .withMotionMagic(motionMagicConfigs);
    tiltMotor.getConfigurator().apply(tiltConfig);
  }
}
