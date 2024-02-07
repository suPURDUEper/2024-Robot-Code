// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKV(0)
            .withKA(0)
            .withKG(0);
    SoftwareLimitSwitchConfigs softlimitConfig =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(TiltConstants.kForwardSoftLimit)
            .withForwardSoftLimitEnable(false)
            .withReverseSoftLimitThreshold(TiltConstants.kReverseSoftLimit)
            .withReverseSoftLimitEnable(false);
    tiltConfig = new TalonFXConfiguration();
  }
}
