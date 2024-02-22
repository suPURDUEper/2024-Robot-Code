// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.surpurdueper.robot.Constants.AmpConstants;
import org.surpurdueper.robot.Constants.CANIDs;
import org.surpurdueper.robot.Constants.DIOPorts;

public class Amp extends SubsystemBase {

  private TalonFX ampMotor;
  private DigitalInput ampBreakBeam;
  private VoltageOut voltageRequest = new VoltageOut(0);
  private StaticBrake stopRequest = new StaticBrake();
  private CoastOut coastRequest = new CoastOut();

  public Amp() {
    ampMotor = new TalonFX(CANIDs.kAmpMotor);
    ampBreakBeam = new DigitalInput(DIOPorts.kAmpBreakBeam);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Intake/BreakBeam", ampBreakBeam.get());
  }

  public Command load() {
    return Commands.startEnd(
            () -> ampMotor.setControl(voltageRequest.withOutput(AmpConstants.kLoadVoltage)),
            () -> ampMotor.setControl(stopRequest),
            this)
        .until(this::isAmpLoaded);
  }

  public Command score() {
    return Commands.startEnd(
            () -> ampMotor.setControl(voltageRequest.withOutput(AmpConstants.kScoreVoltage)),
            () -> ampMotor.setControl(coastRequest),
            this)
        .withTimeout(0);
  }

  public boolean isAmpLoaded() {
    return !ampBreakBeam.get();
  }

  public void configureTalonFx(TalonFX motor) {
    MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive);
    CurrentLimitsConfigs currentConfig =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(AmpConstants.kStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(AmpConstants.kSupplyCurrentLimit)
            .withSupplyCurrentThreshold(AmpConstants.kSupplyCurrentLimitThreshold)
            .withSupplyTimeThreshold(AmpConstants.kSupplyTimeThreshold)
            .withSupplyCurrentLimitEnable(true);
    motor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withMotorOutput(motorOutputConfigs)
                .withCurrentLimits(currentConfig));
  }
}
