// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxExtensions;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMax.FrameStrategy;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.TiltConstants;

public class ShooterTilt extends SubsystemBase {

  SparkMax tiltMotor;
  SparkAbsoluteEncoder climbAbsoluteEncoder;

  public ShooterTilt() {
    tiltMotor = new SparkMax(CANIDs.kTiltMotor).withInitializer(ShooterTilt::sparkMaxInitializer);
    climbAbsoluteEncoder = tiltMotor.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private static Boolean sparkMaxInitializer(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    AbsoluteEncoder encoder =
        sparkMax.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo(sparkMax));
    errors += SparkMaxUtils.check(CANSparkMaxExtensions.setInverted(sparkMax, false));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(
                SoftLimitDirection.kForward, (float) TiltConstants.kForwardSoftLimit));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(
                SoftLimitDirection.kReverse, (float) TiltConstants.kReverseSoftLimit));
    errors += SparkMaxUtils.check(encoder.setZeroOffset(TiltConstants.kAbsoluteEncoderOffset));
    errors +=
        SparkMaxUtils.check(
            encoder.setPositionConversionFactor(TiltConstants.kEncoderPositionConversion));
    errors += SparkMaxUtils.check(encoder.setInverted(TiltConstants.kAbsoluteEncoderInverted));
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kBrake));
    SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kPosition);
    return errors == 0;
  }
}
