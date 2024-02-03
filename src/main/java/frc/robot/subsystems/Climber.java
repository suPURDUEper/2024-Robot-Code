// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxExtensions;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMax.FrameStrategy;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  CANSparkMax climberMotor;

  CANSparkMax climber2Motor;
  SparkAbsoluteEncoder climbAbsoluteEncoder = climber2Motor.getAbsoluteEncoder(Type.kDutyCycle);

  public Climber() {
    climberMotor =
        new SparkMax(CANIDs.kClimberMotor, MotorType.kBrushless)
            .withInitializer(Climber::sparkMaxInitializer);
    climber2Motor =
        new SparkMax(CANIDs.kClimber2Motor, MotorType.kBrushless)
            .withInitializer(Climber::sparkMaxInitializer);
    ;
    climbAbsoluteEncoder = climber2Motor.getAbsoluteEncoder(Type.kDutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private static Boolean sparkMaxInitializer(CANSparkMax sparkMax, Boolean isInit) {
   int errors = 0;
    AbsoluteEncoder encoder = sparkMax.getAbsoluteEncoder(com.revrobotics.SparkAbsoluteEncoder.Type.kDutyCycle);
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo(sparkMax));
    errors += SparkMaxUtils.check(CANSparkMaxExtensions.setInverted(sparkMax, false));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(
                SoftLimitDirection.kForward, (float) ClimberConstants.kForwardSoftLimit));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(
                SoftLimitDirection.kReverse, (float) ClimberConstants.kReverseSoftLimit));
    SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kPosition);
    return errors == 0;
  }

  
  }

