// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxExtensions;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMax.FrameStrategy;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.robot.Constants.CANIDs;
import com.revrobotics.SparkAbsoluteEncoder;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
 CANSparkMax climberMotor = new CANSparkMax(CANIDs.kclimberMotor, MotorType.kBrushless);
 CANSparkMax climber2Motor = new CANSparkMax(CANIDs.kclimber2Motor, MotorType.kBrushless);
 SparkAbsoluteEncoder climbAbsoluteEncoder = climber2Motor.getAbsoluteEncoder(Type.kDutyCycle);
  
 
  public Climber() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
    private static Boolean sparkMaxInitializer(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo(sparkMax));
    errors += SparkMaxUtils.check(CANSparkMaxExtensions.setInverted(sparkMax, false));
    SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kNoFeedback);
    return errors == 0;
  }
}
