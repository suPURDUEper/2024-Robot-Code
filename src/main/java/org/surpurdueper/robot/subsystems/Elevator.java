// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.surpurdueper.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxExtensions;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.frc3005.lib.vendor.motorcontroller.SparkMax;
import org.frc3005.lib.vendor.motorcontroller.SparkMax.FrameStrategy;
import org.frc3005.lib.vendor.motorcontroller.SparkMaxUtils;
import org.surpurdueper.robot.Constants.CANIDs;
import org.surpurdueper.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  CANSparkMax elevatorMotor;

  public Elevator() {
    elevatorMotor =
        new SparkMax(CANIDs.kElevatorMotor).withInitializer(Elevator::sparkMaxInitializer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private static Boolean sparkMaxInitializer(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    RelativeEncoder encoder = sparkMax.getEncoder();
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo(sparkMax));
    errors += SparkMaxUtils.check(CANSparkMaxExtensions.setInverted(sparkMax, false));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(
                SoftLimitDirection.kForward, (float) ElevatorConstants.kForwardSoftLimit));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSoftLimit(
                SoftLimitDirection.kReverse, (float) ElevatorConstants.kReverseSoftLimit));
    errors +=
        SparkMaxUtils.check(
            encoder.setPositionConversionFactor(ElevatorConstants.kMetersPerRotation));
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kBrake));
    SparkMax.setFrameStrategy(sparkMax, FrameStrategy.kPosition);
    return errors == 0;
  }
}
