// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;
import frc.robot.Constants.DIOPorts;

public class Amp extends SubsystemBase {
  /** Creates a new Amp. */
   CANSparkMax ampMotor = new CANSparkMax(CANIDs.kIntakeMotor, MotorType.kBrushless);
    DigitalInput ampBreakBeam = new DigitalInput(DIOPorts.kAmpBreakBeam);
  public Amp() {}

  @Override
  public void periodic() {
   // This method will be called once per scheduler run
  }
 public Command amp() {
   return Commands.startEnd(
            () -> {
                ampMotor.setVoltage(12);
                
            }, 
            () -> {
                ampMotor.stopMotor();;
            
            }, 
            this);
 }
  
}
