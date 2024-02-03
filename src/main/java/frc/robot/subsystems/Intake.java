package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.CANIDs;
import static frc.robot.Constants.DIOPorts;

public class Intake extends SubsystemBase {

    CANSparkMax intakeMotor = new CANSparkMax(CANIDs.kIntakeMotor, MotorType.kBrushless);
    CANSparkMax feederMotor = new CANSparkMax(CANIDs.kFeederMotor, MotorType.kBrushless);
    DigitalInput feederBreakBeam = new DigitalInput(DIOPorts.kFeederBreakBeam);

    public Command intake() {
        return Commands.startEnd(
            () -> {
                intakeMotor.setVoltage(12);
                feederMotor.setVoltage(12);
            }, 
            () -> {
                intakeMotor.stopMotor();;
                feederMotor.stopMotor();;
            }, 
            this);
    }
    
}
