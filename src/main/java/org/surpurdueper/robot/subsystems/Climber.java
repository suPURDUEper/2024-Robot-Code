package org.surpurdueper.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.util.LoggedTunableNumber;
import org.surpurdueper.robot.Constants;
import org.surpurdueper.robot.Constants.CANIDs;
import org.surpurdueper.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  private final TalonFX climberMotor;
  private final TalonFX climberFollower;

  private final DutyCycleEncoder absoluteEncoder;

  private MotionMagicExpoVoltage positionRequest = new MotionMagicExpoVoltage(0);
  private VoltageOut voltageRequest = new VoltageOut(0);
  private ControlRequest stopRequest = new StaticBrake();
  private Follower followRequest;
  private TalonFXConfiguration climberConfig;
  private DigitalInput bottomLimit1;
  private DigitalInput bottomLimit2;

  // Tunable numbers
  private static final LoggedTunableNumber kp = new LoggedTunableNumber("Climber/Kp");
  private static final LoggedTunableNumber ki = new LoggedTunableNumber("Climber/Ki");
  private static final LoggedTunableNumber kd = new LoggedTunableNumber("Climber/Kd");
  private static final LoggedTunableNumber ks = new LoggedTunableNumber("Climber/Ks");
  private static final LoggedTunableNumber kv = new LoggedTunableNumber("Climber/Kv");
  private static final LoggedTunableNumber ka = new LoggedTunableNumber("Climber/Ka");
  private static final LoggedTunableNumber kg = new LoggedTunableNumber("Climber/Kg");
  private static final LoggedTunableNumber profileKv = new LoggedTunableNumber("Climber/profileKv");
  private static final LoggedTunableNumber profileKa = new LoggedTunableNumber("Climber/profileKa");
  private static final List<LoggedTunableNumber> pidGains = new ArrayList<>();

  // SysID Setup
  private final SysIdRoutine sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(
              null,
              null,
              null,
              state -> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                setVoltage(volts.in(edu.wpi.first.units.Units.Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              null, // Using the CTRE SignalLogger API instead
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

  static {
    kp.initDefault(ClimberConstants.kp);
    ki.initDefault(ClimberConstants.ki);
    kd.initDefault(ClimberConstants.kd);
    ks.initDefault(ClimberConstants.ks);
    kv.initDefault(ClimberConstants.kv);
    ka.initDefault(ClimberConstants.ka);
    kg.initDefault(ClimberConstants.kg);
    profileKv.initDefault(ClimberConstants.profileKv);
    profileKa.initDefault(ClimberConstants.profileKa);
    pidGains.addAll(List.of(kp, ki, kd, ks, kv, ka, kg, profileKa, profileKv));
  }

  public Climber() {
    this.climberMotor = new TalonFX(CANIDs.kClimberMotor, "canivore");
    this.climberFollower = new TalonFX(CANIDs.kClimber2Motor, "canivore");
    configureTalonFX();
    followRequest = new Follower(climberMotor.getDeviceID(), true).withUpdateFreqHz(250);
    absoluteEncoder = new DutyCycleEncoder(Constants.DIOPorts.kClimberEncoder);
    this.bottomLimit1 = new DigitalInput(Constants.DIOPorts.kClimberLimit1);
    this.bottomLimit2 = new DigitalInput(Constants.DIOPorts.kClimberLimit2);

  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()
        && Math.abs(getPositionRotations() - getAbsoluteSensorAngle())
            > Units.degreesToRotations(1)) {
      syncMotorAndAbsEncoder();
    }
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
        climberMotor
            .getConfigurator()
            .apply(climberConfig.withSlot0(slot0config).withMotionMagic(motionMagicConfigs));
        break;
      }
    }

    // Log out to Glass for debugging
    double rawAbsSensor = Units.rotationsToDegrees(rawAbsSensorAngle());
    double armPositionAbs = Units.rotationsToDegrees(getAbsoluteSensorAngle());
    double armPositionMotor =
        Units.rotationsToDegrees(climberMotor.getPosition().getValueAsDouble());
    double armPositionSetpoint =
        Units.rotationsToDegrees(climberMotor.getClosedLoopReference().getValueAsDouble());
    SmartDashboard.putNumber("Climber/Position (Abs)", armPositionAbs);
    SmartDashboard.putNumber("Climber/Position (Motor)", armPositionMotor);
    SmartDashboard.putNumber("Climber/Profile Target", armPositionSetpoint);
    SmartDashboard.putNumber("Climber/Raw Abs Sensor", rawAbsSensor);
    SmartDashboard.putBoolean("Climber/Limit 1", bottomLimit1.get());
    SmartDashboard.putBoolean("Climber/Limit 2", bottomLimit2.get());
  }

  private void setupSysIdTiming(TalonFX motorToTest) {
    /* Speed up signals for better charaterization data */
    BaseStatusSignal.setUpdateFrequencyForAll(
        250, motorToTest.getPosition(), motorToTest.getVelocity(), motorToTest.getMotorVoltage());

    /* Optimize out the other signals, since they're not particularly helpful for us */
    motorToTest.optimizeBusUtilization();
  }

  public double rawAbsSensorAngle() {
    return Units.radiansToRotations(
        MathUtil.angleModulus(Units.rotationsToRadians(absoluteEncoder.getDistance())));
  }

  public double getAbsoluteSensorAngle() {
    double zeroedSensorAngle =
        Rotation2d.fromRotations(rawAbsSensorAngle())
            .rotateBy(Rotation2d.fromRotations(-1 * ClimberConstants.kAbsoluteEncoderOffset))
            .getRotations();

    if (zeroedSensorAngle > Units.degreesToRotations(40)) {
      zeroedSensorAngle--;
    }
    Rotation2d armAngle =
        Rotation2d.fromRotations(zeroedSensorAngle * -1 / ClimberConstants.kSprocketGearRatio);
    armAngle =
        armAngle.rotateBy(Rotation2d.fromDegrees(-8.8)); // Zeroed against hardstop at -8.8 degrees.
    return armAngle.getRotations();
  }

  public boolean bottomLimit() {
    return !bottomLimit1.get() || !bottomLimit2.get();
  }

  public void syncMotorAndAbsEncoder() {
    climberMotor.setPosition(getAbsoluteSensorAngle());
  }

  public double getPositionRotations() {
    return climberMotor.getPosition().getValueAsDouble();
  }

  public void setPositionRotations(double rotations) {
    climberMotor.setControl(positionRequest.withPosition(rotations));
    climberFollower.setControl(followRequest);
  }

  public void setVoltage(double volts) {
    climberMotor.setControl(voltageRequest.withOutput(volts));
    climberFollower.setControl(followRequest);
  }

  public void stop() {
    climberMotor.setControl(stopRequest);
    climberFollower.setControl(stopRequest);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  private void configureTalonFX() {
    MotorOutputConfigs motorOutputConfigs =
        new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive);
    CurrentLimitsConfigs currentConfig =
        new CurrentLimitsConfigs()
            .withStatorCurrentLimit(ClimberConstants.kStatorCurrentLimit)
            .withStatorCurrentLimitEnable(true);
    FeedbackConfigs feedbackConfig =
        new FeedbackConfigs().withSensorToMechanismRatio(ClimberConstants.kGearRatio);
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
            .withForwardSoftLimitThreshold(ClimberConstants.kForwardSoftLimit)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(ClimberConstants.kReverseSoftLimit)
            .withReverseSoftLimitEnable(true);
    climberConfig =
        new TalonFXConfiguration()
            .withMotorOutput(motorOutputConfigs)
            .withSoftwareLimitSwitch(softlimitConfig)
            .withCurrentLimits(currentConfig)
            .withFeedback(feedbackConfig)
            .withSlot0(slot0config)
            .withMotionMagic(motionMagicConfigs);
    climberMotor.getConfigurator().apply(climberConfig);
    climberFollower.getConfigurator().apply(new TalonFXConfiguration());
  }
}
