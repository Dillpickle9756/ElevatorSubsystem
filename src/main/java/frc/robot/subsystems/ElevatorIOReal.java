// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

//import edu.wpi.first.units.measure.Angle;
//import edu.wpi.first.units.measure.AngularVelocity;
//import edu.wpi.first.units.measure.Current;
//import edu.wpi.first.units.measure.Temperature;
//import edu.wpi.first.units.measure.Voltage;



public class ElevatorIOReal implements ElevatorIO {
TalonFX leadTalon = new TalonFX(ElevatorSubsystem.LEAD_MOTOR_ID);
TalonFX followTalon = new TalonFX(ElevatorSubsystem.FOLLOW_MOTOR_ID);
public VoltageOut voltageOut = new VoltageOut(0.0).withEnableFOC(true);
MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);

  /** Creates a new ElevatorIOReal. */
  public ElevatorIOReal() {
    TalonFXConfiguration configs = new TalonFXConfiguration();
    var slot0Configs = configs.Slot0;

    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    leadTalon.getConfigurator().apply(configs);
    followTalon.getConfigurator().apply(configs);

    var motionMagicConfigs = configs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

  }

  @Override
  public void setPosition(double height) {
    //Positon in the next line is in desired rotations. Need to figure out how to get that to height.
    leadTalon.setControl(motionMagic.withPosition(height));
    followTalon.setControl(new Follower(leadTalon.getDeviceID(), false));
  }
  @Override
  public void setVolts(double leadVolts){
    leadTalon.setControl(voltageOut.withOutput(leadVolts));
    followTalon.setControl(new Follower(leadTalon.getDeviceID(), false));
  }
}
