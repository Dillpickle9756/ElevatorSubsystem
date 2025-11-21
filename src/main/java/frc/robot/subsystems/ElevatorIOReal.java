// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;

//import edu.wpi.first.units.measure.Angle;
//import edu.wpi.first.units.measure.AngularVelocity;
//import edu.wpi.first.units.measure.Current;
//import edu.wpi.first.units.measure.Temperature;
//import edu.wpi.first.units.measure.Voltage;



public class ElevatorIOReal implements ElevatorIO {
TalonFX leadTalon = new TalonFX(ElevatorSubsystem.LEAD_MOTOR_ID);
TalonFX followTalon = new TalonFX(ElevatorSubsystem.FOLLOW_MOTOR_ID);
  /** Creates a new ElevatorIOReal. */
  public ElevatorIOReal() {}

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
  }

  @Override
  public void setPosition(double height) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setPosition'");
  }
  @Override
  public void setVolts(double lead, double follow){
    
  }
}
