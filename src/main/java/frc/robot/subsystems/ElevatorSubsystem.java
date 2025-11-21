// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
//import edu.wpi.first.math.util.Units;

//import com.ctre.phoenix6.controls.VoltageOut;
//import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  public static final int LEAD_MOTOR_ID = 1;
  public static final int FOLLOW_MOTOR_ID = 2;

  ElevatorIO io = Robot.isReal() ? new ElevatorIOReal() : new ElevatorIOSim();
  ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  /** Creates a new DrivetrainSubsystem. */
  public ElevatorSubsystem() {

  }
  private void setVolts(double lead, double follow){
    io.setVolts(lead,follow);
  }
  public Command setVoltagesCommand(DoubleSupplier left, DoubleSupplier right) {
    return this.run(() -> this.setVolts(left.getAsDouble(), right.getAsDouble()));
    }
  public Command setVoltagesArcadeCommand(DoubleSupplier drive, DoubleSupplier steer) {
    return this.run(() -> {
      var speeds = DifferentialDrive.arcadeDriveIK(drive.getAsDouble(), steer.getAsDouble(), false);
      this.setVolts(speeds.left * 12, speeds.right * 12);
    });
  }
  private void setPosition(double height){
    io.setPosition(height);
  }
  public Command setPositionCommand(DoubleSupplier height) {
  return this.run(() -> this.setPosition(height.getAsDouble()));
  }
  public Command setPositionArcadeCommand(DoubleSupplier drive, DoubleSupplier steer) {
  return this.run(() -> {
    var speeds = DifferentialDrive.arcadeDriveIK(drive.getAsDouble(), steer.getAsDouble(), false);
    this.setVolts(speeds.left * 12, speeds.right * 12);
  });
}
  
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (Robot.isSimulation()) {
  }
}
}
