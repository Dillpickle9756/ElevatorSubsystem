// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
//import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO {

TalonFX leadTalon = new TalonFX(ElevatorSubsystem.LEAD_MOTOR_ID);
TalonFX followTalon = new TalonFX(ElevatorSubsystem.FOLLOW_MOTOR_ID);

VoltageOut leadVoltage = new VoltageOut(0);
VoltageOut followVoltage = new VoltageOut(0);
final DutyCycleOut m_request = new DutyCycleOut(0);
DifferentialDrivetrainSim physicsSim = DifferentialDrivetrainSim.createKitbotSim(KitbotMotor.kDoubleFalcon500PerSide,KitbotGearing.k8p45,KitbotWheelSize.kSixInch,null);

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        physicsSim.update(0.020);

        var leadSimState = leadTalon.getSimState();
        leadSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());

        var followSimState = followTalon.getSimState();
        followSimState.setSupplyVoltage(RoboRioSim.getVInVoltage());

        physicsSim.setInputs(leadSimState.getMotorVoltage(), followSimState.getMotorVoltage());

        inputs.leadOutputVolts = leadSimState.getMotorVoltage();
        inputs.followOutputVolts = followSimState.getMotorVoltage();

        inputs.leadCurrentAmps = leadSimState.getTorqueCurrent();
        inputs.leadTempCelsius = 0.0;
        inputs.followCurrentAmps = followSimState.getTorqueCurrent();
        inputs.followTempCelsius = 0.0;

        inputs.leadVelocityMetersPerSecond = physicsSim.getLeftVelocityMetersPerSecond();
        inputs.followVelocityMetersPerSecond = physicsSim.getRightVelocityMetersPerSecond();
    
        inputs.leadPositionMeters = physicsSim.getLeftPositionMeters();
        inputs.followPositionMeters = physicsSim.getRightPositionMeters();
    }

    @Override
    public void setPosition(double height) {
        
    }
    @Override
    public void setVolts(double leadVolts){

    }
}