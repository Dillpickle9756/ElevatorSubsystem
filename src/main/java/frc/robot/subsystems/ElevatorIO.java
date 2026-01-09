// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        //Output Voltages
        public double leadOutputVolts = 0.0;
        public double followOutputVolts = 0.0;

        //Current Velocities
        public double leadVelocityMetersPerSecond = 0.0;
        public double followVelocityMetersPerSecond = 0.0;

        //Current Position
        public double leadPositionMeters = 0.0;            
        public double followPositionMeters = 0.0;
    
        //Motor Currents
        public double leadCurrentAmps = 0.0;
        public double followCurrentAmps = 0.0;

        //Temperature
        public double leadTempCelsius = 0.0;
        public double followTempCelsius = 0.0;
    }
        public void updateInputs(ElevatorIOInputs inputs);
        public void setPosition(double height);
        public void setVolts(double leadVolts);
}

