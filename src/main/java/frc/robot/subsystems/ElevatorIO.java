// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

/** Add your docs here. */
public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        //Output Voltages
        public double leftOutputVolts = 0.0;
        public double rightOutputVolts = 0.0;

        //Current Velocities
        public double leftVelocityMetersPerSecond = 0.0;
        public double rightVelocityMetersPerSecond = 0.0;

        //Current Position
        public double leftPositionMeters = 0.0;            public double rightPositionMeters = 0.0;
    
        //Motor Currents
        public double leftCurrentAmps = 0.0;
        public double rightCurrentAmps = 0.0;
    }
        public void updateInputs(ElevatorIOInputs inputs);
        public void setPosition(double height);
}
