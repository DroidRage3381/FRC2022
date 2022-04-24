// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import frc.robot.Constants;
import frc.robot.Constants.shooter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  TalonFX left_motor = new TalonFX(Constants.shooter.kleftTalon);
  TalonFX Right_motor = new TalonFX(Constants.shooter.kRightTalon);
  
  public Shooter() {
    left_motor.setNeutralMode(NeutralMode.Coast);
    Right_motor.setNeutralMode(NeutralMode.Coast);
  
    
  }

  /**
   * 
   * @param P // percent output
   */
  public void Spin(double P){
    left_motor.set(ControlMode.PercentOutput,P);
    Right_motor.set(ControlMode.PercentOutput,-P);
  }

  public double getRotations(){
    double SPMS = Right_motor.getSelectedSensorVelocity();
    double SPS = SPMS * 10;
    double RPM = (SPS/2048) * 60;
    return Math.floor(Math.abs(RPM));
  }

  public double getPercent(){
    
    return left_motor.getMotorOutputPercent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
