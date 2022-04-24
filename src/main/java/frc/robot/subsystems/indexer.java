// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class indexer extends SubsystemBase {

  TalonSRX frontConveyer;
  CANSparkMax backConveyer;
  CANSparkMax boomMotor;
  RelativeEncoder boomEncoder;
  AnalogInput frontSensor;
  AnalogInput backSensor;
  
  public static enum state{
    FRONT,
    BACK,
    BOTH,
    NONE,
  }
  
  state _state  = state.NONE;
  
  /** Creates a new indexer. 
   * 
   * @param frontConveyerId CAN id for front motor
   * @param backConveyerId CAN id for back motor
   * @param boomMotorId CAN id for Boom Motor
   * @param frontSensorId Analog Id for front sensor
   * @param backSensorId Analog Id for back senor
  */
  public indexer(int frontConveyerId, int backConveyerId, int boomMotorId, int frontSensorId, int backSensorId){
    frontConveyer = new TalonSRX(frontConveyerId);
    backConveyer = new CANSparkMax(backConveyerId, MotorType.kBrushless);
    boomMotor = new CANSparkMax(boomMotorId, MotorType.kBrushless);
    boomEncoder = boomMotor.getEncoder();

    frontSensor = new AnalogInput(frontSensorId);
    backSensor = new AnalogInput(backSensorId);


  }
  /**
   * 
   * @return number of rotations of the motor, its relitave
   */
  public double boomEncoder(){
    return boomEncoder.getPosition();
    
  }

  /**
   * 
   */
  public void updateState(){
    if(frontSensor.getValue() > 2 && backSensor.getValue() > 2){
      _state = state.FRONT;

    } else if (frontSensor.getValue() < 2 && backSensor.getValue() > 2){
      _state = state.BACK;

    }else if (frontSensor.getValue() > 2 && backSensor.getValue() > 2){
      _state = state.BOTH;
    }else{
      _state = state.NONE;
    }

  }

  /** 
   * @param P percent speed from -1 to 1
   * @param FRWD whether or not to drive it forward.
   */
  public void DriveBackConveyer(double P,boolean FRWD){
    if(FRWD){backConveyer.set(P);
    }
    else{backConveyer.set(-P);}
  }
  /**
   * 
   * @param P percent output from -1 to 1
   */
  public void DriveFrontConveyer(double P, boolean FRWD){
    if(FRWD){
    frontConveyer.set(TalonSRXControlMode.PercentOutput, P);}
    else{frontConveyer.set(TalonSRXControlMode.PercentOutput, -P);}
  }

  public void DriveBoom(double P, Boolean FRWD){
    if(FRWD){
      boomMotor.set(P);
    }
    else{
      boomMotor.set(-P);
    }
  }



  /**
   * 
   * @return Front sensor voltage
   */
  public double GetfrontSensor(){
    return frontSensor.getVoltage();
  }
  /**
   * @return back sensor voltage
   */
  public double getBackSensor(){
    return backSensor.getVoltage();
  }


  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
