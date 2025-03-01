// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase {
  /** Creates a new Manipulator. */
  private final TalonFX manipulatorMotor;
  private DigitalInput coralSensor;
  private DigitalInput algaeSensor;

  public Manipulator() {
    manipulatorMotor = new TalonFX(Constants.Manipulator.MAN_CANID);
    coralSensor = new DigitalInput(Constants.Manipulator.CORAL_SENSOR_PORT);
    algaeSensor = new DigitalInput(Constants.Manipulator.ALGAE_SENSOR_PORT);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Coral Present:", this.getCoralSensor());
  }

  public boolean getAlgaeSensor(){
    return algaeSensor.get();
  }

  public boolean getCoralSensor(){
    return coralSensor.get();
  }

  public void intakeAlgae(){
    if(!getAlgaeSensor()){
      manipulatorMotor.set(-Constants.Manipulator.ALGAE_INTAKE_SPEED);
    } else {
      manipulatorMotor.set(0.0);
    }
  }

  public void intakeCoral(){
    if(!getAlgaeSensor()){
      manipulatorMotor.set(Constants.Manipulator.CORAL_INTAKE_SPEED);
    } else {
      manipulatorMotor.set(0.0);
    }
  }

  public void outtake(boolean reversed){
    if(reversed){
      manipulatorMotor.set(-Constants.Manipulator.OUTTAKE_SPEED);
    } else {
      manipulatorMotor.set(Constants.Manipulator.OUTTAKE_SPEED);
    }
  }

  public void stopAll(){
    manipulatorMotor.stopMotor();
  }
}
