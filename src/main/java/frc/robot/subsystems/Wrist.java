// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final TalonFX wristMotor;
  private DutyCycleEncoder wristEncoder;
  private PIDController wristController;
  private double pidOutput = 0.0;

  public Wrist() {
    wristMotor = new TalonFX(Constants.Wrist.WRIST_CANID);
    wristEncoder = new DutyCycleEncoder(Constants.Wrist.WRIST_ENCODER_PORT);
    wristController = new PIDController(Constants.Wrist.kP, Constants.Wrist.kI, Constants.Wrist.kD);
    wristController.setTolerance(Constants.Wrist.errTolerance);

    wristMotor.getConfigurator().apply(Constants.Wrist.WRIST_CONFIG);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double getWristPos(){
    return wristEncoder.get();
  }

  public double getWristPosInDegrees(){
    return wristEncoder.get() * (360 / 1024);
  }

  public void manualWrist(double speed){
    wristMotor.set(speed);
  }

  public void setWristPos(double angle){
    pidOutput = wristController.calculate(getWristPosInDegrees(), angle);
    wristMotor.set(pidOutput);
  }

  public void stopAll(){
    wristMotor.stopMotor();
  }
}
