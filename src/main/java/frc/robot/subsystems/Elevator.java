// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */

  private final SparkMax elevatorMotorOne;
  private final SparkMax elevatorMotorTwo;
  private final SparkMax testMax;
  private RelativeEncoder encoder;
  private DigitalInput elevatorHomeSwitch;
  private PIDController elevatorController;
  private ElevatorFeedforward feedforward;
  private double pidOutput = 0.0;


  public Elevator() {
    elevatorMotorOne = new SparkMax(Constants.Elevator.ELEVATOR_ONE_CANID, MotorType.kBrushless);
    elevatorMotorTwo = new SparkMax(Constants.Elevator.ELEVATOR_TWO_CANID, MotorType.kBrushless);
    testMax = new SparkMax(35, MotorType.kBrushless);

    elevatorHomeSwitch = new DigitalInput(Constants.Elevator.ELEVATOR_HOMESWITCH_PORT);
    encoder = elevatorMotorOne.getEncoder();
    elevatorController = new PIDController(Constants.Elevator.kP, Constants.Elevator.kI, Constants.Elevator.kD);
    feedforward = new ElevatorFeedforward(Constants.Elevator.kS, Constants.Elevator.kG, Constants.Elevator.kV);

    /* elevatorMotorOne.configure(
      Constants.Elevator.MOTOR_CONFIG.inverted(true), 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    );

    elevatorMotorTwo.configure(
      Constants.Elevator.MOTOR_CONFIG, 
      ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters
    ); */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Elevator Homed:", this.getHomeSwitch());
    SmartDashboard.putNumber("Elevator ENC:", this.getEncoderPosition());
    if(getHomeSwitch()){
      encoder.setPosition(0.0);
    }
  }

  public boolean getHomeSwitch(){
    return !elevatorHomeSwitch.get();
  }

  public double getEncoderPosition(){
    return encoder.getPosition();
  }

  public double getLiftPosition(){
    return 0.0;  //FIXME: Change this to elevator height relative to encoder
  }

  public void manualLift(double speed){
    elevatorMotorOne.set(speed);
    elevatorMotorTwo.set(-speed);
    testMax.set(speed);
    System.out.println("Speed: " + speed);
    System.out.println("MSTATE: " + elevatorMotorOne.get());
  }

  public void stopAll(){
    elevatorMotorOne.stopMotor();
    elevatorMotorTwo.stopMotor();
  }

  public void goToSetpoint(double setpoint){  //FIXME
    pidOutput = elevatorController.calculate(getLiftPosition(), setpoint);
    elevatorMotorOne.set(pidOutput);
  }

  public void homeElevator(){ //FIXME: Implement this later

  }
}
