// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Helpers.IntakeCoral;
import frc.robot.commands.Sequentials.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  private Manipulator s_manipulator = new Manipulator();
  private Elevator s_elevator = new Elevator();
  private Wrist s_wrist = new Wrist();
  private XboxController controller = new XboxController(0);
  private JoystickButton a = new JoystickButton(controller, XboxController.Button.kA.value);
  private JoystickButton x = new JoystickButton(controller, XboxController.Button.kX.value);
  private JoystickButton b = new JoystickButton(controller, XboxController.Button.kB.value);
  private JoystickButton y = new JoystickButton(controller, XboxController.Button.kY.value);
  private JoystickButton rightClick = new JoystickButton(controller, XboxController.Button.kRightStick.value);
  private JoystickButton intake = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
  private JoystickButton score = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
  private JoystickButton HELP = new JoystickButton(controller, XboxController.Button.kLeftStick.value);

  public RobotContainer() {
    configureBindings();

    s_elevator.setDefaultCommand(new RunCommand(() -> s_elevator.manualLift(controller.getRightY()), s_elevator));
  }

  private void configureBindings() {

    if(s_manipulator.getCoralSensor()){
      // CORAL
      a.onTrue(new L1Coral(s_elevator, s_manipulator, s_wrist));
      x.onTrue(new L2Coral(s_elevator, s_manipulator, s_wrist));
      y.onTrue(new L3Coral(s_elevator, s_manipulator, s_wrist));
      b.onTrue(new L4Coral(s_elevator, s_manipulator, s_wrist));
      intake.onTrue(new IntakeCoral(s_manipulator));
    } else {
      // ALGAE
      a.onTrue(new L1Algae(s_elevator, s_manipulator, s_wrist));
      x.onTrue(new L2Algae(s_elevator, s_manipulator, s_wrist));
      intake.onTrue(new RunCommand(() -> s_manipulator.intakeAlgae(), s_manipulator));
    }

    rightClick.onTrue(new Home(s_elevator, s_manipulator, s_wrist));
    score.onTrue(new RunCommand(() -> s_manipulator.intakeCoral(), s_manipulator));
    score.onFalse(new InstantCommand(() -> s_manipulator.stopAll(), s_manipulator));
    HELP.onTrue(new EXPL3Coral(s_elevator, s_manipulator, s_wrist));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous GOOFY");
  }
}
