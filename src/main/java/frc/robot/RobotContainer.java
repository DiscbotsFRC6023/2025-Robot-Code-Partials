// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Manipulator;

public class RobotContainer {

  private Manipulator s_manipulator = new Manipulator();
  private XboxController controller = new XboxController(0);
  JoystickButton Aintake = new JoystickButton(controller, XboxController.Button.kA.value);
  JoystickButton Cintake = new JoystickButton(controller, XboxController.Button.kX.value);
  JoystickButton outtake = new JoystickButton(controller, XboxController.Button.kB.value);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Aintake.onTrue(new RunCommand(() -> s_manipulator.intakeAlgae(), s_manipulator));
    Aintake.onFalse(new RunCommand(() -> s_manipulator.stopAll(), s_manipulator));
    Cintake.onTrue(new RunCommand(() -> s_manipulator.intakeCoral(), s_manipulator));
    Cintake.onFalse(new RunCommand(() -> s_manipulator.stopAll(), s_manipulator));
    outtake.onTrue(new RunCommand(() -> s_manipulator.outtake(false), s_manipulator));
    outtake.onFalse(new RunCommand(() -> s_manipulator.stopAll(), s_manipulator));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
