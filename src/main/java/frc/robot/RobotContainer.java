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
import frc.robot.commands.IntakeCoral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Wrist;

public class RobotContainer {

  private Manipulator s_manipulator = new Manipulator();
  private Elevator s_elevator = new Elevator();
  private Wrist s_wrist = new Wrist();
  private XboxController controller = new XboxController(0);
  JoystickButton a = new JoystickButton(controller, XboxController.Button.kA.value);
  JoystickButton x = new JoystickButton(controller, XboxController.Button.kX.value);
  JoystickButton IntakeCoral = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
  JoystickButton ScoreCoral = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
  JoystickButton HELP = new JoystickButton(controller, XboxController.Button.kY.value);

  public RobotContainer() {
    configureBindings();

    s_elevator.setDefaultCommand(new RunCommand(() -> s_elevator.manualLift(controller.getRightY()), s_elevator));
  }

  private void configureBindings() {
    a.onTrue(new RunCommand(() -> s_wrist.setWristPos(16.0), s_wrist));
    a.onFalse(new RunCommand(() -> s_wrist.setWristPos(0.1), s_wrist));
    x.onTrue(new RunCommand(() -> s_wrist.setWristPos(90.0), s_wrist));
    x.onFalse(new RunCommand(() -> s_wrist.setWristPos(0.1), s_wrist));

    IntakeCoral.onTrue(new IntakeCoral(s_manipulator));
    ScoreCoral.onTrue(new RunCommand(() -> s_manipulator.intakeCoral(), s_manipulator));
    ScoreCoral.onFalse(new InstantCommand(() -> s_manipulator.stopAll(), s_manipulator));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
