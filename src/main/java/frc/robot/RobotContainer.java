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
  private XboxController driver = new XboxController(0);
  private XboxController controller = new XboxController(1);
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
    x.onTrue(new L2(s_elevator, s_manipulator, s_wrist));
    y.onTrue(new L3(s_elevator, s_manipulator, s_wrist));
    b.onTrue(new L4(s_elevator, s_manipulator, s_wrist));
    a.onTrue(new RunCommand(() -> s_manipulator.intakeAlgae(), s_manipulator));
    a.onFalse(new RunCommand(() -> s_manipulator.stopAll(), s_manipulator));
    HELP.onTrue(new Barge(s_elevator, s_manipulator, s_wrist));

    intake.onTrue(new IntakeCoral(s_manipulator).andThen(new RunCommand(() -> s_manipulator.intakeCoral(0.1), s_manipulator).withTimeout(0.2).andThen(new InstantCommand(() -> s_manipulator.stopAll()))));
    rightClick.onTrue(new Home(s_elevator, s_manipulator, s_wrist));
    score.onTrue(new RunCommand(() -> s_manipulator.intakeCoral(), s_manipulator));
    score.onFalse(new InstantCommand(() -> s_manipulator.stopAll(), s_manipulator));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous GOOFY");
  }
}
