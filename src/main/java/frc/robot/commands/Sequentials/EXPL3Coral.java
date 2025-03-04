// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequentials;

import frc.robot.commands.Helpers.setElevatorPOS;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class EXPL3Coral extends SequentialCommandGroup {
  /** Creates a new L1Coral. */
  public EXPL3Coral(Elevator s_elevator, Manipulator s_manipulator, Wrist s_wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunCommand(() -> s_wrist.setWristPos(90), s_wrist).withTimeout(1.5));
    addCommands(
      new setElevatorPOS(s_elevator, 0.9)
      .alongWith(new WaitCommand(1.5)
      .andThen(new RunCommand(() -> s_wrist.setWristPos(70), s_wrist).withTimeout(1.5)))
    );    
  }
}
