// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequentials;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L4Coral extends SequentialCommandGroup {
  /** Creates a new L1Coral. */
  public L4Coral(Elevator s_elevator, Manipulator s_manipulator, Wrist s_wrist) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunCommand(() -> s_wrist.setWristPos(90), s_wrist).withTimeout(1.5));
    addCommands(new RunCommand(() -> s_elevator.goToSetpoint(1.3), s_elevator).withTimeout(1.5));    
    addCommands(new RunCommand(() -> s_wrist.setWristPos(16), s_wrist).withTimeout(1.5));
  }
}
