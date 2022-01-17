// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Controller;
import frc.robot.subsystems.LaunchWheels;

public class WheelsCommand extends CommandBase {
  private LaunchWheels launchWheels;

  /** Creates a new WheelsCommand. */
  public WheelsCommand(LaunchWheels launchWheels) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.launchWheels = launchWheels;
    addRequirements(launchWheels);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Controller.Drive.get_a_button()) {
      launchWheels.set(0.5);
    } else if (Controller.Drive.get_b_button()) {
      launchWheels.set(-0.5);
    } else {
      launchWheels.set(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
