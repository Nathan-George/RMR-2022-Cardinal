// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.ResourceBundle.Control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CardinalShuffleboard;
import frc.robot.Controller;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends CommandBase {
  private DriveTrain driveTrainSubsystem;

  private double maxForward = Math.sqrt(0.7);
  private double maxTurn = Math.sqrt(0.5);

  // percent per seccond
  private final double ACCELERATION = 1.0;
  private final double DECCELERATION = 3.0;

  private final double ANGULAR_ACCELERATION = 3.0;

  // current effective power for moving forward and turning
  private double forwardMAGIC;
  private double turnMAGIC;

  /** Creates a new Drive. */
  public DriveCommand(DriveTrain in_driveTrainSubsystem) {
    driveTrainSubsystem = in_driveTrainSubsystem;
    
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardMAGIC = 0.0;
    turnMAGIC = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // will be squared later in differential drive
    
    maxForward = Math.sqrt(CardinalShuffleboard.getMaxForwardPowerEntry());
    maxTurn = Math.sqrt(CardinalShuffleboard.getMaxTurnPowerEntry());

    double targetForwardMAGIC = Controller.Drive.get_forward();
    double targetTurnMAGIC = Controller.Drive.get_turn();

    // adjust forward power based on the target

    // if close enough set them equal
    if (Math.abs(targetForwardMAGIC - forwardMAGIC) < Math.max(ACCELERATION, DECCELERATION) * Robot.period) {
      forwardMAGIC = targetForwardMAGIC;
    }
    // if accelerating
    else if ((forwardMAGIC > 0 && targetForwardMAGIC > forwardMAGIC) || (forwardMAGIC < 0 && targetForwardMAGIC < forwardMAGIC))
    {
      forwardMAGIC += Math.copySign(ACCELERATION, forwardMAGIC) * Robot.period;
    }
    // else decelerate
    else {
      forwardMAGIC += Math.copySign(DECCELERATION, -forwardMAGIC) * Robot.period;
    }

    // adjust turn power based on the target

    // if close enough set them equal
    if (Math.abs(targetTurnMAGIC - turnMAGIC) < ANGULAR_ACCELERATION * Robot.period)
    {
      turnMAGIC = targetTurnMAGIC;
    }
    // else change turn power
    else {
      turnMAGIC += Math.copySign(ANGULAR_ACCELERATION, targetTurnMAGIC-turnMAGIC) * Robot.period;
    }
    
    // command subsystem
    driveTrainSubsystem.set(forwardMAGIC * maxForward, Controller.Drive.get_turn() * 0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double getMaxForward() {
    return maxForward;
  }

  public double getMaxTurn() {
    return maxTurn;
  }
}
