// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CardinalShuffleboard;
import frc.robot.Controller;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class DriveCommand extends CommandBase {
  private DriveTrain driveTrainSubsystem;
  private Limelight limelight;

  private double maxForward = Math.sqrt(0.7); 
  private double maxTurn = Math.sqrt(0.5);

  // percent per second
  private double acceleration = 1.0;
  private double deceleration = 3.0;

  private final double ANGULAR_ACCELERATION = 3.0;

  // current effective power for moving forward and turning
  private double forwardPower;
  private double turnPower;

  /** Creates a new Drive. */
  public DriveCommand(DriveTrain in_driveTrainSubsystem, Limelight Limelight) {
    this.limelight = Limelight;
    driveTrainSubsystem = in_driveTrainSubsystem;
    
    addRequirements(limelight);
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardPower = 0.0;
    turnPower = 0.0;
    limelight.getLightValue().setNumber(3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // will be squared later in differential drive
    // maxForward = Math.sqrt(CardinalShuffleboard.getMaxForwardPowerEntry());
    maxTurn = Math.sqrt(CardinalShuffleboard.getMaxTurnPowerEntry());
    maxForward = 1;

    double targetForwardPower = Controller.Drive.get_forward();
    double targetTurnPower = Controller.Drive.get_turn();
    boolean yButton = Controller.Drive.getYButton();

    // adjust forward power based on the target

    if (forwardPower > 0.4 || forwardPower < -0.4) {
      acceleration = 1.5;
    } else {
      acceleration = 10;
    }

    System.out.println(acceleration);

    // if close enough set them equal
    if (Math.abs(targetForwardPower - forwardPower) < Math.max(acceleration, deceleration) * Robot.period) {
      forwardPower = targetForwardPower;
    }
    // if accelerating
    /* 0 changed to 0.5 - this should allow the robot to run
       normally until it hits half power, in which the robot
       will then use acceleration to hit max speed
    */
    else if ((forwardPower > 0 && targetForwardPower > forwardPower) || (forwardPower < 0 && targetForwardPower < forwardPower))
    {
      forwardPower += Math.copySign(acceleration, forwardPower) * Robot.period;
    }
    // else decelerate
    else {
      forwardPower += Math.copySign(deceleration, -forwardPower) * Robot.period;
    }

    // adjust turn power based on the target

    // if close enough set them equal
    if (Math.abs(targetTurnPower - turnPower) < ANGULAR_ACCELERATION * Robot.period)
    {
      turnPower = targetTurnPower;
    }
    // else change turn power
    else {
      turnPower += Math.copySign(ANGULAR_ACCELERATION, targetTurnPower-turnPower) * Robot.period;
    }
    
  
    //command subsystem
    driveTrainSubsystem.set(forwardPower * maxForward, turnPower * maxTurn);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limelight.getLightValue().setNumber(0);
  }

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