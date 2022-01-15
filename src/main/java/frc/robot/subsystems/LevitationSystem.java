// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Grabber;

public class LevitationSystem extends SubsystemBase {
  private WPI_VictorSPX levitatior;
  private ArmFeedforward levitator_also;
  /** Creates a new LevitationSystem. */
  public LevitationSystem() {
    levitatior = new WPI_VictorSPX(Grabber.ELEVATOR);
    levitator_also = new ArmFeedforward(1, 1, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void set(double spd){
    levitatior.set(spd*0.5);
    System.out.println(spd);
  }
}
