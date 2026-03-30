// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;



import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants;


import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class DrivetrainSubsystem extends SubsystemBase {

  /*Instacias do Drivetrain - BEGIN*/
  CANSparkMax MSE = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax MSD = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax MIE = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax MID = new CANSparkMax(8, CANSparkMaxLowLevel.MotorType.kBrushless);

  RelativeEncoder ESE = MSE.getEncoder();
  RelativeEncoder ESD = MSD.getEncoder();
  RelativeEncoder EIE = MIE.getEncoder();
  RelativeEncoder EID = MID.getEncoder();

  MotorControllerGroup EsquerdaCTR_GROUP = new MotorControllerGroup(MSE, MIE);
  MotorControllerGroup DireitaCTR_GROUP  = new MotorControllerGroup(MSD, MID);

  DifferentialDrive differentialDrive = new DifferentialDrive(EsquerdaCTR_GROUP, DireitaCTR_GROUP);
  /*Instacias do Drivetrain - END*/
  /** Creates a new ExampleSubsystem. */
  public DrivetrainSubsystem()
  {
    MSE.restoreFactoryDefaults();
    MSD.restoreFactoryDefaults();
    MIE.restoreFactoryDefaults();
    MID.restoreFactoryDefaults();

    ESE.setPosition(0);
    ESD.setPosition(0);
    EIE.setPosition(0);
    EID.setPosition(0);

    MSE.follow(MSE);
    MSD.follow(MSD);
    MIE.follow(MIE);
    MID.follow(MID);

    DireitaCTR_GROUP.setInverted(true);
    EsquerdaCTR_GROUP.setInverted(false);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
