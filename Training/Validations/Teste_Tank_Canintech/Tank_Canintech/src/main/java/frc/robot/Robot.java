// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Joystick;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   //Motor

  private VictorSPX MSE = new VictorSPX(4);
  private VictorSPX MIE = new VictorSPX(5);
  private VictorSPX MSD = new VictorSPX(3);
  private VictorSPX MID = new VictorSPX(2);



   //Joystick
  private Joystick Joy = new Joystick(1);






  @Override
  public void robotInit() {

    


    MID.follow(MSD);
    MIE.follow(MSE);

    MIE.setInverted(true);
    MSE.setInverted(true);


  }

private double startTime;
  
  @Override
  public void robotPeriodic() {}


  @Override
  public void autonomousInit() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp();

    if (time - startTime < 3) {
      MIE.set(VictorSPXControlMode.PercentOutput,0.6);
      MSD.set(VictorSPXControlMode.PercentOutput,0.6);
      MSE.set(VictorSPXControlMode.PercentOutput,0.6);
      MID.set(VictorSPXControlMode.PercentOutput,0.6);

    } else {
      if (time - startTime > 3 && time - startTime < 5) {
      MIE.set(VictorSPXControlMode.PercentOutput,0);
      MSD.set(VictorSPXControlMode.PercentOutput,0);
      MSE.set(VictorSPXControlMode.PercentOutput,0);
      MID.set(VictorSPXControlMode.PercentOutput,0); 

      } else {
        if (time - startTime < 8) {
        MIE.set(VictorSPXControlMode.PercentOutput,-0.6);
        MSD.set(VictorSPXControlMode.PercentOutput,-0.6);
        MSE.set(VictorSPXControlMode.PercentOutput,-0.6);
        MID.set(VictorSPXControlMode.PercentOutput,-0.6); 

        } else {
          MIE.set(VictorSPXControlMode.PercentOutput,0);
          MSD.set(VictorSPXControlMode.PercentOutput,0);
          MSE.set(VictorSPXControlMode.PercentOutput,0);
          MID.set(VictorSPXControlMode.PercentOutput,0); 
        }
      }
    }



}

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {

    double GetA = Joy.getRawAxis(2);      // Aceleration Button
    double GetD = Joy.getRawAxis(1);      // Drive Button
    double GetC = Joy.getRawAxis(3);      // Curve Button
    boolean GetB = Joy.getRawButton(1); // Brake Button
    
    /*MIE.set(VictorSPXControlMode.PercentOutput,(((GetD - GetC) * (GetA)) ));
    MSD.set(VictorSPXControlMode.PercentOutput,(((GetD + GetC) * (GetA)) ));
    MSE.set(VictorSPXControlMode.PercentOutput,(((GetD - GetC) * (GetA)) ));
    MID.set(VictorSPXControlMode.PercentOutput,(((GetD + GetC) * (GetA)) ));*/
  
  if (GetB == false) {
    MIE.set(VictorSPXControlMode.PercentOutput,(((GetD - (GetC / 2 )) * (GetA)) ));
    MSD.set(VictorSPXControlMode.PercentOutput,(((GetD + (GetC / 2 )) * (GetA)) ));
    MSE.set(VictorSPXControlMode.PercentOutput,(((GetD - (GetC / 2 )) * (GetA)) ));
    MID.set(VictorSPXControlMode.PercentOutput,(((GetD + (GetC / 2 )) * (GetA)) ));

  }
  else{
    MIE.set(VictorSPXControlMode.PercentOutput, 0.0300);
    MSD.set(VictorSPXControlMode.PercentOutput, 0.0300);
    MSE.set(VictorSPXControlMode.PercentOutput, 0.0300);
    MID.set(VictorSPXControlMode.PercentOutput, 0.0300);
  }



    }



  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}