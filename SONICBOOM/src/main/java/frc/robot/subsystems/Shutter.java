package frc.robot.subsystems;
import frc.robot.RobotContainer;
import frc.robot.Constants.ConstController;
import frc.robot.Constants.ConstShutter;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shutter {
    int inProgresSpk;
    int inProgresAmp;
    double vMotorDisp;
    double vMotorAmp;
    double vMotorItk;
    double tMotorSpk;
    double tMotorAmp;

    public Shutter(double speedMotorDisparo, double speedMotorIntake, double timerEmbaloSpk, double speedMotorAmp, double timerEmbaloAmp){
        inProgresSpk=0;
        inProgresAmp=0;
        vMotorDisp=speedMotorDisparo;
        vMotorAmp=speedMotorAmp;
        vMotorItk=speedMotorIntake*(-1);
        tMotorSpk=timerEmbaloSpk;
        tMotorAmp=timerEmbaloAmp;


        ConstShutter.shutter_disp_sup.setIdleMode(IdleMode.kCoast);
        ConstShutter.shutter_disp_inf.setIdleMode(IdleMode.kCoast);
        ConstShutter.shutter_itk.setIdleMode(IdleMode.kCoast);
        ConstShutter.shutter_disp_sup.setInverted(false);
    }

    public class DispararSpk extends Thread {
        @Override
        public void run(){
            while(!interrupted()){
                SmartDashboard.putBoolean("THREAD DispararSpk", true);
                if(inProgresSpk==0){
                    ConstShutter.shutter_disp_inf.set(0.8);   //vMotorDisp
                    ConstShutter.shutter_disp_sup.set(0.8);
                    Timer.delay(tMotorSpk);
                    inProgresSpk=1;
                }
                if(inProgresSpk==1){
                    ConstShutter.shutter_itk.set(vMotorItk);
                    Timer.delay(1);
                    inProgresSpk=2;
                }
                if(inProgresSpk==2)
                {
                    ConstShutter.shutter_disp_inf.set(0);
                    ConstShutter.shutter_disp_sup.set(0);
                    ConstShutter.shutter_itk.set(0);
                    RobotContainer.engatilhado = false;
                    inProgresSpk=0;
                    ConstController.controllerCoDriver.setRumble(RumbleType.kRightRumble, 0);
                    SmartDashboard.putBoolean("THREAD DispararSpk", false);
                    interrupt();
                }
            }
            try{
                Thread.sleep(100);
            } catch(InterruptedException e){
                interrupt();
                return;
            }
        }
    }

    public class DispararSpkTarget extends Thread {
        @Override
        public void run(){
            while(!interrupted()){
                if(inProgresSpk==0){
                    ConstShutter.shutter_itk.set(vMotorItk);
                    Timer.delay(1.5);
                    inProgresSpk=1;
                }
                if(inProgresSpk==1)
                {
                    ConstShutter.shutter_disp_inf.set(0);
                    ConstShutter.shutter_disp_sup.set(0);
                    ConstShutter.shutter_itk.set(0);
                    RobotContainer.engatilhado = false;
                    inProgresSpk=0;
                    ConstController.controllerCoDriver.setRumble(RumbleType.kRightRumble, 0);
                    interrupt();
                }
            }
            try{
                Thread.sleep(100);
            } catch(InterruptedException e){
                interrupt();
                return;
            }
        }
    }
    public class DispararAmp extends Thread {
        @Override
        public void run(){
            while(!interrupted()){
                if(inProgresAmp==0){
                    ConstShutter.shutter_disp_inf.set(0.5);
                    ConstShutter.shutter_disp_sup.set(0.15);// Posição do shuter = 4;
                    // ConstShutter.shutter_disp_inf.set(0.5);
                    // ConstShutter.shutter_disp_sup.set(0.09);// Posição do shuter = 3;
                    Timer.delay(tMotorAmp);
                    inProgresAmp=1;
                }


                // if(inProgresAmp==0 && ConstShutter.shutter_inc_left.getEncoder().getPosition()>4){
                //     ConstShutter.shutter_disp_sup.set(0.05);
                //     ConstShutter.shutter_disp_inf.set(0.15);
                //     ConstShutter.shutter_itk.set(-0.4);
                //     Timer.delay(2);
                //     inProgresAmp=1;
                //     ConstController.controllerCoDriver.setRumble(RumbleType.kRightRumble, 0);
                    
                // }
                // if(inProgresAmp==1){
                //     ConstShutter.shutter_disp_sup.set(0);
                //     ConstShutter.shutter_disp_inf.set(0);
                //     ConstShutter.shutter_itk.set(0);
                //     inProgresAmp=0;
                //     interrupt();
                // }

                if(inProgresAmp==1){
                    ConstShutter.shutter_itk.set(vMotorItk);
                    Timer.delay(1);
                    inProgresAmp=2;
                }
                if(inProgresAmp==2)
                {
                    ConstShutter.shutter_disp_inf.set(0);
                    ConstShutter.shutter_disp_sup.set(0);
                    ConstShutter.shutter_itk.set(0);
                    RobotContainer.engatilhado = false;
                    inProgresAmp=0;
                    ConstController.controllerCoDriver.setRumble(RumbleType.kRightRumble, 0);
                    SmartDashboard.putBoolean("THREAD DispararAmp", false);
                    interrupt();
                }
            }
            try{
                Thread.sleep(100);
            } catch(InterruptedException e){
                interrupt();
                return;
            }
        }
    }

    public class SetOff extends Thread {
        @Override
        public void run(){
            while(!interrupted()){
                Timer.delay(0.4);
                ConstShutter.shutter_disp_inf.set(0);
                ConstShutter.shutter_disp_sup.set(0);
                interrupt();
            }
            try{
                Thread.sleep(100);
            } catch(InterruptedException e){
                interrupt();
                return;
            }
        }
    }

    public void disparoNoteSpk(){
        DispararSpk dispara = new DispararSpk();
        dispara.start();
    }

    public void disparoNoteAmp(){
        DispararAmp dispara = new DispararAmp();
        dispara.start();
    }

    public void dispararSpkTarget(){
        DispararSpkTarget dispara = new DispararSpkTarget();
        dispara.start();
    }

    public void setOff(){
        SetOff off = new SetOff();
        off.start();
    }
    
}
