package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ConstChassy;
import frc.robot.Constants.ConstShutter;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret.MoveTurret;

/*
 * 23.8 = posição maxima da torreta na horizontal
 * 
 */

public class Turret {
    private static final double MAX_ANGLE = 360.0;
    private static final int MAX_TURNS = 1; // Máximo de voltas em qualquer direção

    private int currentTurns; // Contador de voltas

    private double minPosiInc;
    private double maxPosiInc;
    private double minPosiTurret;
    private double maxPosiTurret;

    private int inProgres;
    private int inProgresTrt;
    private boolean inRange1;
    private boolean inRange2;
    private double movePosiShutter;
    private double movePosiTurret;
    private boolean shutterDown;
    public boolean setMode;

    private double setPStr;
    private double setPTrt;

    private boolean limiteHorario;
    private double velo;
    private boolean control;
    private boolean ctr;
    private double targetLime;
    private boolean targetmode;

    private double defasagem;

    public Turret(double minPositionInc, double maxPositionInc, double setPShutter, double setOutRangeMinShutter, double setOutRangeMaxShutter,
    double minPositionTurret, double maxPositionTurret, double setPTurret, double setOutRangeMinTurret, double setOutRangeMaxTurret)
    {
        targetLime=0;
        minPosiInc=minPositionInc;
        maxPosiInc=maxPositionInc;
        minPosiTurret=minPositionTurret;
        maxPosiTurret=maxPositionTurret;

        inProgres=0;
        inProgresTrt=0;
        inRange1=false;
        inRange2=false;
        shutterDown=false;
        setMode=false;

        setPStr=setPShutter;
        setPTrt=setPTurret;
        limiteHorario=false;
        ctr=false;
        targetmode=false;
        ConstShutter.shutter_inc_left.setInverted(true);

        ConstShutter.pidShutterLeft.setP(setPStr);
        ConstShutter.pidShutterLeft.setI(0);
        ConstShutter.pidShutterLeft.setD(0);
        ConstShutter.pidShutterLeft.setIZone(0);
        ConstShutter.pidShutterLeft.setFF(0);
        ConstShutter.pidShutterLeft.setOutputRange(setOutRangeMinShutter, setOutRangeMaxShutter);

        ConstShutter.shutter_inc_left.setOpenLoopRampRate(1);
        ConstShutter.shutter_inc_right.setOpenLoopRampRate(1);

        ConstShutter.shutter_inc_right.follow(ConstShutter.shutter_inc_left, true);

        ConstShutter.shutter_inc_right.setIdleMode(IdleMode.kBrake);
        ConstShutter.shutter_inc_left.setIdleMode(IdleMode.kBrake);

        ConstShutter.shutter_inc_right.getEncoder().setPosition(0);
        ConstShutter.shutter_inc_left.getEncoder().setPosition(0);

        ConstChassy.pidTurret.setP(setPTrt);
        ConstChassy.pidTurret.setI(0);
        ConstChassy.pidTurret.setD(0);
        ConstChassy.pidTurret.setIZone(0);
        ConstChassy.pidTurret.setFF(0);
        ConstChassy.pidTurret.setOutputRange(setOutRangeMinTurret, setOutRangeMaxTurret);  // -0.1 e 0.1
        ConstChassy.pidTurret.setPositionPIDWrappingEnabled(true);
        ConstChassy.pidTurret.setPositionPIDWrappingMaxInput(360);
        ConstChassy.pidTurret.setPositionPIDWrappingMinInput(0);

        ConstChassy.turret_Hor.getEncoder().setPositionConversionFactor(16.969775133509705862887610876268/5);// 360/21,214188
        ConstChassy.turret_Hor.setOpenLoopRampRate(1);

        ConstChassy.turret_Hor.getEncoder().setPosition(0);
        ConstChassy.turret_Hor.setIdleMode(IdleMode.kBrake);

        currentTurns = 0;

        ConstChassy.pigeon.setYaw(0);
    }

    public class MoveShotter extends Thread {
        @Override
        public void run(){
            while(!interrupted()){
                ConstShutter.pidShutterLeft.setOutputRange(-0.08, 0.25);

                SmartDashboard.putBoolean("THREAD MoveShutter", true);
                if(movePosiShutter>=minPosiInc && movePosiShutter<=maxPosiInc){
                    inRange1=true;
                }
                else{
                    inRange1=false;
                    SmartDashboard.putBoolean("THREAD MoveShutter", false);
                    interrupt();
                }

                if(inRange1==true){
                    if(inProgres==0 && movePosiShutter>=1){
                        ConstShutter.pidShutterLeft.setP(setPStr);
                        ConstShutter.pidShutterLeft.setReference(movePosiShutter, ControlType.kPosition);
                        SmartDashboard.putBoolean("THREAD MoveShutter", false);
                        interrupt();
                    }
                    if(movePosiShutter<1 && shutterDown==false){
                        ConstShutter.pidShutterLeft.setP(setPStr);
                        ConstShutter.pidShutterLeft.setReference(movePosiShutter, ControlType.kPosition);
                        shutterDown=true;
                    }
                     if(ConstShutter.shutter_inc_left.getEncoder().getPosition()<1 && shutterDown==true){
                        ConstShutter.pidShutterLeft.setP(0);
                        shutterDown=false;
                        SmartDashboard.putBoolean("THREAD MoveShutter", false);
                        interrupt();
                    }
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

    public class MoveTurret extends Thread {
        @Override
        public void run(){
            while(!interrupted()){
                SmartDashboard.putBoolean("THREAD MoveTurret", true);

                double yaw;

                yaw = (ConstChassy.pigeon.getYaw().getValue()+defasagem) % 360; // Primeiro, reduzimos o valor de yaw para estar dentro de um intervalo de -360 a 360
                if (yaw < 0) {
                    yaw += 360; // Se o valor for negativo, adicionamos 360 para que ele fique no intervalo de 0 a 360
                }

                if(((ConstChassy.turret_Hor.getEncoder().getPosition()>180) || (ConstChassy.turret_Hor.getEncoder().getPosition()<(-180))) && inProgresTrt==0){
                    ConstChassy.pidTurret.setOutputRange(-0.75, 0.75); //0.4
                    ConstChassy.pidTurret.setP(0.1);
                    ConstChassy.pidTurret.setPositionPIDWrappingEnabled(false);
                    inProgresTrt=1;
                }
                if(((ConstChassy.turret_Hor.getEncoder().getPosition()>=(-180)) && (ConstChassy.turret_Hor.getEncoder().getPosition()<=180)) && inProgresTrt==0){
                    ConstChassy.pidTurret.setOutputRange(-0.75, 0.75);
                    ConstChassy.pidTurret.setP(0.1);
                    ConstChassy.pidTurret.setPositionPIDWrappingEnabled(true);
                    inProgresTrt=1;
                }

                if(inProgresTrt==1)
                {
                    if(ConstChassy.turret_Hor.getEncoder().getPosition()<(-1)) yaw=yaw*(-1);

                    ConstChassy.pidTurret.setReference(yaw, ControlType.kPosition);
                    inProgresTrt=0;
                    SmartDashboard.putBoolean("THREAD MoveTurret", false);
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
    public class GetNoteTurret extends Thread {
        @Override
        public void run(){
            while(!interrupted()){
                if(((ConstChassy.turret_Hor.getEncoder().getPosition()>180) || (ConstChassy.turret_Hor.getEncoder().getPosition()<(-180))) && inProgresTrt==0){
                    ConstChassy.pidTurret.setPositionPIDWrappingEnabled(false);
                    inProgresTrt=1;
                }
                if(((ConstChassy.turret_Hor.getEncoder().getPosition()>=(-180)) && (ConstChassy.turret_Hor.getEncoder().getPosition()<=180)) && inProgresTrt==0){
                    ConstChassy.pidTurret.setPositionPIDWrappingEnabled(true);
                    inProgresTrt=1;
                }

                if(inProgresTrt==1)
                {
                    if(ConstChassy.turret_Hor.getEncoder().getPosition()<(-1)) movePosiTurret=movePosiTurret*(-1);

                    ConstChassy.pidTurret.setReference(movePosiTurret, ControlType.kPosition);
                    inProgresTrt=0;
                    SmartDashboard.putBoolean("THREAD MoveTurret", false);
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
    
    // public class TargetMoveShutter extends Thread {
    //     @Override
    //     public void run(){
    //         while(!interrupted()){
    //             SmartDashboard.putBoolean("THREAD TargetMoveShutter", true);

    //             double alvo = targetLime;
    //             double range= 10;

    //             if(setMode==false){
    //                 targetmode=true;
    //                 interrupt();
    //             }

    //             if((LimeLight.targeTY>=(alvo-1) && LimeLight.targeTY<=(alvo+1))){
    //                 targetmode=true;
    //                 ConstShutter.shutter_inc_left.set(0.003);
    //                 ConstShutter.shutter_inc_right.set(0.003);
    //                 ConstShutter.pidShutterRight.setP(0.42);
    //                 ConstShutter.pidShutterLeft.setP(0.42);
    //                 ConstShutter.pidShutterLeft.setReference(ConstShutter.shutter_inc_left.getEncoder().getPosition(), ControlType.kPosition);
    //                 // ConstShutter.pidShutterLeft.setOutputRange(-0.05, 0.2);
    //                 ConstShutter.pidShutterRight.setReference(ConstShutter.shutter_inc_right.getEncoder().getPosition(), ControlType.kPosition);
    //                 // ConstShutter.pidShutterRight.setOutputRange(-0.05, 0.2);
    //                 SmartDashboard.putBoolean("THREAD TargetMoveShutter", false);
    //                 interrupt();
    //             }

    //             if((LimeLight.targeTY>=(alvo-1) && LimeLight.targeTY<=(alvo+1))){
    //                 targetmode=true;
    //                 ConstShutter.shutter_inc_left.set(0.003);
    //                 ConstShutter.shutter_inc_right.set(0.003);
    //                 ConstShutter.pidShutterRight.setP(0.42);
    //                 ConstShutter.pidShutterLeft.setP(0.42);
    //                 ConstShutter.pidShutterLeft.setReference(ConstShutter.shutter_inc_left.getEncoder().getPosition(), ControlType.kPosition);
    //                 ConstShutter.pidShutterRight.setReference(ConstShutter.shutter_inc_right.getEncoder().getPosition(), ControlType.kPosition);
    //                 SmartDashboard.putBoolean("THREAD TargetMoveShutter", false);
    //                 interrupt();
    //             }

    //             if(LimeLight.targeTArea==0 && LimeLight.targeTY==0){
    //                 targetmode=true;
    //                 ConstShutter.shutter_inc_left.set(0.003);
    //                 ConstShutter.shutter_inc_right.set(0.003);
    //                 ConstShutter.pidShutterLeft.setP(0.42);
    //                 ConstShutter.pidShutterRight.setP(0.42);
    //                 ConstShutter.pidShutterLeft.setReference(ConstShutter.shutter_inc_left.getEncoder().getPosition(), ControlType.kPosition);
                    
    //                 ConstShutter.pidShutterRight.setReference(ConstShutter.shutter_inc_right.getEncoder().getPosition(), ControlType.kPosition);
                    
    //                 SmartDashboard.putBoolean("THREAD TargetMoveShutter", false);
                    
    //                 interrupt();
    //             }
    //             if(setMode && LimeLight.targeTArea!=0 && targetmode==false){
    //                 if(ConstShutter.shutter_inc_left.getEncoder().getPosition()<=3.5){
    //                     if(LimeLight.targeTY>(alvo+range)){
    //                         ConstShutter.pidShutterLeft.setP(0);
    //                         ConstShutter.pidShutterRight.setP(0);
    //                         ConstShutter.shutter_inc_left.set(0.1);
    //                         ConstShutter.shutter_inc_right.set(0.1);
    //                     }
    //                     if(LimeLight.targeTY>(alvo) && LimeLight.targeTY<=(alvo+range)){
    //                         ConstShutter.pidShutterLeft.setP(0);
    //                         ConstShutter.pidShutterRight.setP(0);
    //                         ConstShutter.shutter_inc_left.set(0.07);
    //                         ConstShutter.shutter_inc_right.set(0.07);
    //                     }
    //                     if(LimeLight.targeTY<(alvo-range)){
    //                         ConstShutter.pidShutterLeft.setP(0);
    //                         ConstShutter.pidShutterRight.setP(0);
    //                         ConstShutter.shutter_inc_left.set(-0.005);  // -0.2
    //                         ConstShutter.shutter_inc_right.set(-0.005);
    //                     }
    //                     if(LimeLight.targeTY>=(alvo-range) && LimeLight.targeTY<(alvo)){
    //                         ConstShutter.pidShutterLeft.setP(0);
    //                         ConstShutter.pidShutterRight.setP(0);
    //                         ConstShutter.shutter_inc_left.set(0.005);  // -0.2
    //                         ConstShutter.shutter_inc_right.set(0.005);
    //                     }
    //                 }
    //                 if(ConstShutter.shutter_inc_left.getEncoder().getPosition()>3.5){
    //                     if(LimeLight.targeTY>(alvo+range)){
    //                         ConstShutter.pidShutterLeft.setP(0);
    //                         ConstShutter.pidShutterRight.setP(0);
    //                         ConstShutter.shutter_inc_left.set(0.08);
    //                         ConstShutter.shutter_inc_right.set(0.08);
    //                     }
    //                     if(LimeLight.targeTY<(alvo-range)){
    //                         ConstShutter.pidShutterLeft.setP(0);
    //                         ConstShutter.pidShutterRight.setP(0);
    //                         ConstShutter.shutter_inc_left.set(-0.05);
    //                         ConstShutter.shutter_inc_right.set(-0.05);
    //                     }
    //                 }
    //             }
    //             else{
    //                 ConstShutter.shutter_inc_left.set(0.003);
    //                 ConstShutter.shutter_inc_right.set(0.003);
    //                 ConstShutter.pidShutterLeft.setP(0.42);
    //                 ConstShutter.pidShutterRight.setP(0.42);
    //                 ConstShutter.pidShutterLeft.setReference(ConstShutter.shutter_inc_left.getEncoder().getPosition(), ControlType.kPosition);
                    
    //                 ConstShutter.pidShutterRight.setReference(ConstShutter.shutter_inc_right.getEncoder().getPosition(), ControlType.kPosition);
                    
    //                 SmartDashboard.putBoolean("THREAD TargetMoveShutter", false);
                    
    //                 interrupt();
    //             }
    //         }
    //         try{
    //             Thread.sleep(100);
    //         } catch(InterruptedException e){
    //             ConstShutter.shutter_inc_left.set(0);
    //             ConstShutter.shutter_inc_right.set(0);
    //             ConstShutter.pidShutterLeft.setReference(ConstShutter.shutter_inc_left.getEncoder().getPosition(), ControlType.kPosition);
    //             ConstShutter.pidShutterLeft.setOutputRange(-0.05, 0.2);
    //             ConstShutter.pidShutterLeft.setP(0.42);
    //             ConstShutter.pidShutterRight.setReference(ConstShutter.shutter_inc_right.getEncoder().getPosition(), ControlType.kPosition);
    //             ConstShutter.pidShutterRight.setOutputRange(-0.05, 0.2);
    //             ConstShutter.pidShutterRight.setP(0.42);
    //             interrupt();
    //             return;
    //         }
    //     }
    // }

    // public class TargetMoveTurret extends Thread {
    //     @Override
    //     public void run(){
    //         while(!interrupted()){

    //             ConstShutter.shutter_disp_inf.set(0.8);
    //             ConstShutter.shutter_disp_sup.set(0.8);

    //             if(setMode==false){
    //                 interrupt();
    //             }

    //             if((LimeLight.targeTX>=(-1) && LimeLight.targeTX<=(1))){
    //                 ConstChassy.turret_Hor.set(0);
    //                 ConstChassy.pidTurret.setReference(ConstChassy.turret_Hor.getEncoder().getPosition(), ControlType.kPosition);
    //                 ConstChassy.pidTurret.setOutputRange(-0.25, 0.25);
    //                 ConstChassy.pidTurret.setP(0.1);
    //                 SmartDashboard.putBoolean("THREAD TargetMoveTurret", false);
    //                 interrupt();
    //             }

    //             if(LimeLight.targeTArea==0 && LimeLight.targeTX==0){
    //                 ConstChassy.turret_Hor.set(0);
    //                 ConstChassy.pidTurret.setReference(ConstChassy.turret_Hor.getEncoder().getPosition(), ControlType.kPosition);
    //                 ConstChassy.pidTurret.setOutputRange(-0.25, 0.25);
    //                 ConstChassy.pidTurret.setP(0.1);
    //                 SmartDashboard.putBoolean("THREAD TargetMoveTurret", false);
    //                 interrupt();
    //             }

    //             if(setMode && LimeLight.targeTArea!=0){
    //                 ConstChassy.pidTurret.setP(0);

    //                 if(LimeLight.targeTX>=(15)){
    //                     ConstChassy.turret_Hor.set(0.1);
    //                 }
    //                 if(LimeLight.targeTX>(3) && LimeLight.targeTX<(15)){
    //                     ConstChassy.turret_Hor.set(0.06);
    //                 }
    //                 if(LimeLight.targeTX>(1) &&LimeLight.targeTX<=(3)){
    //                     ConstChassy.turret_Hor.set(0.3);
    //                 }

    //                 if(LimeLight.targeTX<=(-15)){
    //                     ConstChassy.turret_Hor.set(-0.1);
    //                 }
    //                 if(LimeLight.targeTX>(-15) && LimeLight.targeTX<(-3)){
    //                     ConstChassy.turret_Hor.set(-0.06);
    //                 }
    //                 if(LimeLight.targeTX>=(-3) &&  LimeLight.targeTX<(-1)){
    //                     ConstChassy.turret_Hor.set(-0.3);
    //                 }

    //                 if(LimeLight.targeTX>=(-1) &&  LimeLight.targeTX<=(1)){
    //                     ConstChassy.turret_Hor.set(0);
    //                     ConstChassy.pidTurret.setReference(ConstChassy.turret_Hor.getEncoder().getPosition(), ControlType.kPosition);
    //                     ConstChassy.pidTurret.setOutputRange(-0.25, 0.25);
    //                     ConstChassy.pidTurret.setP(0.05);
    //                     SmartDashboard.putBoolean("THREAD TargetMoveTurret", false);
    //                     interrupt(); 
    //                 }
    //             }
    //         }
    //         try{
    //             Thread.sleep(100);
    //         } catch(InterruptedException e){
    //             interrupt();
    //             return;
    //         }
    //     }
    // }
    
    public class MoveShotterDisp extends Thread {
        @Override
        public void run(){
            while(!interrupted()){
                ConstShutter.pidShutterLeft.setOutputRange(-0.08, 0.25);

                if(movePosiShutter==0){
                    ConstShutter.shutter_disp_inf.set(0);
                    ConstShutter.shutter_disp_sup.set(0);
                }
                else{
                    ConstShutter.shutter_disp_inf.set(0.8);
                    ConstShutter.shutter_disp_sup.set(0.8);
                }


                SmartDashboard.putBoolean("THREAD MoveShutter", true);
                if(movePosiShutter>=minPosiInc && movePosiShutter<=maxPosiInc){
                    inRange1=true;
                }
                else{
                    inRange1=false;
                    SmartDashboard.putBoolean("THREAD MoveShutter", false);
                    interrupt();
                }

                if(inRange1==true){
                    if(inProgres==0 && movePosiShutter>=1){
                        ConstShutter.pidShutterLeft.setP(setPStr);
                        ConstShutter.pidShutterLeft.setReference(movePosiShutter, ControlType.kPosition);
                        SmartDashboard.putBoolean("THREAD MoveShutter", false);
                        interrupt();
                    }
                    if(movePosiShutter<1 && shutterDown==false){
                        ConstShutter.pidShutterLeft.setP(setPStr);
                        ConstShutter.pidShutterLeft.setReference(movePosiShutter, ControlType.kPosition);
                        shutterDown=true;
                    }
                     if(ConstShutter.shutter_inc_left.getEncoder().getPosition()<1 && shutterDown==true){
                        ConstShutter.pidShutterLeft.setP(0);
                        shutterDown=false;
                        SmartDashboard.putBoolean("THREAD MoveShutter", false);
                        interrupt();
                    }
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


    public void moveShotter(double movePosition)
    {
        movePosiShutter=movePosition;
        setMode=false;
        MoveShotter moveSht = new MoveShotter();
        moveSht.start();
    }
    // public void targetMoveShotter(double target){
    //     setMode=true;
    //     targetLime=target;
    //     targetmode=false;
    //     TargetMoveShutter modesht = new TargetMoveShutter();
    //     modesht.start();
    // }

    public void moveTurret(double defasa)
    {
        defasagem=defasa;
        MoveTurret moveTr = new MoveTurret();
        moveTr.start();
    }

    // public void targetMoveTurret(){
    //     setMode=true;
    //     TargetMoveTurret modetrt = new TargetMoveTurret();
    //     modetrt.start();
    // }
    
    public void getNoteTurret(double movePosition){
        movePosiTurret=movePosition;
        setMode=false;
        GetNoteTurret moveTrt = new GetNoteTurret();
        moveTrt.start();
    }

    public void moveShotterDisp(double movePosition){
        movePosiShutter=movePosition;
        setMode=false;
        MoveShotterDisp moveSht = new MoveShotterDisp();
        moveSht.start();
    }
    
}
