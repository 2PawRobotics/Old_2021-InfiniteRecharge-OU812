/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import com.kauailabs.navx.frc.AHRS;


/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private DifferentialDrive tank;

  private Joystick rightjoy;
  private Joystick leftjoy;
  private Joystick operator;
  private Timer timer = new Timer();

  private final Spark lift = new Spark(6);
  private final Spark balleater = new Spark(5);
  private final Spark shoot = new Spark(4);
  private final Spark spinnything = new Spark(7);
  private final Spark liftlock = new Spark(8);
  private AHRS gyro;
  private double DEAD_ZONE = 0.05; //dead zone for controlers
  private boolean tankDriveMode = false; 
  private double targetangle = 0;
  private double gyrobias = 0.05;
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private double currentgyroangle = 0;
  private double gyrocorrectionvalue = 0;
  private double ANGLE_DEAD_ZONE = 2;
  private double NGANGLE_DEAD_ZONE = -2;
  private double Yaccel;
  private double ydistancetraveled = 0;
  private double newydistancetraveled = 0;
  private double distancebias = 0.05;
  private double finalyaccel;
  private boolean moving;
  private double autonpath = 1;
  private boolean turnmode = false;
  private double stepcounter = 1;
  private double currenttime = 0;
  private double autondeadzone = 2;
  public Robot()
  {
    System.out.println("Robot.constructor()");
  }
  @Override
  public void robotInit() {
    System.out.println("Robot.robotInit()");

   final SpeedController left = new SpeedControllerGroup(new Spark(0), new Spark(2));
   final SpeedController right = new SpeedControllerGroup(new Spark(1), new Spark(3));

    tank = new DifferentialDrive(left, right);
    rightjoy = new Joystick(0);
    leftjoy = new Joystick(1);
    operator = new Joystick(2);
    CameraServer.getInstance().startAutomaticCapture();
    gyro = new AHRS(i2cPort);
    gyro.reset();
    gyro.calibrate();
  }

  public double getLeftJoy(){
    double raw = leftjoy.getY();
    return Math.abs(raw) < DEAD_ZONE ? 0.0 : raw; //if true returns 0.0 else return raw
  }
  public double getRightJoy(){
    double raw = rightjoy.getY();
    return Math.abs(raw) < DEAD_ZONE ? 0.0 : raw; //if true returns 0.0 else return raw
  }
  public double getRightJoyX(){
    double raw = rightjoy.getX();
    return Math.abs(raw) < DEAD_ZONE ? 0.0 : -raw; //if true returns 0.0 else return raw
  }
  private void gyrodrivecorrection(){
     
    gyrocorrectionvalue = targetangle - currentgyroangle;
  }
  private void distancemeasuring(){ //this method measures distance
    
    if (Math.abs(Yaccel) < distancebias){
      finalyaccel = 0.0; }
    else{
      finalyaccel = Yaccel;
    }
    newydistancetraveled = ydistancetraveled + finalyaccel;
    ydistancetraveled = newydistancetraveled;
  }


  public void autonpath1(){
    if(timer.get() > 0 && timer.get() <= 3.817 && stepcounter == 1)
    {
      targetangle = 0;
      turnmode = false;
      System.out.println("SC1 drive");
    }    
    else if(timer.get() > 3.817 && stepcounter == 1)
    {
      turnmode = true;
      targetangle = 90;
      stepcounter++;
      System.out.println("SC1 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 2)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }
    else if(timer.get() > 0 && timer.get() <= 1.445 && stepcounter == 3)
    {
      targetangle = 90;
      turnmode = false;
      System.out.println("SC3 Drive");
    }    
    else if(timer.get() > 1.445 && stepcounter == 3)
    {
      turnmode = true;
      targetangle = 180;
      stepcounter++;
      System.out.println("SC3 Turning");

    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 4)
    {
      turnmode = false;
      timer.reset();
      stepcounter++;
      System.out.println("Timer Reset");
    }
    else if(timer.get() > 0 && timer.get() <= 1.303 && stepcounter == 5) //Logan started here
    {
      targetangle = 180;
      turnmode = false;
      System.out.println("SC5 Drive");
    }    
    else if(timer.get() > 1.303 && stepcounter == 5)
    {
      turnmode = true;
      targetangle = 270;
      stepcounter++;
      System.out.println("SC5 Turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 6)
    {
      turnmode = false;
      timer.reset();
      stepcounter++;
      System.out.println("Timer Reset");
    }
    else if(timer.get() > 0 && timer.get() <= 1.182 && stepcounter == 7)
    {
      targetangle = 270;
      turnmode = false;
      System.out.println("SC7 Drive");
    }    
    else if(timer.get() > 1.182 && stepcounter == 7)
    {
      turnmode = true;
      targetangle = 348;
      stepcounter++;
      System.out.println("Sc7 Turn");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 8)
    {
      turnmode = false;
      timer.reset();
      stepcounter++;
      System.out.println("Timer Reset");
    }
    else if(timer.get() > 0 && timer.get() <= 3.41 && stepcounter == 9)
    {
      targetangle = 348;
      turnmode = false;
      System.out.println("SC9 Drive");
    }    
    else if(timer.get() > 3.41 && stepcounter == 9)
    {
      turnmode = true;
      targetangle = 270;
      stepcounter++;
      System.out.println("Sc9 Turn");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 10)
    {
      turnmode = false;
      timer.reset();
      stepcounter++;
      System.out.println("Reset Time");
    }
    else if(timer.get() > 0 && timer.get() <= 1.333 && stepcounter == 11)
    {
      targetangle = 270;
      turnmode = false;
      System.out.println("SC11 Drive");
    }    
    else if(timer.get() > 1.333 && stepcounter == 11)
    {
      turnmode = true;
      targetangle = 180;
      stepcounter++;
      System.out.println("SC11 Turn");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 12)
    {
      turnmode = false;
      timer.reset();
      stepcounter++;
      System.out.println("Reset Time");
    }
    else if(timer.get() > 0 && timer.get() <= 1.394 && stepcounter == 13)
    {
      targetangle = 180;
      turnmode = false;
      System.out.println("SC13 Drive");
    }    
    else if(timer.get() > 1.394 && stepcounter == 13)
    {
      turnmode = true;
      targetangle = 90;
      stepcounter++;
      System.out.println("SC13 Turn");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 14)
    {
      turnmode = false;
      timer.reset();
      stepcounter++;
      System.out.println("Reset Time");
    }
    else if(timer.get() > 0 && timer.get() <= 1.424 && stepcounter == 15)
    {
      targetangle = 90;
      turnmode = false;
      System.out.println("SC15 Drive");
    }    
    else if(timer.get() > 1.424 && stepcounter == 15)
    {
      turnmode = true;
      targetangle = 40;
      stepcounter++;
      System.out.println("SC15 Turn");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 16)
    {
      turnmode = false;
      timer.reset();
      stepcounter++;
      System.out.println("Reset Time");
    }
    else if(timer.get() > 0 && timer.get() <= 2.82 && stepcounter == 17)
    {
      targetangle = 40;
      turnmode = false;
      System.out.println("SC17 Drive");
    }    
    else if(timer.get() > 2.82 && stepcounter == 17)
    {
      turnmode = true;
      targetangle = -30;
      stepcounter++;
      System.out.println("SC17 Turn");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 18)
    {
      turnmode = false;
      timer.reset();
      stepcounter++;
      System.out.println("Reset Time");
    }
    else if(timer.get() > 0 && timer.get() <= 1.152 && stepcounter == 19)
    {
      targetangle = -30;
      turnmode = false;
      System.out.println("SC19 Drive");
    }    
    else if(timer.get() > 1.152 && stepcounter == 19)
    {
      turnmode = true;
      targetangle = -90;
      stepcounter++;
      System.out.println("SC19 Turn");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 20)
    {
      turnmode = false;
      timer.reset();
      stepcounter++;
      System.out.println("Reset Time");
    }
    else if(timer.get() > 0 && timer.get() <= 1.424 && stepcounter == 21)
    {
      targetangle = -90;
      turnmode = false;
      System.out.println("SC21 Drive");
    }    
    else if(timer.get() > 1.424 && stepcounter == 21)
    {
      turnmode = true;
      targetangle = -180;
      stepcounter++;
      System.out.println("SC21 Turn");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 22)
    {
      turnmode = false;
      timer.reset();
      stepcounter++;
      System.out.println("Reset Time");
    }
    else if(timer.get() > 0 && timer.get() <= 7.871 && stepcounter == 23)
    {
      targetangle = -180;
      turnmode = false;
      System.out.println("SC23 Drive");
    }    
    else if(timer.get() > 7.871 && stepcounter == 23)
    {
      targetangle = -180;
      turnmode = true;
      System.out.println("End of Auton Path 1");
    }
  }
  public void autonpath2(){
    if(timer.get() > 0 && timer.get() <= 1.576 && stepcounter == 1)
    {
      targetangle = 0;
      turnmode = false;
      System.out.println("SC1 drive");
    }    
    else if(timer.get() > 1.576 && stepcounter == 1)
    {
      turnmode = true;
      targetangle = -75;
      stepcounter++;
      System.out.println("SC1 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 2)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }
    else if(timer.get() > 0 && timer.get() <= 2.031 && stepcounter == 3)
    {
      targetangle = -75;
      turnmode = false;
      System.out.println("SC3 drive");
    }    
    else if(timer.get() > 2.031 && stepcounter == 3)
    {
      turnmode = true;
      targetangle = -5;
      stepcounter++;
      System.out.println("SC3 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 4)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }else if(timer.get() > 0 && timer.get() <= 4.051 && stepcounter == 5)
    {
      targetangle = -5;
      turnmode = false;
      System.out.println("SC5 drive");
    }    
    else if(timer.get() > 4.051 && stepcounter == 5)
    {
      turnmode = true;
      targetangle = 65;
      stepcounter++;
      System.out.println("SC5 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 6)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }else if(timer.get() > 0 && timer.get() <= 2 && stepcounter == 7)
    {
      targetangle = 65;
      turnmode = false;
      System.out.println("SC7 drive");
    }    
    else if(timer.get() > 2 && stepcounter == 7)
    {
      turnmode = true;
      targetangle = -23;
      stepcounter++;
      System.out.println("SC7 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 8)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }else if(timer.get() > 0 && timer.get() <= 1.576 && stepcounter == 9)
    {
      targetangle = -23;
      turnmode = false;
      System.out.println("SC9 drive");
    }    
    else if(timer.get() > 1.576 && stepcounter == 9)
    {
      turnmode = true;
      targetangle = -125;
      stepcounter++;
      System.out.println("SC9 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 10)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }
    else if(timer.get() > 0 && timer.get() <= 1.818 && stepcounter == 11)
    {
      targetangle = -125;
      turnmode = false;
      System.out.println("SC11 drive");
    }    
    else if(timer.get() > 1.818 && stepcounter == 11)
    {
      turnmode = true;
      targetangle = -190;
      stepcounter++;
      System.out.println("SC11 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 12)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }else if(timer.get() > 0 && timer.get() <= 2.436 && stepcounter == 13)
    {
      targetangle = -190;
      turnmode = false;
      System.out.println("SC13 drive");
    }    
    else if(timer.get() > 2.436 && stepcounter == 13)
    {
      turnmode = true;
      targetangle = -180;
      stepcounter++;
      System.out.println("SC13 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 14)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }else if(timer.get() > 0 && timer.get() <= 3.589 && stepcounter == 15)
    {
      targetangle = -180;
      turnmode = false;
      System.out.println("SC15 drive");
    }    
    else if(timer.get() > 3.589 && stepcounter == 15)
    {
      turnmode = true;
      targetangle = -100;
      stepcounter++;
      System.out.println("SC15 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 16)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }else if(timer.get() > 0 && timer.get() <= 1.878 && stepcounter == 17)
    {
      targetangle = -100;
      turnmode = false;
      System.out.println("SC17 drive");
    }    
    else if(timer.get() > 1.878 && stepcounter == 17)
    {
      turnmode = true;
      targetangle = -180;
      stepcounter++;
      System.out.println("SC17 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 18)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }else if(timer.get() > 0 && timer.get() <= 1.667 && stepcounter == 19)
    {
      targetangle = -180;
      turnmode = false;
      System.out.println("SC19 drive");
    }    
    else if(timer.get() > 1.667 && stepcounter == 19)
    {
      turnmode = true;
      targetangle = -180;
      System.out.println("End of Auton 2");
    }
  }
  public void autonpath3(){
    if(timer.get() > 0 && timer.get() <= 1.575 && stepcounter == 1)
    {
      targetangle = 0;
      turnmode = false;
      System.out.println("SC1 drive");
    }    
    else if(timer.get() > 1.575 && stepcounter == 1)
    {
      turnmode = true;
      targetangle = -87;
      stepcounter++;
      System.out.println("SC1 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 2)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }
    else if(timer.get() > 0 && timer.get() <= 1.182 && stepcounter == 3)
    {
      targetangle = -87;
      turnmode = false;
      System.out.println("SC3 drive");
    }    
    else if(timer.get() > 1.182 && stepcounter == 3)
    {
      turnmode = true;
      targetangle = 65;
      stepcounter++;
      System.out.println("SC3 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 4)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }
    else if(timer.get() > 0 && timer.get() <= 2.564 && stepcounter == 5)
    {
      targetangle = 65;
      turnmode = false;
      System.out.println("SC5 drive");
    }    
    else if(timer.get() > 2.564 && stepcounter == 5)
    {
      turnmode = true;
      targetangle = 0;
      stepcounter++;
      System.out.println("SC5 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 6)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }
    else if(timer.get() > 0 && timer.get() <= 1.272 && stepcounter == 7)
    {
      targetangle = 0;
      turnmode = false;
      System.out.println("SC7 drive");
    }    
    else if(timer.get() > 1.272 && stepcounter == 7)
    {
      turnmode = true;
      targetangle = -87;
      stepcounter++;
      System.out.println("SC7 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 8)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }
    else if(timer.get() > 0 && timer.get() <= 2.513 && stepcounter == 9)
    {
      targetangle = -87;
      turnmode = false;
      System.out.println("SC9 drive");
    }    
    else if(timer.get() > 2.513 && stepcounter == 9)
    {
      turnmode = true;
      targetangle = 93;
      stepcounter++;
      System.out.println("SC9 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 10)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }
    else if(timer.get() > 0 && timer.get() <= 2.513 && stepcounter == 11)
    {
      targetangle = 93;
      turnmode = false;
      System.out.println("SC11 drive");
    }    
    else if(timer.get() > 2.513 && stepcounter == 11)
    {
      turnmode = true;
      targetangle = 0;
      stepcounter++;
      System.out.println("SC11 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 12)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }
    else if(timer.get() > 0 && timer.get() <= 2.462 && stepcounter == 13)
    {
      targetangle = 0;
      turnmode = false;
      System.out.println("SC13 drive");
    }    
    else if(timer.get() > 2.462 && stepcounter == 13)
    {
      turnmode = true;
      targetangle = -90;
      stepcounter++;
      System.out.println("SC13 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 14)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }
    else if(timer.get() > 0 && timer.get() <= 2.513 && stepcounter == 15)
    {
      targetangle = -90;
      turnmode = false;
      System.out.println("SC15 drive");
    }    
    else if(timer.get() > 2.513 && stepcounter == 15)
    {
      turnmode = true;
      targetangle = 90;
      stepcounter++;
      System.out.println("SC15 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 16)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }
    if(timer.get() > 0 && timer.get() <= 1.378 && stepcounter == 17)
    {
      targetangle = 90;
      turnmode = false;
      System.out.println("SC17 drive");
    }    
    else if(timer.get() > 1.378 && stepcounter == 17)
    {
      turnmode = true;
      targetangle = 0;
      stepcounter++;
      System.out.println("SC17 turning");
    }
    else if(Math.abs(gyrocorrectionvalue) < autondeadzone && stepcounter == 18)
    {
      turnmode = false;
      timer.reset();
      System.out.println("Timer Reset");
      stepcounter++;
    }
    else if(timer.get() > 0 && timer.get() <= 1.939 && stepcounter == 19)
    {
      targetangle = 0;
      turnmode = false;
      System.out.println("SC19 drive");
    }    
    else if(timer.get() > 1.939 && stepcounter == 19)
    {
      turnmode = true;
      targetangle = 0;
      System.out.println("End of Auton 3");
    }
  }
  @Override
  public void teleopPeriodic() { //1 is 100% so .2 is 20%

    if(rightjoy.getRawButton(5)){tankDriveMode = true;}
    if(rightjoy.getRawButton(6)){tankDriveMode = false;}

    if(tankDriveMode){
      tank.tankDrive(getLeftJoy(), getRightJoy(), true);
    }
    else{
      tank.arcadeDrive(getRightJoy(), getRightJoyX(), true);
    }
    distancemeasuring();
    System.out.println("distance traveled" + ydistancetraveled);

    System.out.println("gyro angle is" + gyro.getAngle());
    System.out.println("Distance is " + gyro.getWorldLinearAccelY());
  if (operator.getRawButton(12)) {
    lift.set(-1);
    } else if (operator.getRawButton(11)) {
        lift.set(1);
    } else {
        lift.set(0);
    }

  if (operator.getRawButton(3)) {
  balleater.set(-0.7);
  } else if (operator.getRawButton(4)) {
  balleater.set(0.4);
  } else {
  balleater.set(0);
  }

  if (operator.getRawButton(2)) { //this is the statment for shooting forward
  shoot.set(-1);
} else if (operator.getRawButton(1)) { //this is a statment for reversing the shooter
  shoot.set(1);
} else {
  shoot.set(0);
}

  if (operator.getRawButton(7)) {
        spinnything.set(1);
      }
      else {
        spinnything.set(0);
  }
  
  if (operator.getRawButton(9)) {
  liftlock.set(-1);
} else if (operator.getRawButton(10)) {
  liftlock.set(1);
} else {
  liftlock.set(0);
}


  }


  @Override
  public void autonomousInit()
   {
    System.out.println("Robot.autonomous()");
    timer.reset();
    timer.start();
    turnmode = false;
    tank.setSafetyEnabled(false);
    gyro.reset();
    gyro.calibrate();
    ydistancetraveled = 0;
    targetangle = 0;
    stepcounter = 1;
    currenttime = 0;
   }
  public void ZeroTurn(){
    if (gyrocorrectionvalue > ANGLE_DEAD_ZONE){
       tank.tankDrive(-0.375, 0.375, false);
       
      }
      else if(gyrocorrectionvalue < NGANGLE_DEAD_ZONE){
       tank.tankDrive(0.375, -0.375, false);
       
      }
    }
    public void MovingTurn(){
      if (gyrocorrectionvalue > ANGLE_DEAD_ZONE){
         tank.tankDrive(-0.4, -0.3, false);
         
        }
        else if(gyrocorrectionvalue < NGANGLE_DEAD_ZONE){
         tank.tankDrive(-0.3, -0.4, false);
         
        }
      }
   @Override
   public void autonomousPeriodic() { //autonomous code
    currentgyroangle = gyro.getAngle();
    Yaccel = gyro.getRawAccelY();
    gyrodrivecorrection();
    distancemeasuring();

    if (autonpath == 1){
    autonpath1();
    }
    else if (autonpath == 2){
    autonpath2();
    }
    else if (autonpath == 3){
    autonpath3();
    }

   //System.out.println("gyro correction value is " + gyrocorrectionvalue);
   //System.out.println("distance traveled" + ydistancetraveled);
   //System.out.println("Yaccel = " + Yaccel);
   //System.out.println("Time is " + timer.get());
   //System.out.println("Target angle = " + targetangle);
   //System.out.println("" + currentgyroangle);
   //System.out.println(""  );

    currenttime = timer.get();

      if (Math.abs(gyrocorrectionvalue) > 10){
        ZeroTurn();
      }
       else if (Math.abs(gyrocorrectionvalue) > 2){
        MovingTurn();
      }
      else if(turnmode == true){
        tank.tankDrive(0, 0, false);
      }
      else 
      tank.tankDrive(-0.4, -0.4, false);
        
     

   }

  public void operatorControl ()
  {
    System.out.println("Robot.operatorControl()");

    tank.setSafetyEnabled(true);
  }

  public void disable()
  {
    System.out.println("Robot.disable()");
  }
  public void test()
  {
    System.out.println("Robot.test");
  }
}