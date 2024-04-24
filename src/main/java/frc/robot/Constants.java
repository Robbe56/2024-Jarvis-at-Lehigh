// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import swervelib.parser.PIDFConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag

  public static final class Auton
  {

    public static final PIDFConfig TranslationPID = new PIDFConfig(0.015, 0, 0);
    public static final PIDFConfig angleAutoPID   = new PIDFConfig(0.04, 0, 0.01);

    public static final double MAX_ACCELERATION = .1;
    public static final double TargetNoteTime = 1.9;
    public static final double AcrossFieldNoteTime = 1.5;
    public static final double midlineSpinUpTime = 0.8;
    public static final double TurnTarget = 80; //angle to turn to point at next note at midline
    public static final double LookAcrossFieldAngle = 10; //angle to turn to shooter back at our side of the field
    public static final double midlineTurnSpeed = 2;
  }

  public static final class Drivebase
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double autoForwardSpeed = 0.75; //creep forward speed when auto searching for note in m/s
    public static final double TeleopAutoForwardSpeed = 1.5;
    public static final double NoteKP = 0.035;
    public static final double AmpForwardSpeed = -1;
    public static final double AmpKP = 0.12;
    public static final double AmpAlignKP = 0.04;

    public static final double SpeakerTrackKP = -0.1;
    public static final double AprilTagTrackForwardSpeed = -0.5;
    public static final double AprilTagTrackSideSpeedKP = 0.05;

    public static final double SlowDownTurn = 0.7;//slow down turn by this much

    public static final double SlowDown = 0.75; //slow down robot by this much %
   

  }

  public static final class Intake{
    public static final int topMotorCANID = 22;
    public static final int bottomMotorCANID = 21;
    public static final double topMotorIntakeSpeed = 1;
    public static final double bottomMotorIntakeSpeed = 1;
    public static final int SpinButton = 1;
    public static final double SpitOutSpeed = -1;
    public static final double IntakeWatchdog = 6;//intake will run for this long then command will stop
    public static final double ShortIntakeTimer = 0.85; //intake command stops after this long in midline robbery
    
  }

  public static final class Hang{
    public static final int HangMotorPWMID = 1;
    public static final double armHangSpeed = 1;
    public static final double hangerUnwindSpeed = -0.5;
    public static final int leftHangMotorPWMID = 2;
    public static final int rightHangMotorPWMID = 3;
    public static final double hookHangSpeed = -1;
  }

  public static final class Shooter{
    public static final int armMotorCANID = 23;
    public static final int feedMotorCANID = 26;
    public static final int leftShooterMotorID = 25;
    public static final int rightShooterMotorID = 24;

    public static final int armDownLimitSwitch = 8;
    public static final int armUpLimitSwitch = 9;
    public static final int noteInFeederSensor = 0;
    public static final int LEDPWMPort = 0;
    public static final int analogNoteSensor = 1;
    public static final double noteSensorThreshold = 400;

    //Arm Encoder Values
    public static final int almostUpValue = 110;
    public static final int almostDownValue = 10;
    public static final double aimedAtSpeaker = 58; //From center, 55 works
    public static final int aimedFromSafe = 82; //90 to Shoot across field
    public static final int aimedFarShot = 52; //far shot from auto mode
    public static final int aimedAcrossField = 90; //arm angle to shoot across field

    public static final int ArmTopValue = 160; //encoder value when motor is all the way at the top

    public static final double armEquationSlope = -0.95; //empirically found
    public static final double armEquationIntercept = 77.0; //empirically found

    public static final int TrapEncoderSetting = 68;
     public static final double TrapSpeed = -0.75;
     public static final double TrapTime = 3.5;
     public static final double TrapAirTime = 1.0;
    
    public static final double ampFeedSpeed = 0.8;
    public static final double ampShooterSpeed = -0.6;
    public static final double waitTimeForScore = 1;

    public static final double armUpSpeedMax = -0.8;
    public static final double armUpAutoSpeed = -0.5;
    public static final double armHoldSpeed = -0.18;
    public static final double armDownSpeedMax = 0.4;
    public static final double DownReductionFactor = 0.15; //15% of speed when going down
    public static final double UpReductionFactor = 0.50; //50% of speed when going up
    
    public static final double feedLowSpeed = -0.5;
    public static final double feedHighSpeed = -1;
    public static final double shootLowSpeed = -0.65;
    public static final double shootHighSpeed = -.88; //speed when operator holds down right bumper
   
    public static final double suckInSpeed = -0.4;
    public static final double shootAutoDistance = -.5;

    public static final double TimeToRunShooterIntoAmp = 1; //run motors for this many seconds
    public static final double feedBackward = .6;
    public static final double BackUpShooterWheelTime = 0; //run motors backward to get note out of shooter wheels before we run them
    
    
    //auto mode values
    
    public static final double PreparedShotDelay = 1;
    public static final double AcrossFieldSpinUpTime = 1;
    public static final double MidlineNoteInAir = .75;
    public static final double NoteInAirTime = 1.6;
    public static final double NoteInAirAcrossFieldTime = .75;
    public static final double UnJamTime = 0.1 ;
    public static final double intakeTimer = 0;
    public static final double TrackNoteTime = .5;

    public static final double WaitForArm = 0.2;
    public static final double autoModeUnjamTime = 0.075;
    public static final double ShooterSpinUpTime = 1.5;
    public static final double AutoModeNoteInAir = 0.4;

    public static final double MoveOffSubWoofer = 0.1;
    
    
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.08;
    public static final double LEFT_Y_DEADBAND  = 0.08;
    public static final double RIGHT_X_DEADBAND = 0.08;
    public static final double RotationDeadband = 0.08;
    public static final double TURN_CONSTANT    = 6;
  }

  

}
