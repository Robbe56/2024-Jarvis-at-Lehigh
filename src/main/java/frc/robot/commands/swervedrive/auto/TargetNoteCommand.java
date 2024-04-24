// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TargetNoteCommand extends Command {
  /** Creates a new TargetNoteCommand. */
  private final SwerveSubsystem swerveDrive;
  private final IntakeSubsystem intake;
  private final ShooterSubsystem shooter;
  private final ArmSubsystem arm;
  private final Timer timer;
  private final ChassisSpeeds autoDriveSpeeds;

  public TargetNoteCommand(SwerveSubsystem m_swerveDrive, IntakeSubsystem m_intake, ShooterSubsystem m_shooter, ArmSubsystem m_arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = m_swerveDrive;
    intake = m_intake;
    shooter = m_shooter;
    arm = m_arm;
    timer = new Timer();

    autoDriveSpeeds = new ChassisSpeeds(0, 0, 0);

    addRequirements(swerveDrive, intake, shooter, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    autoDriveSpeeds.vxMetersPerSecond = Constants.Drivebase.autoForwardSpeed;
    autoDriveSpeeds.vyMetersPerSecond = -Constants.Drivebase.NoteKP*swerveDrive.TrackNote(); //multiply Limelight value by P factor
    autoDriveSpeeds.omegaRadiansPerSecond = 0;
    
    swerveDrive.drive(autoDriveSpeeds);
    
    intake.intakeActive();
    shooter.FeedMotorFast();
    arm.ArmDownCommand();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    intake.intakeRest();
    shooter.StopFeedRoller();
    arm.StopArm();
    autoDriveSpeeds.vxMetersPerSecond = 0;
    autoDriveSpeeds.vyMetersPerSecond = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > Constants.Auton.TargetNoteTime; //stop auto driving after this much time

  }
}
