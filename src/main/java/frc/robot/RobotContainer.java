// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.climber;
import frc.robot.subsystems.corral;
import swervelib.SwerveInputStream;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Rotation;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * su bsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

 private SendableChooser<Command> autoChooser = new SendableChooser<>();
 private final frc.robot.subsystems.climber climber = new climber();
 private final corral corral = new frc.robot.subsystems.corral();
 private final LED led = new LED();

 private final ElevatorSubsystem elevator = new ElevatorSubsystem();

   // The robot's subsystems and commands are defined here...
   private final SwerveSubsystem drivebase = new SwerveSubsystem();
   // Replace with CommandPS4Controller or CommandJoystick if needed
   private final CommandXboxController m_driverController =
       new CommandXboxController(OperatorConstants.kDriverControllerPort);
       private final edu.wpi.first.wpilibj2.command.button.CommandJoystick CommandJoystick =
       new edu.wpi.first.wpilibj2.command.button.CommandJoystick(1);
   /** The container for the robot. Contains subsystems, OI devices, and commands. */
   public RobotContainer() {
  
     // Configure the trigger bindings
     DriverStation.silenceJoystickConnectionWarning(true);
     configureBindings();
     drivebase.setDefaultCommand(!RobotBase.isSimulation() ? driveFieldOrientedAngularVelocity : driveFieldOrientedDirectAngleSim);
  
     NamedCommands.registerCommand("erm", corral.spitCoralOut(.5));
     NamedCommands.registerCommand("out", corral.spitCoralOut(-.3));
     NamedCommands.registerCommand("hp", elevator.setGoal(49));
     NamedCommands.registerCommand("l3", elevator.setGoal(95));
     NamedCommands.registerCommand("l2", elevator.setGoal(13.6));
     NamedCommands.registerCommand("l1", elevator.setGoal(0));

   autoChooser = AutoBuilder.buildAutoChooser();


  // Another option that allows you to specify the default auto by its name
  // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");     

  SmartDashboard.putData("Auto Chooser", autoChooser);
}
// The real world (whats that?)
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> m_driverController.getLeftY() * -1,
                                                                () -> m_driverController.getLeftX() * -1)
                                                                .withControllerRotationAxis(m_driverController::getRightX)
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .allianceRelativeControl(true);          ;

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(m_driverController::getRightX,
                                                                                             m_driverController::getRightY)
                                                                                             .headingWhile(true);

  Command driveFieldOrientedDriectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

//Non reality code


  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> -m_driverController.getLeftY(),
                                                                   () -> -m_driverController.getLeftX())
                                                               .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
                                                               .deadband(OperatorConstants.DEADBAND)
                                                               .scaleTranslation(0.8)
                                                               .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleSim     = driveAngularVelocitySim.copy()
                                                                     .withControllerHeadingAxis(() -> Math.sin(
                                                                                                    m_driverController.getRawAxis(
                                                                                                        2) * Math.PI) * (Math.PI * 2),
                                                                                                () -> Math.cos(
                                                                                                    m_driverController.getRawAxis(
                                                                                                        2) * Math.PI) *
                                                                                                      (Math.PI * 2))
                                                                     .headingWhile(true);

  Command driveFieldOrientedDirectAngleSim = drivebase.driveFieldOriented(driveDirectAngleSim);


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  
  
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  // Drivebase
m_driverController.b().onTrue(Commands.runOnce(drivebase::zeroGyro));
m_driverController.b().onTrue(Commands.runOnce(drivebase::zeroGyro));
m_driverController.y().onTrue(Commands.runOnce(led::red));
m_driverController.x().onTrue(Commands.runOnce(led::blue));


// Climber Controls
m_driverController.rightBumper().onTrue(Commands.runOnce(climber::climb));
m_driverController.leftBumper().onTrue(Commands.runOnce(climber::release));

// Elevator Controls
CommandJoystick.button(2).whileTrue(elevator.setGoal(74)); 
CommandJoystick.button(3).whileTrue(elevator.setGoal(0));   
CommandJoystick.button(6).whileTrue(elevator.setGoal(28));     
CommandJoystick.button(7).whileTrue(elevator.setGoal(8));     

// Corral Controls
CommandJoystick.button(4).whileTrue(Commands.run(corral::l2)); // Start intake when button 9 is held

CommandJoystick.button(5).onFalse(Commands.runOnce(corral::stop)); // Stop corral when button 9 is released
CommandJoystick.button(5).whileTrue(Commands.run(corral::intake)); // Start intake when button 9 is held
CommandJoystick.button(1).whileTrue(Commands.runOnce(corral::outtake)); // Start outtake when button 8 is held

m_driverController.y().whileTrue(//Human player reload station
drivebase.driveToPose(
new Pose2d(new Translation2d
(Meter.of(1),
Meter.of(7)),
Rotation2d.fromDegrees(130)))
.andThen(
drivebase.driveToPose(//Coral side 1
new Pose2d(new Translation2d
(Meter.of(3.4),
Meter.of(5.1)),
Rotation2d.fromDegrees(-50)))
).andThen(
drivebase.driveToPose(//Human Player station
new Pose2d(new Translation2d
(Meter.of(1),
 Meter.of(7)),
Rotation2d.fromDegrees(130)))
).andThen(
drivebase.driveToPose(//Set point
new Pose2d(new Translation2d
(Meter.of(4.3),
Meter.of(6.3)),
Rotation2d.fromDegrees(-50)))
).andThen(
drivebase.driveToPose( //Coral 2
new Pose2d(new Translation2d
(Meter.of(5.2),
Meter.of(5.2)),
Rotation2d.fromDegrees(-120)))
)

);

m_driverController.button(9).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                      (Meter.of(3),
                                                                      Meter.of(4)),
                                                                Rotation2d.fromDegrees(-180))));  
                                                                                    
m_driverController.button(8).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                      (Meter.of(5),
                                                                      Meter.of(3)),
                                                              Rotation2d.fromDegrees(0))));
                                                              
m_driverController.button(7).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                              (Meter.of(3.5),
                                                              Meter.of(2.5)),
                                                      Rotation2d.fromDegrees(125))));

m_driverController.button(6).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                      (Meter.of(5),
                                                                      Meter.of(3)),
                                                              Rotation2d.fromDegrees(-50))));

m_driverController.button(5).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                              (Meter.of(6.1),
                                                              Meter.of(4)),
                                                      Rotation2d.fromDegrees(125))));

m_driverController.button(4).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                      (Meter.of(5.2),
                                                                      Meter.of(5.2)),
                                                              Rotation2d.fromDegrees(-120))));


m_driverController.button(3).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                      (Meter.of(3.3),
                                                                      Meter.of(5.3)),
                                                              Rotation2d.fromDegrees(-50))));
//Processor
m_driverController.button(2).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                      (Meter.of(11.5),
                                                                      Meter.of(7.5)),
                                                              Rotation2d.fromDegrees(90))));  
//Human Playerstation                                                                                
m_driverController.button(1).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                      (Meter.of(1),
                                                                      Meter.of(7)),
                                                                Rotation2d.fromDegrees(130)))); 



       // m_driverController.y().onTrue(Commands.run(led::red));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  } 

  /**
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

        m_driverController.button(17).whileTrue( //True lazy button
          drivebase.driveToPose(//Coral side 1
          new Pose2d(new Translation2d(Meter.of(3.4),Meter.of(5.1)), Rotation2d.fromDegrees(-50)))
        .andThen(
          drivebase.driveToPose(//Human Player station
          new Pose2d(new Translation2d(Meter.of(1),Meter.of(7)), Rotation2d.fromDegrees(130)))
        ).andThen(
          drivebase.driveToPose(//Set point
          new Pose2d(new Translation2d(Meter.of(4.3), Meter.of(6.3)), Rotation2d.fromDegrees(-50)))
        ).andThen(
          drivebase.driveToPose( //Coral side 2
          new Pose2d(new Translation2d(Meter.of(5.2), Meter.of(5.2)), Rotation2d.fromDegrees(-120)))
         ).andThen(
          drivebase.driveToPose(//Set point
          new Pose2d(new Translation2d(Meter.of(4.3), Meter.of(6.3)), Rotation2d.fromDegrees(130)))
        ).andThen(
          drivebase.driveToPose(//Human Player station
          new Pose2d(new Translation2d(Meter.of(1), Meter.of(7)), Rotation2d.fromDegrees(130)))
        ).andThen (
          drivebase.driveToPose(//Set point
          new Pose2d(new Translation2d(Meter.of(6.5), Meter.of(5.6)), Rotation2d.fromDegrees(130)))
        ).andThen(//Coral side 3
          drivebase.driveToPose(new Pose2d(new Translation2d(Meter.of(6.1), Meter.of(4)), Rotation2d.fromDegrees(180)))
        )

        );

        m_driverController.button(10).whileTrue(drivebase.sysIdDriveMotorCommand());
        m_driverController.button(9).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                                (Meter.of(3),
                                                                                Meter.of(4)),
                                                                          Rotation2d.fromDegrees(-180))));  
                                                                                              
        m_driverController.button(8).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                                (Meter.of(5),
                                                                                Meter.of(3)),
                                                                        Rotation2d.fromDegrees(0))));
                                                                        
         m_driverController.button(7).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                        (Meter.of(3.5),
                                                                        Meter.of(2.5)),
                                                                Rotation2d.fromDegrees(125))));

        m_driverController.button(6).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                                (Meter.of(5),
                                                                                Meter.of(3)),
                                                                        Rotation2d.fromDegrees(-50))));

         m_driverController.button(5).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                        (Meter.of(6.1),
                                                                        Meter.of(4)),
                                                                Rotation2d.fromDegrees(180))));

        m_driverController.button(4).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                                (Meter.of(5.2),
                                                                                Meter.of(5.2)),
                                                                        Rotation2d.fromDegrees(-120))));


        m_driverController.button(3).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                                (Meter.of(3.3),
                                                                                Meter.of(5.3)),
                                                                        Rotation2d.fromDegrees(-50))));
        //Processor
        m_driverController.button(2).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                                (Meter.of(11.5),
                                                                                Meter.of(7.5)),
                                                                        Rotation2d.fromDegrees(90))));  
        //Human Playerstation                                                                                
        m_driverController.button(1).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d
                                                                                (Meter.of(1),
                                                                                Meter.of(7)),
                                                                          Rotation2d.fromDegrees(130))));  
                                                                                
            // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
            // cancelling on release.

                  */
         
         
        
          
        
   /* 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
  */


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }


  }


 



