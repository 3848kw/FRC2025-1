// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ElevatorSubsystem extends SubsystemBase
{

  // This gearbox represents a gearbox containing 1 Neo
  private final DCMotor m_elevatorGearbox = DCMotor.getNEO(1);


  private final SparkMax                  m_motor      = new SparkMax(17, MotorType.kBrushless);
  private final SparkClosedLoopController m_controller = m_motor.getClosedLoopController();
  private final RelativeEncoder           m_encoder    = m_motor.getEncoder();

  // Simulation classes help us simulate what's going on, including gravity.
  

  // Create a Mechanism2d visualization of the elevator
  private final Mechanism2d         m_mech2d         = new Mechanism2d(20, 12);


  /**
   * Subsystem constructor.
   */
  public ElevatorSubsystem()
  {
    SparkMaxConfig config = new SparkMaxConfig();
    config.encoder
        .positionConversionFactor(1) // Converts Rotations to Meters
        .velocityConversionFactor(1); // Converts RPM to MPS
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(.1, 0, .02)
        .maxMotion
        .maxVelocity(10)
        .maxAcceleration(9)
        .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal)
        .allowedClosedLoopError(0.01);
    m_motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

    // Publish Mechanism2d to SmartDashboard
    // To view the Elevator visualization, select Network Tables -> SmartDashboard -> Elevator Sim
    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  /**
   * Advance the simulation.
   */
 
  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain
   */
  public void reachGoal(double goal)
  {
    m_controller.setReference(goal,
                              ControlType.kPosition,
                              ClosedLoopSlot.kSlot0);
  }


  /**
   * Get the height in meters.
   *
   * @return Height in meters
   */
  public double getHeight()
  {
    return m_encoder.getPosition();
  }

  /**
   * A trigger for when the height is at an acceptable tolerance.
   *
   * @param height    Height in Meters
   * @param tolerance Tolerance in meters.
   * @return {@link Trigger}
   */
  public Trigger atHeight(double height, double tolerance)
  {
    return new Trigger(() -> MathUtil.isNear(height,
                                             getHeight(),
                                             tolerance));
  }

  /**
   * Set the goal of the elevator
   *
   * @param goal Goal in meters
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command setGoal(double goal)
  {
    return run(() -> reachGoal(goal));
  }

  /**
   * Stop the control loop and motor output.
   */


  /**
   * Update telemetry, including the mechanism visualization.
   */

   public void homeElevator() {
    // Move the elevator down slowly
    m_motor.set(.5);  // Set motor to a low speed in reverse (assuming -1 is full speed down)
    
    // Check if limit switch or sensor is triggered
    if (m_motor.getReverseLimitSwitch().isPressed()) {  // Assuming a limit switch is used
        m_motor.set(0);  // Stop the motor once we hit the home position
        m_encoder.setPosition(0);  // Reset encoder position to 0
    }
}
  // Run the intake method with obstruction checking

  @Override
  public void periodic()
  {
  }
}