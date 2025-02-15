package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class elevator {
private SparkMax elevatorMotor;
private SparkMaxConfig motorConfig;
@SuppressWarnings("unused")
private SparkClosedLoopController closedLoopController;
private RelativeEncoder encoder;

public elevator() {
    closedLoopController = elevatorMotor.getClosedLoopController();
    encoder = elevatorMotor.getEncoder();
    motorConfig = new SparkMaxConfig();

	elevatorMotor = new SparkMax(1, MotorType.kBrushless);
    motorConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);

/*
 * Configure the closed loop controller. We want to make sure we set the
 * feedback sensor as the primary encoder.
 */
motorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // Set PID values for position control. We don't need to pass a closed loop
    // slot, as it will default to slot 0.
    .p(0.1)
    .i(0)
    .d(0)
    .outputRange(-1, 1)
    // Set PID values for velocity control in slot 1
    .p(0.0001, ClosedLoopSlot.kSlot1)
    .i(0, ClosedLoopSlot.kSlot1)
    .d(0, ClosedLoopSlot.kSlot1)
    .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
    .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
    elevatorMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // Initialize dashboard values
    SmartDashboard.setDefaultNumber("Target Position", 0);
    SmartDashboard.setDefaultNumber("Target Velocity", 0);
    SmartDashboard.setDefaultBoolean("Control Mode", false);
    SmartDashboard.setDefaultBoolean("Reset Encoder", false);
  }

public void up(){
    encoder.setPosition(1*100);
}}

 
    