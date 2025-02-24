package frc.robot.subsystems;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A hatch mechanism actuated by a single {@link DoubleSolenoid}. */
public class climber extends SubsystemBase {
  private final DoubleSolenoid m_hatchSolenoid =
  new DoubleSolenoid(2,
      PneumaticsModuleType.CTREPCM,
      0,
      1);
  @SuppressWarnings("unused")
  private final Compressor m_compressor = new Compressor(2,PneumaticsModuleType.CTREPCM);

 

  /** Grabs the hatch. */
  public void climb() {
    m_hatchSolenoid.set(kForward);
  }

  /** Releases the hatch. */
  public void release() {
    m_hatchSolenoid.set(kReverse);
  }
   public Command up() {
    // implicitly require `this`
    return this.runOnce(() -> m_hatchSolenoid.set(kForward));
  }
  public Command down() {
    // implicitly require `this`
    return this.runOnce(() -> m_hatchSolenoid.set(kReverse));
  }
 
}

