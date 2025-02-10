package frc.robot.subsystems;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A hatch mechanism actuated by a single {@link DoubleSolenoid}. */
public class climber extends SubsystemBase {
  //private final Compressor m_compressor = new Compressor(1,PneumaticsModuleType.CTREPCM);

  private final DoubleSolenoid m_hatchSolenoid =
      new DoubleSolenoid(
          PneumaticsModuleType.CTREPCM,
          0,
          1);

  /** Grabs the hatch. */
  public void climb() {
    m_hatchSolenoid.set(kForward);
  }

  /** Releases the hatch. */
  public void release() {
    m_hatchSolenoid.set(kReverse);
  }
  
 
}

