package frc.robot.subsystems.network_tables;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Dump;
import frc.robot.util.PIDTuner;
import static frc.robot.util.Constants.DumpConstants.*;

public class DumpNT extends Dump {
    private NetworkTableInstance m_globalNT = NetworkTableInstance.getDefault();

    private PIDTuner m_rollerPIDTuner;
    private PIDTuner m_floorPIDTuner;

    private double m_previousRollerKP;
    private double m_previousRollerKI;
    private double m_previousRollerKD;

    private double m_previousFloorKP;
    private double m_previousFloorKI;
    private double m_previousFloorKD;

    private DoubleTopic m_rollerTargetRPMTopic;
    private DoublePublisher m_rollerTargetRPMPublish;
    private DoubleSubscriber m_rollerTargetRPMSub;

    private DoubleTopic m_floorTargetRPMTopic;
    private DoublePublisher m_floorTargetRPMPublish;
    private DoubleSubscriber m_floorTargetRPMSub;

    public DumpNT() {
        m_rollerPIDTuner = new PIDTuner("dump/tuning/roller", false);
        m_floorPIDTuner = new PIDTuner("dump/tuning/floor", false);

        m_rollerPIDTuner.setP(ROLLER_KP);
        m_rollerPIDTuner.setI(ROLLER_KI);
        m_rollerPIDTuner.setD(ROLLER_KD);

        m_floorPIDTuner.setP(FLOOR_KP);
        m_floorPIDTuner.setI(FLOOR_KI);
        m_floorPIDTuner.setD(FLOOR_KD);

        m_rollerTargetRPMTopic = m_globalNT.getDoubleTopic("dump/roller/targetRPM");
        m_rollerTargetRPMPublish = m_rollerTargetRPMTopic.publish();
        m_rollerTargetRPMSub = m_rollerTargetRPMTopic.subscribe(0);

        m_previousRollerKP = ROLLER_KP;
        m_previousRollerKI = ROLLER_KI;
        m_previousRollerKD = ROLLER_KD;

        m_previousFloorKP = FLOOR_KP;
        m_previousFloorKI = FLOOR_KI;
        m_previousFloorKD = FLOOR_KD;
    }

    @Override
    public void periodic() {
        super.periodic();
        m_rollerTargetRPMPublish.set(super.getTargetRPM());

        if (m_rollerPIDTuner.isDifferentValues(m_previousRollerKP, m_previousRollerKI, m_previousRollerKD)) {
            m_previousRollerKP = m_rollerPIDTuner.getP();
            m_previousRollerKI = m_rollerPIDTuner.getI();
            m_previousRollerKD = m_rollerPIDTuner.getD();
            setRollerPID(m_rollerPIDTuner.getP(), m_rollerPIDTuner.getI(), m_rollerPIDTuner.getD());
        }

        if (m_floorPIDTuner.isDifferentValues(m_previousFloorKP, m_previousFloorKI, m_previousFloorKD)) {
            m_previousFloorKP = m_floorPIDTuner.getP();
            m_previousFloorKI = m_floorPIDTuner.getI();
            m_previousFloorKD = m_floorPIDTuner.getD();
            setFloorPID(m_floorPIDTuner.getP(), m_floorPIDTuner.getI(), m_floorPIDTuner.getD());
        }
    }
}
