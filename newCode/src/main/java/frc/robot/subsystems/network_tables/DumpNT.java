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
    private PIDTuner m_kickerPIDTuner;

    private double m_previousRollerKP;
    private double m_previousRollerKI;
    private double m_previousRollerKD;

    private double m_previousFloorKP;
    private double m_previousFloorKI;
    private double m_previousFloorKD;

    private DoubleTopic m_kickerTargetRPMTopic;
    private DoublePublisher m_kickerTargetRPMPublish;

    private DoubleTopic m_kickerActualRPMTopic;
    private DoublePublisher m_kickerActualRPMPublish;

    private DoubleTopic m_floorTargetVoltageTopic;
    private DoublePublisher m_floorTargetVoltagePublish;

    public DumpNT() {
        m_kickerPIDTuner = new PIDTuner("dump/tuning/floor", false);

        m_kickerPIDTuner.setP(KICKER_KP);
        m_kickerPIDTuner.setI(KICKER_KI);
        m_kickerPIDTuner.setD(KICKER_KD);

        m_kickerTargetRPMTopic = m_globalNT.getDoubleTopic("dump/kicker/targetRPM");
        m_kickerTargetRPMPublish = m_kickerTargetRPMTopic.publish();

        m_kickerActualRPMTopic = m_globalNT.getDoubleTopic("dump/kicker/actualRPM");
        m_kickerActualRPMPublish = m_kickerActualRPMTopic.publish();

        m_floorTargetVoltageTopic = m_globalNT.getDoubleTopic("dump/floor/targetVoltage");
        m_floorTargetVoltagePublish = m_floorTargetVoltageTopic.publish();

        m_previousFloorKP = KICKER_KP;
        m_previousFloorKI = KICKER_KI;
        m_previousFloorKD = KICKER_KD;
    }

    @Override
    public void periodic() {
        super.periodic();
        m_kickerActualRPMPublish.set(super.getKickerRPM());
        m_kickerTargetRPMPublish.set(super.getKickerTargetRPM());

        m_floorTargetVoltagePublish.set(super.getFloorTargetVoltage());

        if (m_kickerPIDTuner.isDifferentValues(m_previousFloorKP, m_previousFloorKI, m_previousFloorKD)) {
            m_previousFloorKP = m_kickerPIDTuner.getP();
            m_previousFloorKI = m_kickerPIDTuner.getI();
            m_previousFloorKD = m_kickerPIDTuner.getD();
            setKickerPID(m_kickerPIDTuner.getP(), m_kickerPIDTuner.getI(), m_kickerPIDTuner.getD());
        }
    }
}