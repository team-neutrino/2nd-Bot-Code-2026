package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.DoubleSubscriber;

public class PIDTuner {
    private NetworkTableInstance m_globalNetworkTable = NetworkTableInstance.getDefault();

    private DoubleTopic m_P;
    private DoubleTopic m_I;
    private DoubleTopic m_D;
    private IntegerTopic m_slot;

    private DoublePublisher m_P_Publisher;
    private DoublePublisher m_I_Publisher;
    private DoublePublisher m_D_Publisher;
    private IntegerPublisher m_slot_Publisher;

    private DoubleSubscriber m_P_Subscriber;
    private DoubleSubscriber m_I_Subscriber;
    private DoubleSubscriber m_D_Subscriber;
    private IntegerSubscriber m_slot_Subscriber;

    public PIDTuner(String subsystemName, boolean usesSlots) {
        if (usesSlots) {
            m_slot = m_globalNetworkTable.getIntegerTopic("/" + subsystemName + "/slot");
            m_slot_Subscriber = m_slot.subscribe(0);
            m_slot_Publisher = m_slot.publish();
            m_slot_Publisher.setDefault(0);
        }

        m_P = m_globalNetworkTable.getDoubleTopic("/" + subsystemName + "/P");
        m_I = m_globalNetworkTable.getDoubleTopic("/" + subsystemName + "/I");
        m_D = m_globalNetworkTable.getDoubleTopic("/" + subsystemName + "/D");

        m_P_Subscriber = m_P.subscribe(0.0);
        m_I_Subscriber = m_I.subscribe(0.0);
        m_D_Subscriber = m_D.subscribe(0.0);
        m_P_Publisher = m_P.publish();
        m_I_Publisher = m_I.publish();
        m_D_Publisher = m_D.publish();

        m_P_Publisher.setDefault(0.1);
        m_I_Publisher.setDefault(0.0);
        m_D_Publisher.setDefault(0.0);
    }

    public long getSlot() {
        return m_slot_Subscriber.get();
    }

    public double getP() {
        return m_P_Subscriber.get();
    }

    public double getI() {
        return m_I_Subscriber.get();
    }

    public double getD() {
        return m_D_Subscriber.get();
    }

    public void setSlot(long newSlotValue) {
        m_slot_Publisher.set(newSlotValue);
    }

    public void setP(double newPValue) {
        m_P_Publisher.set(newPValue);
    }

    public void setI(double newIValue) {
        m_I_Publisher.set(newIValue);
    }

    public void setD(double newDValue) {
        m_D_Publisher.set(newDValue);
    }

    public boolean isDifferentValues(double previousP, double previousI, double previousD) {
        return getP() != previousP || getI() != previousI ||
                getD() != previousD;
    }

    public boolean isSlotDifferent(long previousSlot) {
        return getSlot() != previousSlot;
    }
}
