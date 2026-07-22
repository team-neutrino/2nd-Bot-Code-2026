package frc.robot.subsystems.network_tables;

import static frc.robot.util.Constants.IntakeConstants.*;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.subsystems.Intake;
import frc.robot.util.PIDTuner;

public class IntakeNT extends Intake {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();

    DoubleTopic currentMotorAngle;
    DoubleTopic targetMotorAngle;
    DoubleTopic rollerRPM;

    final DoublePublisher currentMotorAnglePub;
    final DoublePublisher targetMotorAnglePub;
    final DoublePublisher rollerRPMPub;

    private PIDTuner m_deployPIDTuner;

    private double m_previousDeployKP;
    private double m_previousDeployKI;
    private double m_previousDeployKD;

    public IntakeNT() {
        currentMotorAngle = nt.getDoubleTopic("/intake/current_motor_angle");
        targetMotorAngle = nt.getDoubleTopic("/intake/target_motor_angle");
        rollerRPM = nt.getDoubleTopic("/intake/roller_rpm");

        currentMotorAnglePub = currentMotorAngle.publish();
        currentMotorAnglePub.setDefault(0.0);

        targetMotorAnglePub = targetMotorAngle.publish();
        targetMotorAnglePub.setDefault(0.0);

        rollerRPMPub = rollerRPM.publish();
        rollerRPMPub.setDefault(0.0);

        m_deployPIDTuner = new PIDTuner("intake/{tuning}deployMotor", false);

        m_deployPIDTuner.setP(INTAKE_kP);
        m_deployPIDTuner.setI(INTAKE_kI);
        m_deployPIDTuner.setD(INTAKE_kD);
    }

    @Override
    public void periodic() {
        super.periodic();
        final long now = NetworkTablesJNI.now();

        currentMotorAnglePub.set(getMotorAngle(), now);
        targetMotorAnglePub.set(getTargetAngle(), now);
        rollerRPMPub.set(getRollerRPM(), now);

        if (m_deployPIDTuner.isDifferentValues(m_previousDeployKP,
                m_previousDeployKI, m_previousDeployKD)) {
            m_previousDeployKP = m_deployPIDTuner.getP();
            m_previousDeployKI = m_deployPIDTuner.getI();
            m_previousDeployKD = m_deployPIDTuner.getD();
            setIntakePID(m_deployPIDTuner.getP(), m_deployPIDTuner.getI(),
                    m_deployPIDTuner.getD());
        }
    }
}
