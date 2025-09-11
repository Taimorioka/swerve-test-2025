package com.frcteam3636.frc2025.subsystems.funnel

import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.frcteam3636.frc2025.CTREDeviceId
import com.frcteam3636.frc2025.Robot
import com.frcteam3636.frc2025.TalonFX
import com.frcteam3636.frc2025.utils.math.amps
import com.frcteam3636.frc2025.utils.math.inAmps
import com.frcteam3636.frc2025.utils.math.inVolts
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.math.system.plant.LinearSystemId
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.simulation.FlywheelSim
import org.team9432.annotation.Logged

@Logged
open class FunnelInputs {
//    var rollerVelocity = 0.rotationsPerSecond
//    var rollerCurrent = 0.amps
}

interface FunnelIO {
    fun setSpeed(percent: Double)
    fun setVoltage(voltage: Voltage)
    fun updateInputs(inputs: FunnelInputs)
}

class FunnelIOReal : FunnelIO {
    private var rampMotor = TalonFX(CTREDeviceId.FunnelMotor).apply {
        configurator.apply(
            TalonFXConfiguration().apply {
                MotorOutput.apply {
                    NeutralMode = NeutralModeValue.Coast
                    Inverted = InvertedValue.CounterClockwise_Positive
                }

                CurrentLimits.apply {
                    SupplyCurrentLimit = MOTOR_CURRENT_LIMIT.inAmps()
                    SupplyCurrentLimitEnable = true
                }
            }
        )
    }

    override fun setSpeed(percent: Double) {
        assert(percent in -1.0..1.0)
        rampMotor.set(percent)
    }

    override fun setVoltage(voltage: Voltage) {
        assert(voltage.inVolts() in -12.0..12.0)
        rampMotor.setVoltage(voltage.inVolts())
    }

    override fun updateInputs(inputs: FunnelInputs) {
//        inputs.rollerVelocity = rampMotor.velocity.value
//        inputs.rollerCurrent = rampMotor.supplyCurrent.value
    }

    internal companion object Constants {
        private val MOTOR_CURRENT_LIMIT = 35.amps
    }
}

class FunnelIOSim : FunnelIO {
    private var motor = DCMotor.getKrakenX60(1)
    private var system = LinearSystemId.createFlywheelSystem(motor, 1.0, 5.0)
    private var simMotor = FlywheelSim(system, motor, 0.0)

    override fun setSpeed(percent: Double) {
        simMotor.inputVoltage = percent * 12
    }

    override fun setVoltage(voltage: Voltage) {
        TODO("Not yet implemented")
    }

    override fun updateInputs(inputs: FunnelInputs) {
        simMotor.update(Robot.period)
//        inputs.rollerVelocity = simMotor.angularVelocity
        simMotor.setAngularVelocity(simMotor.angularVelocityRadPerSec * 0.95)
//        inputs.rollerCurrent = simMotor.currentDrawAmps.amps
    }

}
