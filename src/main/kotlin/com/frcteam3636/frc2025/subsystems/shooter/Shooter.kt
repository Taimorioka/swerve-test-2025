package com.frcteam3636.frc2025.subsystems.shooter

import com.frcteam3636.frc2025.REVMotorControllerId
import com.frcteam3636.frc2025.SparkMax
import com.revrobotics.spark.SparkBase.*
import com.revrobotics.spark.SparkLowLevel
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem

object Shooter: Subsystem {

    private var motor1 = SparkMax(REVMotorControllerId.TestMotor1, SparkLowLevel.MotorType.kBrushless).apply {
        val innerConfig = SparkMaxConfig().apply {
            idleMode(IdleMode.kBrake)
            smartCurrentLimit(37)
            inverted(false)
        }
        configure(innerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    private var motor2 = SparkMax(REVMotorControllerId.TestMotor2, SparkLowLevel.MotorType.kBrushless).apply {
        val innerConfig = SparkMaxConfig().apply {
            idleMode(IdleMode.kBrake)
            smartCurrentLimit(37)
            inverted(true)
        }
        configure(innerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    fun spinUp(multiplier: Double): Command =startEnd(
        {
            motor1.setVoltage(12.0 * multiplier)
            motor2.setVoltage(12.0 * multiplier)
        },
        {
            motor1.setVoltage(0.0)
            motor2.setVoltage(0.0)
        }
    )
}