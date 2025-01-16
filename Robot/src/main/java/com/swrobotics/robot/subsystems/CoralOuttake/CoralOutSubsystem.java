package com.swrobotics.robot.subsystems.CoralOuttake;

import java.security.PublicKey;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.swrobotics.lib.net.NTEntry;
import com.swrobotics.robot.config.Constants;

import com.swrobotics.robot.subsystems.motortracker.MotorTrackerSubsystem;
import com.swrobotics.robot.subsystems.music.MusicSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CoralOutSubsystem extends SubsystemBase {
    public enum State {
        
        //sets speed 

        OFF(Constants.kCoralOutIdleSpeed),
        CORALOUT(Constants.kCoralOutSpeed),
        EJECT(Constants.kCoralOuteEjectSpeedTop);

        final NTEntry<Double> topSpeed; 
            
            State(NTEntry<Double> topSpeed) {
                this.topSpeed = topSpeed;                
            }

        }  
        //give value to motor's

        TalonFX Motor1 = new TalonFX(0);

        public CoralOutSubsystem() {
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            Motor1.getConfigurator().apply(config);
            
            MotorTrackerSubsystem.getInstance().addMotor("coralOut", Motor1);
            MusicSubsystem.getInstance().addInstrument(Motor1);
        }
            public void setState(State state){
                Motor1.set(state.topSpeed.get());

            }
            
    }
