package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class BeamTripTrig extends Trigger{
    DigitalInput beam;
    BeamTripTrig(DigitalInput beamIn){
        beam = beamIn;
    }

    @Override
    public boolean get(){
        return beam.get();
    }
}