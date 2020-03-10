package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class BeamTripTrig extends Trigger{
    DigitalInput beam;
    BeamTripTrig(DigitalInput beamIn){
        beam = beamIn;
    }

    @Override
    public boolean get(){ //Should return true if the beam is BROKEN.  If this isn't true in testing, fix in code
                            //Related- TODO: Test that the above is true
        return beam.get();
    }
}