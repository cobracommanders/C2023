package org.team498.C2023;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;

public class Compressor {
    private final edu.wpi.first.wpilibj.Compressor compressor;
    private final PneumaticHub pneumaticHub;

    private Compressor() {
        pneumaticHub = new PneumaticHub(Ports.PENUMATIC_HUB);
        compressor = pneumaticHub.makeCompressor();
    }

    public void enableCompressor() {
        compressor.enableDigital();
    }

    public void disableCompressor() {
        compressor.disable();
    }

    public DoubleSolenoid createDoubleSolenoid(int forwardsPort, int reversePort) {
        return pneumaticHub.makeDoubleSolenoid(forwardsPort, reversePort);
    }


    private static Compressor instance;

    public static Compressor getInstance() {
        if (instance == null) {
            instance = new Compressor();
        }

        return instance;
    }
}
