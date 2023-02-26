package org.team498.lib.drivers;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import static org.team498.C2023.Ports.Accessories.BLINKIN;

public class Blinkin {
    private final Spark blinkin;

    /** Additional colors can be found <a href=https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf>here</a> */
    public enum Color {
        RED(.61), // -.25 for breath
        BLUE(-.23), // -.23 for breath//.87
        GREEN(.77),
        WHITE(-.21),
        GRAY(-.13),
        YELLOW(.69),
        PURPLE(.91);

        public double val;

        private Color(double val) {
            this.val = val;
        }
    }

    private Blinkin() {
        blinkin = new Spark(BLINKIN);
    }

    public void setColor(Color color) {
        set(color.val);
    }

    public void set(double val) {
        if ((val >= -1.0) && (val <= 1.0)) {
            blinkin.set(val);
        }
    }

    private static Blinkin instance;

    public static Blinkin getInstance() {
        if (instance == null) {
            instance = new Blinkin();
        }
        return instance;
    }
}