package org.team498.lib.drivers;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import static org.team498.C2023.Ports.Accessories.BLINKIN;

public class Blinkin {
    private final Spark blinkin;
    private Color color = Color.BLUE;

    /** Additional colors can be found <a href=https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf>here</a> */
    public enum Color {
        RED(0.61),
        BLUE(0.83),
        LIME(0.73),
        WHITE(0.93),
        GRAY(0.95),
        YELLOW(0.69),
        PURPLE(0.91);

        public double val;

        private Color(double val) {
            this.val = val;
        }
    }

    private Blinkin() {
        blinkin = new Spark(BLINKIN);
    }

    public void setColor(Color color) {
        this.color = color;
        set(color.val);
    }

    public Color getColor() {
        return color;
    }

    public void set(double val) {
        if ((val >= -1.0) && (val <= 1.0)) {
            // pwm.setSpeed(val);
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