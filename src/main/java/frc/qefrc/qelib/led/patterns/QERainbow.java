/* Copyright (c) 2023 Urban Inspire Corp 501(c)3 d.b.a. Questionable Engineering. All rights reserved. */
/* This work is licensed under the terms of the MPL 2.0 license */
/* found in the root directory of this project. */
package frc.qefrc.qelib.led.patterns;

import frc.qefrc.qelib.led.LEDPatternBase;
import frc.qefrc.qelib.led.LEDSection;
import frc.qefrc.qelib.led.LEDSectionController;

/** A lovely rainbow effect {@category LEDPattern} */
public class QERainbow extends LEDPatternBase {
    private int rainbowFirstPixelHue;

    public QERainbow(LEDSection mySection) {
        super(mySection);
    }

    @Override
    public void setPattern(LEDSectionController control) {
        for (int i = 0; i < control.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final int hue = (rainbowFirstPixelHue + (i * 180 / control.getLength())) % 180;
            // Set the value
            control.setHSV(i, hue, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;
    }

    @Override
    public boolean isAnimated() {
        return true;
    }
}
