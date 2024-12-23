package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.localization.PinpointLocalizer;

public class DriveConstants {
    // TODO: change the localiser type, to a class that extends localizer
    //   ex : Localizor.class
    public static final Class LOCALIZER_CLASS = PinpointLocalizer.class;
    public static class MotorConfig{
        // TODO: make sure your config has motors with these names (or change them)
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        public static final String LEFT_FRONT = "frontleft";
        public static final String RIGHT_FRONT = "frontright";
        public static final String LEFT_BACK = "backleft";
        public static final String RIGHT_BACK = "backright";
    }
}
