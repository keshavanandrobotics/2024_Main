package teamcode.Autonomous;


import static teamcode.Teleop.Singletons.VARS.*;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SPEC_AUTO_VARS {

    public static double SPEC_VECTOR_X = 16;

    public static double SPEC_VECTOR_Y = -50.5;

    public static double SPEC_LEFT_HEADING = 25;

    public static double SPEC_CENTER_HEADING = -3;


    public static double SPEC_RIGHT_HEADING = -27;

    public static double PICKUP_WAIT_TIME = 0.2;

    public static double CLAW_CLOSE_WAIT_TIME =0.2;

    public static int LEFT_SPEC_EXTENDO_POS = 12500;
    public static int CENTER_SPEC_EXTENDO_POS = 9000;
    public static int RIGHT_SPEC_EXTENDO_POS = 12500;




    public static double CLAW_ROTATE_LEFT_SPEC = ROTATE_NEUTRAL +.09;

    public static double CLAW_ROTATE_CENTER_SPEC = ROTATE_NEUTRAL;

    public static double CLAW_ROTATE_RIGHT_SPEC = ROTATE_NEUTRAL - 0.09;

    public static double SERVOS_IN_WAIT_TIME = 0.55;

    public static double SERVOS_OUT_WAIT_TIME = 0.5;

    public static double CLAW_DROP_WAIT_TIME  = 0.2;








}

