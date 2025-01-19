package teamcode.Teleop.Singletons;


import com.acmerobotics.dashboard.config.Config;

@Config

public class Positions {

    public static double CLAW_CLOSED = 0.53
            ;
    public static double CLAW_LOOSE_CLOSED = 0.55;
    public static double CLAW_OPEN = 0.71;
    public static double CLAW_LESS_OPEN = 0.62;

    public static double ROTATE_NEUTRAL = 0.38;

    public static double ROTATE_90 = 0.67;
    public static double ROTATE_FLIP = 0.95;
    public static double ROTATE_LM3_SPECIMEN_AUTON = 0.29;



    public static double MOVE_WALL_INTAKE = 0.93;


    public static double MOVE_OUTTAKE = 0.93;
    public static double MOVE_SPECIMEN_SCORE = 0.46;
    public static double MOVE_ALL_OUT = 0.46;
    public static double MOVE_HOVER_SAMPLE = 0.47;



    public static double MOVE_PICKUP_SAMPLE = 0.43;
    public static double MOVE_AUTONOMOUS_INIT = 0.6;

    public static double PIVOT_OUTTAKE = 0.15;
    public static double PIVOT_WALL_INTAKE = 0.15;

    public static double PIVOT_SPECIMEN_SCORE = 0.5;
    public static double PIVOT_ALL_OUT = 0.64;
    public static double PIVOT_SAMPLE_PICKUP = 0.95;

    public static double PIVOT_AUTONOMOUS_INIT = 0.17;

    public static int HIGH_SPECIMEN_POS = 18200;
    public static int AUTO_PARK_SLIDE_POS = 28000;

    public static int HIGH_SAMPLE_POS = 61000;

    public static int EXTENDO_MAX_TELE = 21000;







}
