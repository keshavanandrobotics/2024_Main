package teamcode.Autonomous.UsedAutons.MainSpecAuton;



import com.acmerobotics.dashboard.config.Config;

@Config
public class SPEC_AUTO_VARS {


    //PUSHING WAYPOINTS FOR 5+0 - CHRONOLOGICAL - DO NOT EDIT ANY OF THESE

    public static double X1 = 15, Y1 = -30; //INITIAL DIAGONAL STRAFE
    public static double X2 = 55, Y2 = -26; //GOING PAST THE FIRST SPEC
    public static double X3 = 50, Y3 = -31; //STRAFING BEHIND THE FIRST SPEC
    public static double X4 = 10,  Y4 = -30.5; //BRING FIRST SPEC TO HUMAN PLAYER
    public static double X5 = 50, Y5 = -15; //GOING BACK AND BEHIND 2ND SPEC
    public static double X6 = 50,  Y6 = -41; //DRAGGING 2ND SPEC TO HUMAN PLAYER
    public static double X7 = 10, Y7 = -45; //GOING BACK AND BEHIND 3RD SPEC
    public static double X8 = 50,  Y8 = -30; // DRAGGING 3RD SPEC TO HUMAN PLAYER
    public static double X9 = 50, Y9 = -50; // PICKING UP FIRST SPEC
    public static double X10 = 10, Y10 = -46; // GETTING READY TO SCORE FIRST SPEC




    public static double EXTENDO_PICKUP = 30000;






    //Cycling Coordinates

    public static double FIRST_WALL_GRAB_X = -1, FIRST_WALL_GRAB_Y = -22; // SPOT FOR HUMAN PLAYER GRAB FIRST SPEC - DO NOT EDIT
    public static double WALL_GRAB_X = 0.5, WALL_GRAB_Y = -22; //SPOT FOR HUMAN PLAYER GRAB - DO NOT EDIT

    public static double FIRST_SPEC_SCORE_X = 20, FIRST_SPEC_SCORE_Y = 10; // SPOT FOR PRELOAD SCORE

    public static double SPEC_SCORE_X = 26, SPEC_SCORE_Y = 8, SPEC_SCORE_HEADING = 30; //SPOT FOR SCORING in 5+0 - DO NOT EDIT

    public static double SPEC_SCORE_X2 = 24, SPEC_SCORE_Y2 = 6, SPEC_SCORE_HEADING2 = 24; // SPOT FOR SCORING IN 5+1
    public static double RETRY_SPEC_X = 15, RETRY_SPEC_Y = -22; // SPOT TO MOVE TO IF FAILED SPEC - DO NOT EDIT

    // Pickup up coordinates - EDIT THESE
    public static double PICKUP_X1 = 18, PICKUP_Y1 = -15;
    public static double PICKUP_X2 = 18.25, PICKUP_Y2 = -23.5;
    public static double PICKUP_X3 = 19, PICKUP_Y3 = -35;
    public static double PICKUP_HEADING = -45;
    public static double DROP_X1 = 15, DROP_Y1 = -8;
    public static double DROP_X2 = 15, DROP_Y2 = -16;
    public static double DROP_X3 = 15, DROP_Y3 = -24;
    public static double DROP_HEADING = -135;

    //sample score - EDIT THESE
    public static double SAMPLE_SCORE_X1 = 13, SAMPLE_SCORE_Y1 = -25, SAMPLE_SCORE_TRAVELHEADING = -90; // ROTATING AND MOVING AWAY FROM WALL
    public static double SAMPLE_SCORE_X2 = 13, SAMPLE_SCORE_Y2 = 63; // TRAVELING TO BUCKET

    public static double SAMPLE_SCORE_X3 = 0, SAMPLE_SCORE_Y3 = 70, SAMPLE_SCORE_HEADING = -60; // ALIGNING TO BUCKET





    //Trajectory Speeds - SHOULD NOT NEED EDITING - DO NOT EDIT

    //SPEEDS FOR THE PUSHING PATHS
    public static double MAX_PUSHING_VEL = 140;
    public static double MAX_PUSHING_ACCEL = 140;
    public static double MAX_PUSHING_DECCEL = 80; //Only input positive number

    //SPEEDS FOR THE SCORING AND CYCLING PATHS - DO NOT EDIT

    public static double MAX_CYCLING_VEL = 140;
    public static double MAX_CYCLING_ACCEL = 140;
    public static double MAX_CYCLING_DECCEL = 40; //Only input positive number

    //SPEEDS FOR SCORING SAMPLE
    public static double MAX_SAMPLE_VEL = 200;
    public static double MAX_SAMPLE_ACCEL = 200;
    public static double MAX_SAMPLE_DECCEL = 100;

    //SPEEDS FOR PICKUPS
    public static double MAX_PICKUP_VEL = 140;
    public static double MAX_PICKUP_ACCEL = 140;
    public static double MAX_PICKUP_DECCEL = 20;

    //WAITS
    public static double PUSHING_SERVO_ROTATE = 1; // DO NOT EDIT, TIME BEFORE SERVO ROTATES WHILE PUSHING
    public static double PUSHING_SERVO_OPEN = 1; // DO NOT EDIT, TIME BTWN ROTATION AND OPENING
    public static double PUSHING_SERVO_PIVOT = 1; // DO NOT EDIT, TIME BTWN OPENING AND PIVOT MOVING
    public static double PUSHING_SERVO_MOVE = 1; // DO NOT EDIT, TIME BTWN PIVOT MOVING AND MOVE MOVING

    public static double HUMAN_PLAYER_WAIT = 0.1; //TIME ROBOT RESTS BEFORE EXTENDO IN TO GRAB SPECIMEN - DO NOT EDIT

    public static double CLAW_CLOSE_TIME = 0.25; // TIME THAT CLAW CLOSES FOR WALL GRABS - DO NOT EDIT
    public static double CLAW_OPEN_TIME = 0.1; // TIME THAT CLAW OPENS FOR WHEN SCORING SPEC - DO NOT EDIT
    public static double CLAW_DOWN_TIME = 0.1; // TIME BEFORE CLAW MOVES DOWN TO PICKUP SAMPLES
    public static double SAMPLE_DOWN_TIME = 0.3; // TIME FOR MOVE SERVO TO MOVE DOWN TO SCORE SAMPLE

    public static double EXTENDO_OUT_WAIT = 0.8; // TIME FOR EXTENDO TO GO OUT WHILE SCORING - DO NOT EDIT
    public static double EXTENDO_ALL_OUT = 0.75; // TIME FOR EXTENDO TO FULLY EXTEND
    public static double EXTENDO_PICKUP_WAIT = 0.75; // TIME BEFORE EXTENDO GOES OUT TO PICKUP UP FIRST SAMPLE - EDIT THIS
    public static double EXTENDO_FIRST_WAIT = 0.2; // TIME FOR EXTENDO TO FULLY EXTEND TO SCORE PRELOAD SPEC

    public static double EXTENDO_IN_WAIT = 0.1; // TIME FOR EXTENDO TO RETRACT IN SOME CASES - DO NOT EDIT
    public static double MOVE_WAIT = 0.75; // TIME BEFORE MOVE SERVO GOES DOWN TO PICKUP UP FIRST WALL SPEC

    public static double LEAVE_OBS = 1.0; // TIME BETWEEN PICKING UP YELLOW SAMPLE FOR LINEAR SLIDES TO GO UP

    //PID POSITIONS


    public static double EXTENDO_RETRACT_WAIT = 0.5; // TIME FOR EXTENDO TO RETRACT WHEN SCORING SPECS - DO NOT EDIT


    public static int LINEAR_SLIDE_LOWER_THRESHOLD = 1500; // POSITION WHERE LINEAR SLIDE LOWERING ACTION ENDS - DO NOT EDIT

    public static double LINEAR_SLIDES_UP = 3.3;







}