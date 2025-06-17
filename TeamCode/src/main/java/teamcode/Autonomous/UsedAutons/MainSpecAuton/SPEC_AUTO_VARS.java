package teamcode.Autonomous.UsedAutons.MainSpecAuton;



import com.acmerobotics.dashboard.config.Config;

@Config
public class SPEC_AUTO_VARS {


    //PUSHING WAYPOINTS - CHRONOLOGICAL - DO NOT EDIT ANY OF THESE

    public static double X1 = 15, Y1 = -30; //INITIAL DIAGONAL STRAFE
    public static double X2 = 55, Y2 = -26; //GOING PAST THE FIRST SPEC
    public static double X3 = 50, Y3 = -31; //STRAFING BEHIND THE FIRST SPEC
    public static double X4 = 10,  Y4 = -30.5; //BRING FIRST SPEC TO HUMAN PLAYER
    public static double X5 = 50, Y5 = -20; //GOING BACK AND BEHIND 2ND SPEC
    public static double X6 = 50,  Y6 = -40.5; //DRAGGING 2ND SPEC TO HUMAN PLAYER
    public static double X7 = 10, Y7 = -41; //GOING BACK AND BEHIND 3RD SPEC
    public static double X8 = 50,  Y8 = -30; // DRAGGING 3RD SPEC TO HUMAN PLAYER
    public static double X9 = 50, Y9 = -50; // PICKING UP FIRST SPEC
    public static double X10 = 10, Y10 = -46; // GETTING READY TO SCORE FIRST SPEC











    //Cycling Coordinates

    public static double FIRST_WALL_GRAB_X = -1, FIRST_WALL_GRAB_Y = -22; // SPOT FOR HUMAN PLAYER GRAB FIRST SPEC - DO NOT EDIT
    public static double WALL_GRAB_X = 1, WALL_GRAB_Y = -22; //SPOT FOR HUMAN PLAYER GRAB - DO NOT EDIT

    public static double FIRST_SPEC_SCORE_X = 20, FIRST_SPEC_SCORE_Y = 10; // SPOT FOR PRELOAD SCORE

    public static double SPEC_SCORE_X = 25, SPEC_SCORE_Y = 8, SPEC_SCORE_HEADING = 30; //SPOT FOR SCORING - DO NOT EDIT

    public static double SPEC_SCORE_X2 = 25, SPEC_SCORE_Y2 = 9, SPEC_SCORE_HEADING2 = 20;
    public static double RETRY_SPEC_X = 15, RETRY_SPEC_Y = -22; // SPOT TO MOVE TO IF FAILED SPEC - DO NOT EDIT

    // Pickup up coordinates - EDIT THESE
    public static double PICKUP_X1 = 16, PICKUP_Y1 = -10;
    public static double PICKUP_X2 = 14, PICKUP_Y2 = -22;
    public static double PICKUP_X3 = 14, PICKUP_Y3 = -34;
    public static double PICKUP_HEADING = -45;
    public static double DROP_X1 = 15, DROP_Y1 = -8;
    public static double DROP_X2 = 15, DROP_Y2 = -16;
    public static double DROP_X3 = 15, DROP_Y3 = -24;
    public static double DROP_HEADING = -135;

    //sample score - EDIT THESE
    public static double SAMPLE_SCORE_X1 = 5, SAMPLE_SCORE_Y1 = 68, SAMPLE_SCORE_TRAVELHEADING = -90;
    public static double SAMPLE_SCORE_X2 = 5, SAMPLE_SCORE_Y2 = 72, SAMPLE_SCORE_HEADING = -45;





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

    //WAITS
    public static double PUSHING_SERVO_ROTATE = 1; // DO NOT EDIT, TIME BEFORE SERVO ROTATES WHILE PUSHING
    public static double PUSHING_SERVO_OPEN = 1; // DO NOT EDIT, TIME BTWN ROTATION AND OPENING
    public static double PUSHING_SERVO_PIVOT = 1; // DO NOT EDIT, TIME BTWN OPENING AND PIVOT MOVING
    public static double PUSHING_SERVO_MOVE = 1; // DO NOT EDIT, TIME BTWN PIVOT MOVING AND MOVE MOVING

    public static double HUMAN_PLAYER_WAIT = 0.0; //TIME ROBOT RESTS BEFORE EXTENDO IN TO GRAB SPECIMEN - DO NOT EDIT

    public static double CLAW_CLOSE_TIME = 0.25; //SHOULD NOT NEED EDITING, TIME THAT CLAW CLOSES FOR WALL GRABS - DO NOT EDIT
    public static double CLAW_OPEN_TIME = 0.1; //TIME THAT CLAW OPENS FOR WHEN SCORING SPEC - DO NOT EDIT
    public static double CLAW_DOWN_TIME = 0.1;
    public static double SAMPLE_DOWN_TIME = 0.5;

    public static double EXTENDO_OUT_WAIT = 1; // WAIT TIME FOR EXTENDO TO GO OUT WHILE SCORING - DO NOT EDIT
    public static double EXTENDO_ALL_OUT = 0.5;
    public static double EXTENDO_PICKUP_WAIT = 1;
    public static double EXTENDO_FIRST_WAIT = 0.2;

    public static double EXTENDO_IN_WAIT = 0.1; // DO NOT EDIT
    public static double MOVE_WAIT = 1;

    public static double LEAVE_OBS = 1.0;
    public static double LINEAR_SLIDE_UP = 0.8; // WAIT TIME FOR LINEAR SIDES TO GO UP

    //PID POSITIONS


    public static int EXTENDO_INITIAL_HUMAN_PLAYER = 0; //POSITION BEFORE EXTENDO GOES IN AND GRABS 1ST SPEC

    public static int EXTENDO_GRAB_THRESHOLD = 0; //SHOULD NOT NEED EDITING, POSITION FOR EXTENDO IN WHERE ACTION ENDS

    public static int EXTENDO_SCORE_THRESHOLD = 5000; //SHOULD NOT NEED EDITING, POSITIONS FOR EXTENDO OUT WHERE ACTION ENDS
    public static int EXTENDO_CYCLE_HUMAN_PLAYER = 3000; //POSITION BEFORE EXTENDO GOES IN AND GRABS 2-5 SPEC

    public static double EXTENDO_RETRACT_WAIT = 0.6; // DO NOT EDIT

    public static double FIRST_SCORE_WAIT = 0.8;

    public static int LINEAR_SLIDE_LOWER_THRESHOLD = 1000; // DO NOT EDIT, POSITION WHERE LINEAR SLIDE LOWERING ACTION ENDS







}