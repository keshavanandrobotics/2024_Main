package teamcode.Autonomous.UsedAutons.MainSpecAuton;



import com.acmerobotics.dashboard.config.Config;

@Config
public class SPEC_AUTO_VARS {


    //PUSHING WAYPOINTS - CHRONOLOGICAL

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

    public static double FIRST_WALL_GRAB_X = -1, FIRST_WALL_GRAB_Y = -22; // SPOT FOR HUMAN PLAYER GRAB FIRST SPEC
    public static double WALL_GRAB_X = 1, WALL_GRAB_Y = -22; //SPOT FOR HUMAN PLAYER GRAB

    public static double FIRST_SPEC_SCORE_X = 25, FIRST_SPEC_SCORE_Y = 8; // SPOT FOR PRELOAD SCORE

    public static double SPEC_SCORE_X = 25, SPEC_SCORE_Y = 8, SPEC_SCORE_HEADING = 30; //SPOT FOR SCORING
    public static double RETRY_SPEC_X = 15, RETRY_SPEC_Y = -22; // SPOT TO MOVE TO IF FAILED SPEC

    //Trajectory Speeds - SHOULD NOT NEED EDITING

    //SPEEDS FOR THE PUSHING PATHS
    public static double MAX_PUSHING_VEL = 140;
    public static double MAX_PUSHING_ACCEL = 140;
    public static double MAX_PUSHING_DECCEL = 80; //Only input positive number

    //SPEEDS FOR THE SCORING AND CYCLING PATHS

    public static double MAX_CYCLING_VEL = 140;
    public static double MAX_CYCLING_ACCEL = 140;
    public static double MAX_CYCLING_DECCEL = 40; //Only input positive number

    //WAITS
    public static double PUSHING_SERVO_ROTATE = 1; //SHOULD NOT NEED EDITING, TIME BEFORE SERVO ROTATES WHILE PUSHING
    public static double PUSHING_SERVO_OPEN = 1; //SHOULD NOT NEED EDITING, TIME BTWN ROTATION AND OPENING
    public static double PUSHING_SERVO_PIVOT = 1; //SHOULD NOT NEED EDITING, TIME BTWN OPENING AND PIVOT MOVING
    public static double PUSHING_SERVO_MOVE = 1; //SHOULD NOT NEED EDITING, TIME BTWN PIVOT MOVING AND MOVE MOVING

    public static double HUMAN_PLAYER_WAIT = 0.1; //TIME ROBOT RESTS BEFORE EXTENDO IN TO GRAB SPECIMEN

    public static double CLAW_CLOSE_TIME = 0.25; //SHOULD NOT NEED EDITING, TIME THAT CLAW CLOSES FOR WALL GRABS
    public static double CLAW_OPEN_TIME = 0.1; //TIME THAT CLAW OPENS FOR WHEN SCORING SPEC

    public static double EXTENDO_OUT_WAIT = 1; //WAIT TIME FOR EXTENDO TO GO OUT WHILE SCORING

    public static double EXTENDO_IN_WAIT = 0.1;
    public static double LINEAR_SLIDE_UP = 0.8; // WAIT TIME FOR LINEAR SIDES TO GO UP

    //PID POSITIONS


    public static int EXTENDO_INITIAL_HUMAN_PLAYER = 0; //POSITION BEFORE EXTENDO GOES IN AND GRABS 1ST SPEC

    public static int EXTENDO_GRAB_THRESHOLD = 0; //SHOULD NOT NEED EDITING, POSITION FOR EXTENDO IN WHERE ACTION ENDS

    public static int EXTENDO_SCORE_THRESHOLD = 5000; //SHOULD NOT NEED EDITING, POSITIONS FOR EXTENDO OUT WHERE ACTION ENDS
    public static int EXTENDO_CYCLE_HUMAN_PLAYER = 3000; //POSITION BEFORE EXTENDO GOES IN AND GRABS 2-5 SPEC

    public static double EXTENDO_RETRACT_WAIT = 0.6;

    public static double FIRST_SCORE_WAIT = 0.8;

    public static int LINEAR_SLIDE_LOWER_THRESHOLD = 500; //SHOULD NOT NEED EDITING, POSITION WHERE LINEAR SLIDE LOWERING ACTION ENDS







}