package teamcode.javalimelight.teamcode.Teleop.Singletons;

import java.util.Objects;

public class GamepadJoystickCurve {


    public static double JoystickCurve (double input, String CURVE, Double DEGREE) {
        if (Objects.equals(CURVE, "LINEAR")){
            return input*DEGREE;
        } else if (Objects.equals(CURVE,"POWER")){
            double returner = Math.pow(input, DEGREE);
            if (input<0&&returner>0){
                returner*=-1;
            }
            return returner;
        } else if (Objects.equals(CURVE,"WIDE")){
            if (Math.abs(input)<0.5){
                double returner = Math.pow((2*input), DEGREE);
                returner = returner/2;
                if (input<0&&returner>0){
                    returner *= -1;
                }
                return returner;
            } else {
                double returner = Math.pow(((2*Math.abs(input))-1),(1/DEGREE));
                returner = returner/2;
                returner +=0.5;
                if (input<0&&returner>0){
                    returner*=-1;
                }
                return returner;
            }
        } else if (Objects.equals(CURVE, "GUEST")){

            double returner = DEGREE* input;
            return returner;
        }
        return input;
    }
}