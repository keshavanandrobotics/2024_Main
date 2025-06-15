package teamcode.javalimelight.teamcode.Teleop.Singletons;

public class DeadZones {

    public static double Linear (double input, double threshold) {
         if (Math.abs(input)>threshold) {
             return input;
         } else {
             return 0;
         }

    }


}
