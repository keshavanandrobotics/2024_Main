package teamcode.Teleop.Singletons;

import java.util.List;
import java.util.Objects;

public class DeadZones {

    public static double Linear (double input, double threshold) {
         if (Math.abs(input)>threshold) {
             return input;
         } else {
             return 0;
         }

    }


}
