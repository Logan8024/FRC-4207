package frc.robot.Utils;

/** Add your docs here. */
public class ToolKit {
    public static boolean isInTolarance(double input, double target, double tolerance) {
        return Math.abs(input - target) <= tolerance;
//        double upLim = target + tolerance;
 //       double downLim = target - tolerance;
  //      if(downLim <= input && input <= upLim) {
   //         return true;
    //    }
     //   else {
      //      return false;
      //  }
    }
}
