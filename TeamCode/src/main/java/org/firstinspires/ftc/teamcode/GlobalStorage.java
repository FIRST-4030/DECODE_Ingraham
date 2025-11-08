package org.firstinspires.ftc.teamcode;

public class GlobalStorage {
    private static String pattern;
    private static int alliance;

    public static String getPattern(){return pattern;}

    public static void setPattern(String p) {pattern = p;}
    public static int getAlliance(){return alliance;}

    public static void setAlliance(int a) {alliance = a;}

    //put at the beginning auto init to stop it from carry over variable from previous matches: GlobalStorage.setPattern(null);
    // when you read the obelisk: GlobalStorage.setPattern(p);
    // in teleop: myPattern = getPattern();
    //WARNING: in case of errors in obelisk reading or inproper set-up of class, pattern can equal null, plan for that case and
    //MAKE SURE, there is no chance of a null pointer exeception. (this is not solved by removing setting it to null in init, it could still load wrong)
}
