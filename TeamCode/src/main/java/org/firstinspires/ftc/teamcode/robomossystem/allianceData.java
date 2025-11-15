package org.firstinspires.ftc.teamcode.robomossystem;

/* Singleton pattern class to pass alliance and starting position between
   auton on teleop.
   Teams 13598 Mobots and 15357 MOreBots
 */
public class allianceData {
    private static allianceData startInfo =null;  //Create instance of alliance data
    public String alliance;  //String to store alliance
    private allianceData() {alliance=null;}
    public static allianceData allianceInstance(){
        if (startInfo ==null){
            startInfo = new allianceData();
        }
        return startInfo;
    }

    public void setAlliance(String alliance) { // set value of alliance
        this.alliance = alliance;
    }

    public String getAlliance() { //Get value of alliance
        return alliance;
    }
}
