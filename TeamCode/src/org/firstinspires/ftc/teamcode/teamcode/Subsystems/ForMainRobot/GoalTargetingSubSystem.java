package org.firstinspires.ftc.teamcode.teamcode.Subsystems.ForMainRobot;



import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teamcode.Robots.MainRobot;
import system.config.ConfigParam;
import system.config.TeleopConfig;
import system.robot.Robot;
import system.robot.SubSystem;
import util.control.Button;
import util.control.CustomizableGamepad;
import util.control.Toggle;


public class GoalTargetingSubSystem extends SubSystem {
    private DcMotor mot;
    public final int GOAL1_X = 250;
    public final int GOAL1_Y = 0;
    public final int GOAL2_X = 750;
    public final int GOAL2_Y = 0;
    public final int SHOTLINE_Y = 500;
    public final double TRUEDISTANCEMAX = Math.sqrt(Math.pow(500, 2) + Math.pow(SHOTLINE_Y, 2));
    CustomizableGamepad inputs;
    static final String BUTTONIDENTIFIER = "AimButton";
    Toggle toggle = new Toggle(Toggle.ToggleTypes.trueOnceToggle , false);

    public GoalTargetingSubSystem(Robot r, String turnMotor) {
        super(r);
        mot = robot.hardwareMap.dcMotor.get(turnMotor);
        usesConfig = true;
    }
    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        inputs = robot.pullControls(this);
    }

    @Override
    public void handle() {
        toggle.updateToggle(inputs.getInput(BUTTONIDENTIFIER));
        if(toggle.getCurrentState()) {
            ((MainRobot)robot).mDrive.turnPID(findAngle()); //need new mecanum drive first
            //todo something that sends the value from findPower() to the subsystem that fires the disc, without actually firing the disc
        }
    }

    @Override
    public void stop() {

    }

    //returns the angle that the robot needs to be at to fire at the closest goal
    public double findAngle() {
        double[] nums = findDistances(true);
        if(nums[2] == 1) //left goal is closer
            return 360-Math.asin(nums[0]/nums[1]);
        else //right goal is closer
            return Math.asin(nums[0]/nums[1]);
    }
    //todo power returned is percentage of max distance. When we have the robot, use it as input in a line of best fit equation created with trial values to determine true power.
    //returns the power needed to fire the disc to the goal
    public double findPower(){
        double distance = findDistances(false)[0];
        return 100*(distance/TRUEDISTANCEMAX);
    }

    //calculates distances for other methods to use
    public double[] findDistances(boolean methodSwitch) {
        double x = ((MainRobot)robot).odometry.returnXCoordinate();
        double y = ((MainRobot)robot).odometry.returnYCoordinate();
        if(Math.abs(x-GOAL1_X) <= Math.abs(x-GOAL2_X)) { //left goal is closer
            double xDistance = Math.abs(x-GOAL1_X);
            double yDistance = Math.abs(y-GOAL1_Y);
            double trueDistance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
            if(methodSwitch)
                return new double[] {xDistance, trueDistance, 1};
            else
                return new double[] {trueDistance};
        }
        else{ //right goal is closer
            double xDistance = Math.abs(x-GOAL2_X);
            double yDistance = Math.abs(y-GOAL2_Y);
            double trueDistance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
            if(methodSwitch)
                return new double[] {xDistance, trueDistance, 2};
            else
                return new double[] {trueDistance};
        }
    }


    @TeleopConfig
    public static ConfigParam[] teleopConfig() {
        return new ConfigParam[]{
                new ConfigParam(BUTTONIDENTIFIER, Button.BooleanInputs.y, 2)
        };
    }
}
