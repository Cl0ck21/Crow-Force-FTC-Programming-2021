package org.firstinspires.ftc.teamcode.teamcode.OpModes.TellyOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.teamcode.Robots.MainRobot;
import system.config.StandAlone;
import system.robot.BaseTeleop;
import system.robot.Robot;


@StandAlone
@TeleOp(name = "Main Teleop")
public class MainTeleop extends BaseTeleop {
    MainRobot robot;
    @Override
    protected Robot buildRobot() {
        robot = new MainRobot(this);
        return robot;
    }



    @Override
    protected void onInit() {
        /*robot.mDrive.setStabilityPID(new PIDController(0,0,0));*/
    }
}
