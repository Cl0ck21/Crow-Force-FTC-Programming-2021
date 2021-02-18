package org.firstinspires.ftc.teamcode.teamcode.OpModes.TellyOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.teamcode.Robots.PID_TunerRobot;
import system.config.StandAlone;
import system.robot.BaseTeleop;
import system.robot.Robot;


@StandAlone
@TeleOp(name = "PID Tuner")
public class PID_Tuner extends BaseTeleop {

    PID_TunerRobot asd;
    @Override
    protected Robot buildRobot() {
        asd = new PID_TunerRobot(this);
        return asd;
    }
}
