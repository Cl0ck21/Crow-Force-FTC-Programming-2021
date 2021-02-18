package org.firstinspires.ftc.teamcode.teamcode.OpModes.OneTimeOp;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.teamcode.Robots.MotorTuningRobot;
import system.config.StandAlone;
import system.robot.BaseTeleop;
import system.robot.Robot;
import util.control.Toggle;
import util.math.geometry.Vector;
import util.math.geometry.Vector2D;


@StandAlone
@TeleOp(name = "MotorSpeedTuning")
public class MotorSpeedTuningOp extends BaseTeleop {
    MotorTuningRobot asd;
    @Override
    protected Robot buildRobot() {
        asd = new MotorTuningRobot(this);
        return asd;
    }
    Vector2D driveVector;
    double speed;
    Toggle toggle1;
    Toggle toggle2;
    Toggle driveToggle;
    @Override
    public void onInit(){
        //telemetry.setAutoClear(true);
        driveVector = new Vector2D(0, .3);
        speed = .3;
        toggle1 = new Toggle(Toggle.ToggleTypes.trueOnceToggle, false);
        toggle2 = new Toggle(Toggle.ToggleTypes.trueOnceToggle, false);
        driveToggle = new Toggle(Toggle.ToggleTypes.trueOnceToggle, false);
    }
    @Override
    public void onUpdate() {
        toggle1.updateToggle(asd.gamepad1.a);
        toggle2.updateToggle(asd.gamepad1.y);
        driveToggle.updateToggle(asd.gamepad1.b);
        if(toggle1.getCurrentState()){
            speed -= .1;
        }
        if(toggle2.getCurrentState()){
            speed += .1;
        }
        driveVector = new Vector2D(0, speed);
        telemetry.addData("current speed", speed);
        if(driveToggle.getCurrentState()) {
            asd.mDrive.moveTime(driveVector, 1000);
        }
        //telemetry.update();
    }
}
