package org.firstinspires.ftc.teamcode.teamcode.OpModes.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.teamcode.Robots.MainRobot;
import system.config.StandAlone;
import system.robot.BaseAutonomous;


@StandAlone
@Autonomous(name = "Old Autonomous")
public class Old_Autonomouss extends BaseAutonomous {
    @system.robot.MainRobot
    private MainRobot robot;/*
    private EncoderToDistanceProcessor processor = new EncoderToDistanceProcessor(2.5, Units.INCH);
    private EncoderToDistanceProcessor cmprocessor = new EncoderToDistanceProcessor(6, Units.CENTIMETERS);
   // list of common encoder
  //  distances:1tile
    int oneTile = processor.getEncoderAmount(18, Units.INCH);
    //2tiles
    int twoTile = processor.getEncoderAmount(35, Units.INCH);
    //2.5

   // tiles(right before of intake)

    int blockPos = processor.getEncoderAmount(7, Units.INCH);

    private void turnEncoders(double power, int encoder) {
        int encoderStart = robot.distance.fEncoders();
        robot.mDrive.turn(power);
        while (Math.abs(robot.distance.fEncoders() - encoderStart) < encoder) {
            double temp = Math.abs(robot.distance.fEncoders() - encoderStart);
        }
        robot.mDrive.stopAllMotors();
    }

    private void strafeEncoders(double power, int encoder) {
        int encoderStart = robot.distance.sEncoders();
        robot.mDrive.drive(new Vector(power, 0));
        while (Math.abs(robot.distance.sEncoders() - encoderStart) < encoder) {
            double temp = Math.abs(robot.distance.sEncoders() - encoderStart);
            telemetry.addData("Strafe Encoders:", robot.distance.sEncoders());
            telemetry.update();
        }
        robot.mDrive.stopAllMotors();
    }

    private void stopTime(){
        long startTime = System.currentTimeMillis();
        robot.mDrive.stopAllMotors();

        while (System.currentTimeMillis() - startTime < 1000) {
        }
    }



    private void driveEncoders(double power, int encoder) {
        int encoderStart = robot.distance.fEncoders();
        robot.mDrive.drive(new Vector(0, -power));
        telemetry.addData("Before: ", -power);
        telemetry.update();
        while (Math.abs(robot.distance.fEncoders() - encoderStart) < encoder) {
            double temp = Math.abs(robot.distance.fEncoders() - encoderStart);
            telemetry.addData("Forward Encoders:", robot.distance.fEncoders());
            telemetry.addData("Before: ", -power);
            telemetry.update();
        }
        robot.mDrive.stopAllMotors();
    }

    public void nin(int distDif) throws InterruptedException {
      //  45 degrees is 500 ms
        //turnEncoders(0.3, 400 + processor.getEncoderAmount(distDif, Units.INCH));
        robot.mDrive.turnTo(90);
    }

    public static double mod(double number, double number2){
        byte negative = 1;

        if(number < 0){
            negative = -1;
        }
        number = Math.abs(number);

        while (number > number2){
            number -= number2;
        }
        return number * negative;
    }

    public void nin() {
        //45 degrees is 500 ms
        turnTo(-Math.PI/2);
    }

    public void negnin(){
        //turnEncoders(-0.3, 400 + processor.getEncoderAmount(distDif, Units.INCH));
        turnTo(Math.PI/2);
    }



    public void getBlock1res(String color){
        if (color.equalsIgnoreCase("Blue")){
            driveEncoders(-pow1, 1500);
            nin();
            strafeEncoders(-pow1, oneTile);
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, oneTile);
            driveEncoders(-pow1, specBlock1);
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, specBlock1);
            driveEncoders(-pow1, 1500);
            strafeEncoders(-pow1, oneTile);
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, oneTile);
            driveEncoders(-pow1, specBlock1);
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, twoTile);

        }
        else{
            driveEncoders(pow1, 1500);
            nin();
            strafeEncoders(-pow1, oneTile);
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, oneTile);
            driveEncoders(-pow1, specBlock3);
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, specBlock3);
            driveEncoders(-pow1, 1500);
            strafeEncoders(-pow1, oneTile);
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, oneTile);
            driveEncoders(-pow1, specBlock3);
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, twoTile);
        }
    }

    public void getBlock2res(String color){
        if (color.equalsIgnoreCase("Blue")){
            driveEncoders(-pow1, 1500);
            nin();
            driveEncoders(-pow1, 2000);
            strafeEncoders(-pow1, oneTile);
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, oneTile);
            driveEncoders(-pow1, specBlock2);
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, specBlock2);
            driveEncoders(-pow1, 1500);
            strafeEncoders(-pow1, oneTile);
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, oneTile);
            driveEncoders(-pow1, specBlock2);
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, twoTile);

        }
        else{
            driveEncoders(pow1, 1500);
            nin();
            driveEncoders(-pow1, 2000);
            strafeEncoders(-pow1, oneTile);
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, oneTile);
            driveEncoders(-pow1, specBlock2);
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, specBlock2);
            driveEncoders(-pow1, 1500);
            strafeEncoders(-pow1, oneTile);
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, oneTile);
            driveEncoders(-pow1, specBlock2);
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, twoTile);
        }
    }

    public void getBlock3res(String color){
        if (color.equalsIgnoreCase("Blue")){
            driveEncoders(-pow1, 1500);
            nin();
            driveEncoders(-pow1, 2500);
            strafeEncoders(-pow1, oneTile);
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, oneTile);
            driveEncoders(-pow1, specBlock3);
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, specBlock3);
            driveEncoders(-pow1, 1500);
            strafeEncoders(-pow1, oneTile);
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, oneTile);
            driveEncoders(-pow1, specBlock3);
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, twoTile);

        }
        else{
            driveEncoders(-pow1, 1500);
            nin();
            driveEncoders(pow1, 2500);
            strafeEncoders(-pow1, oneTile);
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, oneTile);
            driveEncoders(-pow1, specBlock1);
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, specBlock1);
            driveEncoders(-pow1, 1500);
            strafeEncoders(-pow1, oneTile);
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, oneTile);
            driveEncoders(-pow1, specBlock1);
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, twoTile);
        }
    }

    public void moveFound(String color) {
        if (color.equalsIgnoreCase("Red")) {
            robot.grabber.toggleDown();
            long asd = System.currentTimeMillis();
            while (System.currentTimeMillis() - asd <2000) {}

            robot.mDrive.driveTime(new Vector(0,1), 3700);
            //driveEncoders(pow1, cmprocessor.getEncoderAmount(60, Units.CENTIMETERS));
            //turnTo(-3*PI/2);
            robot.grabber.toggleUp();
        } else {
            robot.grabber.toggleDown();
            long asd = System.currentTimeMillis();
            while (System.currentTimeMillis() - asd <2000) {}
            robot.mDrive.driveTime(new Vector(0,1), 3700);
            //turnTo(-PI/2);
            robot.grabber.toggleUp();
        }
    }

    public void maxPoints1Res(String color) {
        if (color.equalsIgnoreCase("Blue")) {
            robot.grabber.toggleUp();
            driveEncoders(pow1, cmprocessor.getEncoderAmount(3,Units.CENTIMETERS));
            nin();
            driveEncoders(pow1, cmprocessor.getEncoderAmount(8, Units.CENTIMETERS));
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(64, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeDown();
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < 1000) {
            }
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(3, Units.CENTIMETERS));
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(30, Units.CENTIMETERS));
            driveEncoders(-pow1, specBlock1- cmprocessor.getEncoderAmount(10,Units.CENTIMETERS));
            robot.blockIntakeServo.intakeUp();
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(5, Units.CENTIMETERS));
           /* driveEncoders(-pow1, cmprocessor.getEncoderAmount(20, Units.CENTIMETERS));
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(10, Units.CENTIMETERS));
            turnTo(-PI);
            driveEncoders(-pow1, cmprocessor.getEncoderAmount(20, Units.CENTIMETERS));
            moveFound("Blue");*//*
            robot.mDrive.driveTime(new Vector(0,-1), 4500);
            //following strafe is gonna be a different number
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(15, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeDown();
            long startTime2 = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime2 < 1000) {
            }
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(40, Units.CENTIMETERS));
            driveEncoders(-pow1, specBlock1+cmprocessor.getEncoderAmount(60,Units.CENTIMETERS));
            //negnin(); negnin(); driveEncoders(pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, blockPos); negnin(); negnin();
            *//*
            robot.blockIntakeServo.intakeUp();
            driveEncoders(pow1, cmprocessor.getEncoderAmount(30, Units.CENTIMETERS));
        } else { /*nin(); strafeEncoders(-pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, blockPos); driveEncoders(pow1, specBlock3);
        *//*
            robot.grabber.toggleUp();
            robot.blockIntakeServo.intakeUp();
            driveEncoders(pow1, cmprocessor.getEncoderAmount(5,Units.CENTIMETERS));
            nin();
            driveEncoders(-pow1, cmprocessor.getEncoderAmount(19,Units.CENTIMETERS)-100);
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(62, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(2, Units.CENTIMETERS));
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < 1000) {
            }
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(30, Units.CENTIMETERS));
            driveEncoders(pow1, specBlock2 - cmprocessor.getEncoderAmount(60, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, specBlock2 - cmprocessor.getEncoderAmount(56, Units.CENTIMETERS));
            turnTo(-3* PI/2);
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(98, Units.CENTIMETERS));
            //driveEncoders(pow1, cmprocessor.getEncoderAmount(20, Units.CENTIMETERS));
            robot.mDrive.driveTime(new Vector(0,1), 1000);
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(14, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(6, Units.CENTIMETERS));
            long startTime2 = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime2 < 1000) {
            }
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(59, Units.CENTIMETERS));
            driveEncoders(-pow1, specBlock2 - cmprocessor.getEncoderAmount(42, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeUp();
            driveEncoders(pow1, cmprocessor.getEncoderAmount(20, Units.CENTIMETERS));
        }
    }

    public void maxPoints2Res(String color) {
        if (color.equalsIgnoreCase("Blue") ) {
            robot.grabber.toggleUp();
            driveEncoders(pow1, cmprocessor.getEncoderAmount(3,Units.CENTIMETERS));
            nin();
            telemetry.addData("angle", robot.mDrive.getCurrentAngle());
            driveEncoders(-pow1, cmprocessor.getEncoderAmount(9, Units.CENTIMETERS));
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(64, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeDown();
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < 1000) {
            }
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(3, Units.CENTIMETERS));
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(30, Units.CENTIMETERS));
            driveEncoders(-pow1, specBlock1- cmprocessor.getEncoderAmount(32,Units.CENTIMETERS));
            robot.blockIntakeServo.intakeUp();
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(5, Units.CENTIMETERS));
           /* driveEncoders(-pow1, cmprocessor.getEncoderAmount(20, Units.CENTIMETERS));
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(10, Units.CENTIMETERS));
            turnTo(-PI);
            driveEncoders(-pow1, cmprocessor.getEncoderAmount(20, Units.CENTIMETERS));
            moveFound("Blue");*//*
            driveEncoders(pow1, specBlock1+ cmprocessor.getEncoderAmount(32,Units.CENTIMETERS));
            //following strafe is gonna be a different number
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(15, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeDown();
            long startTime2 = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime2 < 1000) {
            }
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(3, Units.CENTIMETERS));
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(30, Units.CENTIMETERS));
            driveEncoders(-pow1, specBlock1+cmprocessor.getEncoderAmount(30,Units.CENTIMETERS));
            //negnin(); negnin(); driveEncoders(pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, blockPos); negnin(); negnin();
            *//*
            robot.blockIntakeServo.intakeUp();
            driveEncoders(pow1, cmprocessor.getEncoderAmount(24, Units.CENTIMETERS));
        } else { /*nin(); strafeEncoders(-pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, blockPos); driveEncoders(pow1, specBlock3);
        *//*
            robot.grabber.toggleUp();
            robot.blockIntakeServo.intakeUp();
            driveEncoders(pow1, cmprocessor.getEncoderAmount(5,Units.CENTIMETERS));
            nin();
            telemetry.addData("angle", robot.mDrive.getCurrentAngle());
            driveEncoders(-pow1, cmprocessor.getEncoderAmount(1,Units.CENTIMETERS));
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(67, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(2, Units.CENTIMETERS));
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < 1000) {
            }
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(34, Units.CENTIMETERS));
            driveEncoders(pow1, specBlock2 - cmprocessor.getEncoderAmount(70, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, specBlock2 - cmprocessor.getEncoderAmount(74, Units.CENTIMETERS));
            turnTo(-3* PI/2);
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(90, Units.CENTIMETERS));
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(3, Units.CENTIMETERS));
            driveEncoders(pow1, cmprocessor.getEncoderAmount(12, Units.CENTIMETERS));
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(14, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(6, Units.CENTIMETERS));
            long startTime2 = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime2 < 1000) {
            }
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(48, Units.CENTIMETERS));
            turnTo((-4*PI)/3);
            turnTo(-3* PI/2);
            driveEncoders(-pow1, specBlock2 - cmprocessor.getEncoderAmount(35, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeUp();
            driveEncoders(pow1, cmprocessor.getEncoderAmount(23, Units.CENTIMETERS));
        }
    }

    public void maxPoints3Res(String color) {
        if (color.equalsIgnoreCase("Blue")) { /* negnin(); strafeEncoders(-pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, blockPos); driveEncoders(pow1, specBlock3); nin();
        *//*
            robot.grabber.toggleUp();
            driveEncoders(pow1, cmprocessor.getEncoderAmount(3,Units.CENTIMETERS));
            turnTo(-31*Math.PI/64);
            driveEncoders(-pow1, cmprocessor.getEncoderAmount(13, Units.CENTIMETERS));
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(64, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeDown();
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < 1000) {
            }
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(3, Units.CENTIMETERS));
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(30, Units.CENTIMETERS));
            driveEncoders(-pow1, specBlock1- cmprocessor.getEncoderAmount(43,Units.CENTIMETERS));
            robot.blockIntakeServo.intakeUp();
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(5, Units.CENTIMETERS));
           /* driveEncoders(-pow1, cmprocessor.getEncoderAmount(20, Units.CENTIMETERS));
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(10, Units.CENTIMETERS));
            turnTo(-PI);
            driveEncoders(-pow1, cmprocessor.getEncoderAmount(20, Units.CENTIMETERS));
            moveFound("Blue");
            *//*
            driveEncoders(pow1, specBlock1+cmprocessor.getEncoderAmount(25, Units.CENTIMETERS));
            //following strafe is gonna be a different number
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(15, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeDown();
            long startTime2 = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime2 < 1000) {
            }
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(15, Units.CENTIMETERS));
            driveEncoders(-pow1, specBlock1+ cmprocessor.getEncoderAmount(25, Units.CENTIMETERS));
            //negnin(); negnin(); driveEncoders(pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, blockPos); negnin(); negnin();
            *//*
            robot.blockIntakeServo.intakeUp();
            /*strafeEncoders(-pow1,cmprocessor.getEncoderAmount(10,Units.CENTIMETERS));
            driveEncoders(pow1, specBlock1+cmprocessor.getEncoderAmount(4, Units.CENTIMETERS));
            turnTo(-Math.PI/2);
            //following strafe is gonna be a different number
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(15, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeDown();
            long startTime3 = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime3 < 1000) {
            }
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(3, Units.CENTIMETERS));
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(20, Units.CENTIMETERS));
            driveEncoders(-pow1, specBlock1+ cmprocessor.getEncoderAmount(4, Units.CENTIMETERS));
            *//*
            driveEncoders(pow1, cmprocessor.getEncoderAmount(30, Units.CENTIMETERS));
        } else { /*nin(); strafeEncoders(-pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, blockPos); driveEncoders(pow1, specBlock3);
        *//*
            robot.grabber.toggleUp();
            robot.blockIntakeServo.intakeUp();
            driveEncoders(pow1, cmprocessor.getEncoderAmount(5,Units.CENTIMETERS));
            turnTo(-PI/2);
            driveEncoders(pow1, cmprocessor.getEncoderAmount(8,Units.CENTIMETERS));
            long startTime3 = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime3 < 500) {
            }
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(62, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(4, Units.CENTIMETERS));
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < 1000) {
            }
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(27, Units.CENTIMETERS));
            driveEncoders(pow1, specBlock2 - cmprocessor.getEncoderAmount(95, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, specBlock2 - cmprocessor.getEncoderAmount(41, Units.CENTIMETERS));
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(40, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(2, Units.CENTIMETERS));
            long startTime2 = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime2 < 1000) {
            }
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(40, Units.CENTIMETERS));
            turnTo(-PI/2);
            driveEncoders(pow1, specBlock2 - cmprocessor.getEncoderAmount(40, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, cmprocessor.getEncoderAmount(20, Units.CENTIMETERS));
            /*robot.grabber.toggleUp();
            robot.blockIntakeServo.intakeUp();
            driveEncoders(pow1, cmprocessor.getEncoderAmount(5,Units.CENTIMETERS));
            nin();
            driveEncoders(pow1, cmprocessor.getEncoderAmount(20, Units.CENTIMETERS));
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(66, Units.CENTIMETERS));
            turnTo(0);
            nin();
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(3, Units.CENTIMETERS));
            long startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < 1000) {
            }
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(45, Units.CENTIMETERS));
            driveEncoders(pow1, specBlock1 - cmprocessor.getEncoderAmount(60, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeUp();
            driveEncoders(pow1, cmprocessor.getEncoderAmount(55, Units.CENTIMETERS));
            turnTo(-PI);
            driveEncoders(-pow1, cmprocessor.getEncoderAmount(50, Units.CENTIMETERS));
            moveFound("Red");
            turnTo(-PI);
            //driveEncoders(pow1, specBlock3 - cmprocessor.getEncoderAmount(60, Units.CENTIMETERS));
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(60, Units.CENTIMETERS));
            driveEncoders(-pow1, cmprocessor.getEncoderAmount(20, Units.CENTIMETERS));
            strafeEncoders(pow1, cmprocessor.getEncoderAmount(60, Units.CENTIMETERS));
            driveEncoders(-pow1, cmprocessor.getEncoderAmount(20, Units.CENTIMETERS));
            turnTo(-3* PI/2);
            driveEncoders(pow1, cmprocessor.getEncoderAmount(30, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeDown();
            strafeEncoders(-pow1, cmprocessor.getEncoderAmount(3, Units.CENTIMETERS));
            long startTime2 = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime2 < 1000) {
            }
            driveEncoders(-pow1, cmprocessor.getEncoderAmount(20, Units.CENTIMETERS));
            turnTo(-2*PI);
            driveEncoders(-pow1, cmprocessor.getEncoderAmount(40, Units.CENTIMETERS));
            turnTo(-2*Math.PI -Math.PI/2);
            driveEncoders(pow1, specBlock1 - cmprocessor.getEncoderAmount(50, Units.CENTIMETERS));
            robot.blockIntakeServo.intakeUp();
            driveEncoders(-pow1, cmprocessor.getEncoderAmount(40, Units.CENTIMETERS));
            *//*
        }
    }

    public void getBlockBuild(String color) throws InterruptedException {
        if (color.equalsIgnoreCase("Blue")) {
            driveEncoders(pow1, processor.getEncoderAmount(5, Units.INCH));
            negnin();
            moveFound("Blue");
            driveEncoders(pow1, processor.getEncoderAmount(6, Units.INCH));
            nin();
            //run opencv
            if (layout1 = true) { /*negnin(); strafeEncoders(-pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, blockPos);
            *//*
                driveEncoders(pow1, oneTile);
                robot.blockIntakeServo.intakeUp();
                driveEncoders(pow1, oneTile);
                negnin();
                driveEncoders(pow1, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, specBlock1);
                nin();
                driveEncoders(pow1, oneTile);
                robot.blockIntakeServo.intakeUp();
                driveEncoders(pow1, oneTile);
                negnin();
                driveEncoders(pow1, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, processor.getEncoderAmount(5, Units.INCH)); /*nin(); nin(); strafeEncoders(-pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, blockPos); nin(); nin();
                *//*
                driveEncoders(pow1, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                driveEncoders(pow1, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, processor.getEncoderAmount(6, Units.INCH));
            } else if (layout2 = true) { /*negnin(); strafeEncoders(-pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, blockPos);
            *//*
                driveEncoders(pow1, oneTile);
                robot.blockIntakeServo.intakeUp();
                driveEncoders(pow1, oneTile);
                negnin();
                driveEncoders(pow1, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, specBlock2);
                nin();
                driveEncoders(pow1, oneTile);
                robot.blockIntakeServo.intakeUp();
                driveEncoders(pow1, oneTile);
                negnin();
                driveEncoders(pow1, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, processor.getEncoderAmount(5, Units.INCH)); /*nin(); nin(); strafeEncoders(-pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, blockPos); nin(); nin();
                *//*
                driveEncoders(pow1, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                driveEncoders(pow1, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, processor.getEncoderAmount(6, Units.INCH));
            } else if (layout3 = true) { /*negnin(); strafeEncoders(-pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, blockPos);
            *//*
                driveEncoders(pow1, oneTile);
                robot.blockIntakeServo.intakeUp();
                driveEncoders(pow1, oneTile);
                negnin();
                driveEncoders(pow1, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, specBlock3);
                nin();
                driveEncoders(pow1, oneTile);
                robot.blockIntakeServo.intakeUp();
                driveEncoders(pow1, oneTile);
                negnin();
                driveEncoders(pow1, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, processor.getEncoderAmount(5, Units.INCH)); /*nin(); nin(); strafeEncoders(-pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, blockPos); nin(); nin();
                *//*
                driveEncoders(pow1, specBlock3 + processor.getEncoderAmount(6, Units.INCH));
                driveEncoders(pow1, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, processor.getEncoderAmount(6, Units.INCH));
            }
        } else {
            driveEncoders(pow1, processor.getEncoderAmount(5, Units.INCH));
            negnin();
            moveFound("Red");
            driveEncoders(pow1, processor.getEncoderAmount(6, Units.INCH));
            negnin();
            //run opencv
            if (layout1 = true) { /*strafeEncoders(-pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, processor.getEncoderAmount(4, Units.INCH)); driveEncoders(pow1, specBlock1);
            *//*
                driveEncoders(pow1, oneTile);
                robot.blockIntakeServo.intakeUp();
                driveEncoders(pow1, oneTile);
                nin();
                driveEncoders(pow1, specBlock1);
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                negnin();
                driveEncoders(pow1, oneTile);
                robot.blockIntakeServo.intakeUp();
                driveEncoders(pow1, oneTile);
                nin();
                driveEncoders(pow1, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, processor.getEncoderAmount(5, Units.INCH)); /*negnin(); negnin(); strafeEncoders(-pow1, processor.getEncoderAmount(4, Units.INCH)); driveEncoders(pow1, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, processor.getEncoderAmount(4, Units.INCH)); negnin(); negnin();
                *//*
                driveEncoders(pow1, specBlock1 + processor.getEncoderAmount(6, Units.INCH));
                driveEncoders(pow1, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, processor.getEncoderAmount(6, Units.INCH));
            } else if (layout2 = true) { /*strafeEncoders(-pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, processor.getEncoderAmount(4, Units.INCH)); driveEncoders(pow1, specBlock1);
            *//*
                driveEncoders(pow1, oneTile);
                robot.blockIntakeServo.intakeUp();
                driveEncoders(pow1, oneTile);
                nin();
                driveEncoders(pow1, specBlock2);
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                negnin();
                driveEncoders(pow1, oneTile);
                robot.blockIntakeServo.intakeUp();
                driveEncoders(pow1, oneTile);
                nin();
                driveEncoders(pow1, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, processor.getEncoderAmount(5, Units.INCH)); /*negnin(); negnin(); strafeEncoders(-pow1, processor.getEncoderAmount(4, Units.INCH)); driveEncoders(pow1, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, processor.getEncoderAmount(4, Units.INCH)); negnin(); negnin();
                *//*
                driveEncoders(pow1, specBlock2 + processor.getEncoderAmount(6, Units.INCH));
                driveEncoders(pow1, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, processor.getEncoderAmount(6, Units.INCH));
            } else if (layout3 = true) { /*strafeEncoders(-pow1, blockPos); driveEncoders(pow1, processor.getEncoderAmount(2, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, processor.getEncoderAmount(4, Units.INCH)); driveEncoders(pow1, specBlock1);
            *//*
                driveEncoders(pow1, oneTile);
                robot.blockIntakeServo.intakeUp();
                driveEncoders(pow1, oneTile);
                nin();
                driveEncoders(pow1, specBlock3);
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, specBlock3 + processor.getEncoderAmount(3, Units.INCH));
                negnin();
                driveEncoders(pow1, oneTile);
                robot.blockIntakeServo.intakeUp();
                driveEncoders(pow1, oneTile);
                nin();
                driveEncoders(pow1, specBlock3 + processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, processor.getEncoderAmount(5, Units.INCH)); /*negnin(); negnin(); strafeEncoders(-pow1, processor.getEncoderAmount(4, Units.INCH)); driveEncoders(pow1, processor.getEncoderAmount(3, Units.INCH)); robot.blockIntakeServo.intakeUp(); strafeEncoders(-pow1, processor.getEncoderAmount(4, Units.INCH)); negnin(); negnin();
                *//*
                driveEncoders(pow1, specBlock3 + processor.getEncoderAmount(3, Units.INCH));
                driveEncoders(pow1, processor.getEncoderAmount(3, Units.INCH));
                robot.blockIntakeServo.intakeDown();
                driveEncoders(pow1, processor.getEncoderAmount(6, Units.INCH));
            }
        }
    }

    public String radToDeg(double rads) {
        DecimalFormat df = new DecimalFormat("###.###");
        return df.format(rads*180/Math.PI);
    }

    public void turnTo(double angle) {
        robot.mDrive.turnTo((mod(startAngle+angle-Math.PI/*-0.0349066, 2*Math.PI))+Math.PI);
        *//*

    }
    public void turnToFromAngle(double angle) {
        double ending = (mod(angle+robot.mDrive.getCurrentAngle(AngleUnit.RADIANS)-Math.PI, 2*Math.PI))+Math.PI;
        telemetry.addLine("Angle: " + radToDeg(angle) + " Ending: " + radToDeg(ending) + " " + radToDeg(angle - ending));
        telemetry.addLine(radToDeg(startAngle));
        telemetry.update();
        turnTo(ending);

    }

    Vector straight = new Vector(0, 1);
    Vector backwards = new Vector(0, -1);
    Vector left = new Vector(-1, 0);
    Vector right = new Vector(1, 0);
    int specBlock1 = cmprocessor.getEncoderAmount(130, Units.CENTIMETERS);
    int specBlock2 = cmprocessor.getEncoderAmount(200, Units.CENTIMETERS);
    int specBlock3 = cmprocessor.getEncoderAmount(210, Units.CENTIMETERS);
    boolean layout1 = true;
    boolean layout2 = false;
    boolean layout3 = false;
    boolean once = true;
    int i = 0;
    double pow1 = -1;
    double pow2 = 0.5;
    DcMotor topLeft;
    DcMotor topRight;
    DcMotor botRight;
    DcMotor botLeft;
    long startTime = System.currentTimeMillis();
    double startAngle;
*/
    @Override
    public void main() {
        /*
        if(once){
            robot.markerServo.MarkerServo.setPosition(.5);
            startAngle = robot.mDrive.getCurrentAngle();
            once = false;
        }
        telemetry.setAutoClear(true);
        String position;
        switch (robot.selector.autonomous) {  // new btd6 hero is trash
            case ("Forward23in"):
                //moveFound("Red");
                //robot.mDrive.driveEncoders(new Vector(-1,0), 4000, true);
                strafeEncoders(pow1,cmprocessor.getEncoderAmount(10,Units.CENTIMETERS));
                strafeEncoders(-pow1,cmprocessor.getEncoderAmount(10,Units.CENTIMETERS));
                strafeEncoders(pow1,cmprocessor.getEncoderAmount(10,Units.CENTIMETERS));
                strafeEncoders(-pow1,cmprocessor.getEncoderAmount(10,Units.CENTIMETERS));
                strafeEncoders(pow1,cmprocessor.getEncoderAmount(10,Units.CENTIMETERS));
                strafeEncoders(-pow1,cmprocessor.getEncoderAmount(10,Units.CENTIMETERS));
                strafeEncoders(pow1,cmprocessor.getEncoderAmount(10,Units.CENTIMETERS));
                strafeEncoders(-pow1,cmprocessor.getEncoderAmount(10,Units.CENTIMETERS));
                strafeEncoders(pow1,cmprocessor.getEncoderAmount(10,Units.CENTIMETERS));
                strafeEncoders(-pow1,cmprocessor.getEncoderAmount(10,Units.CENTIMETERS));
                break;
            case ("Turn90"):
                telemetry.addData("", robot.mDrive.getCurrentAngle(AngleUnit.DEGREES));
                telemetry.update();
                turnTo(Math.PI/2);
                turnTo(0);
                turnTo(Math.PI);
                turnTo(5*Math.PI/4);
                break;
            case ("OpenCV"):
                position = robot.openCV.check();
                break;

            case ("ParkOnBridge"):
                if (robot.selector.color.equals("Red")) {
                    driveEncoders(pow1, oneTile +  850);
                } else if (robot.selector.color.equals("Blue")) {
                    driveEncoders(pow1, oneTile + 850);
                }
                break;
            case ("MoveFoundationPark"):
                    if (robot.selector.color.equals("Red")) {
                        robot.grabber.toggleUp();
                        strafeEncoders(-pow1, oneTile - 500);
                        driveEncoders(-pow1-.3, twoTile -300);
                        moveFound("Red");
                       // driveEncoders(-pow1, processor.getEncoderAmount(57, Units.INCH) );
                        strafeEncoders(pow1, twoTile +3700);


                    }  else if (robot.selector.color.equals("Blue")) {
                        robot.grabber.toggleUp();
                        strafeEncoders(pow1, oneTile - 500);
                        driveEncoders(-pow1-0.3, twoTile - 300);
                        moveFound("Blue");
                        //driveEncoders(-pow1, processor.getEncoderAmount(57, Units.INCH));
                        strafeEncoders(-pow1, twoTile + 3700);


                    }

                break;
            case("getBlockPark"):
                //todo if (robot.openCV.layout.equals(idklol)"
                if(robot.selector.color.equals("Blue")){
                    getBlock1res("Blue");
                    break;
                }
                else{
                    getBlock1res("Red");
                    break;
                 }
            case("intakeTest"):
                robot.blockIntakeServo.intakeDown();

            case ("MaxPoints"):
                if(robot.selector.startPos.equals("Resource")) {
                   String asd = robot.openCV.check();
                   telemetry.addLine(asd);
                   telemetry.update();
                    long ree = System.currentTimeMillis();
                    while (System.currentTimeMillis() - ree <2000) {}
                   if(asd.equals("Right"))
                        if (robot.selector.color.equals("Blue")) {
                            maxPoints1Res("Blue");
                            telemetry.update();
                            break;
                        } else {
                            maxPoints3Res("Red");
                            telemetry.update();
                            break;
                        }
                   else if(asd.equals("Middle")) {
                       if (robot.selector.color.equals("Blue")) {
                           maxPoints2Res("Blue");
                           telemetry.update();
                           break;
                       } else {
                           maxPoints2Res("Red");
                           telemetry.update();
                           break;
                       }
                   }
                   else if(asd.equals("Left")){
                        if (robot.selector.color.equals("Blue")) {
                            maxPoints3Res("Blue");
                            telemetry.update();
                            break;
                        } else {
                            maxPoints1Res("Red");
                            telemetry.update();
                            break;
                        }
                    }
                    else {
                        telemetry.addData("shits displaced ", i);
                        telemetry.update();
                                while (true){

                                }
                    }
                /*if (robot.skystoneDeteector.layout == true) {    todo else if (robot.openCV.layout.equals("maxPoints2Res")) {} if (robot.selector.color.equals("Blue")) { maxPoints2Res("Blue"); telemetry.update(); break; } else { maxPoints2Res("Red"); telemetry.update(); break; } } if (layout3 == true) {      todo else if (robot.openCV.layout.equals("gerBlock3Res")) {} if (robot.selector.color.equals("Blue")) { maxPoints3Res("Blue"); telemetry.update(); break; } else { maxPoints3Res("Red"); telemetry.update(); break; } } } else if (robot.selector.startPos.equals("Build")) { if (robot.selector.color.equals("Blue")) { run the generic start to the build autonomous, move the foundation and position in front of the blocks positioning to run opencv getBlockBuild("Blue"); telemetry.update(); break; } else { getBlockBuild("Red"); telemetry.update(); break; }
                */
    }/*
                }
        }
    }*/
}