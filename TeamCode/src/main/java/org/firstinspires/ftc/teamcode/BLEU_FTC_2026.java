/*
Copyright 2026 FIRST Tech Challenge Team FGC

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains a minimal example of a Linear "OpMode". An OpMode is a 'program' that runs
 * in either the autonomous or the TeleOp period of an FTC match. The names of OpModes appear on
 * the menu of the FTC Driver Station. When an selection is made from the menu, the corresponding
 * OpMode class is instantiated on the Robot Controller and executed.
 *
 * Remove the @Disabled annotation on the next line or two (if present) to add this OpMode to the
 * Driver Station OpMode list, or add a @Disabled annotation to prevent this OpMode from being
 * added to the Driver Station.
 */
@Autonomous

public class BLEU_FTC_2026 extends LinearOpMode {

    private IMU imu = null; 
    private DcMotorEx arriereGauche = null;
    private DcMotorEx arriereDroit = null;
    private DcMotorEx avantGauche = null;
    private DcMotorEx avantDroit = null;
    private int VITESSE_MAX_MOTEUR = 2800; //600; // tick par seconde
    private int VELOCITY_MAX_HEX = 600;
    private DcMotorEx roueLanceur = null;
    private DcMotorEx moissoneuse = null;
    private NormalizedRGBA colors;
    private NormalizedColorSensor colorSensor = null;
    private Servo trieurColor = null;
    private Servo servoViolet = null;
    private Servo servoVert = null;
    private Servo servoPelle = null;
    
    private VisionPortal.Builder myVisionPortalBuilder;
    private AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    private AprilTagProcessor myAprilTagProcessor;
    private AprilTagDetection myAprilTagDetection;
    private ArrayList<AprilTagDetection> myAprilTagDetections;
    private VisionPortal myVisionPortal;
    
    private double hue = 0.0;
    private double posPelle = 0.38;
    private double posViolet = 0.2;
    private double BASPELLE = 0.56;
    private double HAUTPELLE = 0.38;
    private double BAS_VIOLET = 0.4;
    private double HAUT_VIOLET = 0.5;
    private double BAS_VERT = 0.56;
    private double HAUT_VERT = 0.5;
    private double POS_INIT_TRI = 0.47;
    private double posVert = 1;
    private double x = 0.0;
    private double y = 0.0;
    private double z = 0.0;
    private boolean tagFound = false;
    double vitesseRoueAVD;
    double vitesseRoueAVG;
    double vitesseRoueARG;
    double vitesseRoueARD;
    
    
    final double SPEED_GAIN  =  0.01  ;   //  Forward Speed Control "Gain". e.g. Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  e.g. Ramp up to 37% power at a 25 degree Yaw error.   (0.375 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  e.g. Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= 0.3;   //  Clip the strafing speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)
    private boolean x_OK, y_OK, z_OK;
    
    private ArrayList<String> ordre = new ArrayList<String>();
    
    private enum State {
        DEPL1,
        DETECT,
        DEPL2,
        MANGEBALLES,
        SUITE,
        DEPL3,
        
        YAW,
        DIST,
        CENTR,
        TIR,
        FIN,
        GOTO_QR,
        SCAN,
        GOTO_TIR0,
        SE_PLACER0,
        TIR_BALLES,
        MANGERBALLES1,
        RANGEE1,
        GOTO_TIR1,
        SE_PLACER1,
        TIRER1,
        RANGEE2,
        MANGERBALLES2,
        GOTO_TIR2,
        SE_PLACER2,
        TIRER2,
        RANGEE3,
        MANGERBALLES3,
        GOTO_TIR3,
        SE_PLACER3,
        TIRER3,
        IMUHELP,
        BEARING,
        RANGE,
        
        SCAN_ORDRE,
        SE_PLACER,
    }
    
    private State etape = State.SCAN_ORDRE;
    private ElapsedTime timer = new ElapsedTime();
    double temps;
    
    private State etape_intern = State.YAW;
    
    public int calculDuree(double distance){
        return (int) ((distance+8.471)/48.429*1000);
    }
    
    public void translation(String direction, double distance){
        int duree =  calculDuree(distance);
        double x,y;
        if (direction == "AVANT"){
            x = 0;
            y = 1;
        }
        else if(direction == "ARRIERE") {
            x = 0;
            y = -1;
        }
        else if(direction == "GAUCHE"){
            x = -1;
            y = 0;
        }
        else if(direction == "DROITE"){
            x = 1;
            y = 0;
        }
        else {
            return;
        }
        translation(x,y,0.25);
        sleep(duree);
        stopMoving();
        sleep(200);

    }
    
    public void translation(double x,double y,double rTrigger){
        
            double vitesseRoueARG_avd = x* -Math.sqrt(2)/2 + y* Math.sqrt(2)/2 ;
            double vitesseRoueARD_avg = x* Math.sqrt(2)/2 + y* Math.sqrt(2)/2 ;
            double vitesseMax;
            double vitesseRoueAVD = vitesseRoueARG_avd;
            double vitesseRoueAVG = vitesseRoueARD_avg;
            double vitesseRoueARG = vitesseRoueARG_avd;
            double vitesseRoueARD = vitesseRoueARD_avg;
            double maximum = Math.max(Math.max(Math.abs(vitesseRoueAVD),Math.abs(vitesseRoueARD)),Math.max(Math.abs(vitesseRoueAVG),Math.abs(vitesseRoueARG)));
        
            
            if (rTrigger == 0){
                rTrigger = 0.25;
            }
            
            double ampl_Rtrigger = rTrigger * VITESSE_MAX_MOTEUR;
            vitesseRoueAVD = vitesseRoueAVD * ampl_Rtrigger / maximum;
            vitesseRoueAVG = vitesseRoueAVG * ampl_Rtrigger / maximum;
            vitesseRoueARD = vitesseRoueARD * ampl_Rtrigger / maximum;
            vitesseRoueARG = vitesseRoueARG * ampl_Rtrigger / maximum;
            
            avantDroit.setVelocity(vitesseRoueAVD);
            arriereDroit.setVelocity(vitesseRoueARD);
            avantGauche.setVelocity(vitesseRoueAVG);
            arriereGauche.setVelocity(vitesseRoueARG);
    }
    
    public void rotation(String sens , double angle){
        imu.resetYaw();
        double theta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        double objectif, vitesseG, vitesseD;
        if (sens == "DROITE"){
            objectif = theta - angle;
            vitesseG = 0.25 * VITESSE_MAX_MOTEUR;
            vitesseD = -0.25 * VITESSE_MAX_MOTEUR;
        }
        else if (sens == "GAUCHE"){
            objectif = theta + angle;
            vitesseG = -0.25 * VITESSE_MAX_MOTEUR;
            vitesseD = 0.25 * VITESSE_MAX_MOTEUR;
        }
        else {
            return;
        }
        
        while((theta<objectif-5) || (theta>objectif+5)){
            theta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            avantDroit.setVelocity(vitesseD);
            arriereDroit.setVelocity(vitesseD);
            avantGauche.setVelocity(vitesseG);
            arriereGauche.setVelocity(vitesseG);
        }
        stopMoving();
    }
    
    
    public void lancerBalles(){
        servoPelle.setPosition(HAUTPELLE);
    }
    
    public void chargerG(){
        servoPelle.setPosition(BASPELLE);
        servoVert.setPosition(HAUT_VERT);
        sleep(1234);
        servoVert.setPosition(BAS_VERT);
    }
    
    public void chargerP(){
        servoViolet.setPosition(HAUT_VIOLET);
        sleep(987);
        servoPelle.setPosition(BASPELLE);
        sleep(250);
        servoViolet.setPosition(BAS_VIOLET);
    }
    
    
    public void initializeVisionPortal(){
        Position cameraPosition = new Position(DistanceUnit.CM,
                0, 19.5, 43.5, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, -73, 0, 0);


        myVisionPortalBuilder = new VisionPortal.Builder();
        myVisionPortalBuilder.setCamera((hardwareMap.get(WebcamName.class, "webcam")));
        myVisionPortalBuilder.enableLiveView(true);
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation);
        myAprilTagProcessor = (myAprilTagProcessorBuilder.build());
        myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        myVisionPortal = (myVisionPortalBuilder.build());
        
    }
    
    public void displayVisionPortalData(){
        myAprilTagDetections = (myAprilTagProcessor.getDetections());
        for (AprilTagDetection myAprilTagDetection2 : myAprilTagDetections) {
            myAprilTagDetection = myAprilTagDetection2;
            telemetry.addData("ID", (myAprilTagDetection.id));
            telemetry.addData("Range", (myAprilTagDetection.ftcPose.range*2.54));
            telemetry.addData("Yaw", (myAprilTagDetection.ftcPose.yaw));
            telemetry.addData("Bearing", (myAprilTagDetection.ftcPose.bearing));
        }
        telemetry.update();
    }
    
    public void tir(ArrayList<String> ordre){
        roueLanceur.setPower(-1); 
        sleep(2000);
        for(String car : ordre){
            if(car == "G"){
                chargerG();
            }
            else if (car == "P"){
                chargerP();
            }
            sleep(1000);
            lancerBalles();
            sleep(1000);
        }
        roueLanceur.setPower(0);
    }
    
     public void moveRobot(double x, double y, double z) {
        // Calculate wheel powers.
        double vitesseRoueAVG  =  x + y + z;
        double vitesseRoueAVD  =  -x + y - z;
        double vitesseRoueARG  =  -x + y - z;
        double vitesseRoueARD  =  x + y - z;
        double max = Math.max(Math.abs(vitesseRoueAVG), Math.abs(vitesseRoueAVD));
        max = Math.max(max, Math.abs(vitesseRoueARG));
        max = Math.max(max, Math.abs(vitesseRoueARD));

        if (max > 1.0) {
            vitesseRoueAVG /= max;
            vitesseRoueAVD /= max;
            vitesseRoueARG /= max;
            vitesseRoueARD /= max;
        }
        
        avantGauche.setVelocity(vitesseRoueAVG*VITESSE_MAX_MOTEUR);
        avantDroit.setVelocity(vitesseRoueAVD*VITESSE_MAX_MOTEUR);
        arriereGauche.setVelocity(vitesseRoueARG*VITESSE_MAX_MOTEUR);
        arriereDroit.setVelocity(vitesseRoueARD*VITESSE_MAX_MOTEUR);
        telemetry.addData("velocity AVG",vitesseRoueAVG*VITESSE_MAX_MOTEUR);
        telemetry.addData("velocity AVD",vitesseRoueAVD*VITESSE_MAX_MOTEUR);
        telemetry.addData("velocity ARG",vitesseRoueARG*VITESSE_MAX_MOTEUR);
        telemetry.addData("velocity ARD",vitesseRoueARD*VITESSE_MAX_MOTEUR);

     }
    
    
    
    
    
    public void stopMoving(){
        
            avantDroit.setVelocity(0);
            arriereDroit.setVelocity(0);
            avantGauche.setVelocity(0);
            arriereGauche.setVelocity(0);
        
    }


    @Override
    public void runOpMode() {
        
        initializeVisionPortal();

        telemetry.addData("Status nouveau", "Initialized");
        telemetry.update();
        
        avantDroit  = hardwareMap.get(DcMotorEx.class, "avantdroit");
        avantGauche  = hardwareMap.get(DcMotorEx.class, "avantgauche");
        arriereDroit = hardwareMap.get(DcMotorEx.class, "arrieredroit");
        arriereGauche = hardwareMap.get(DcMotorEx.class, "arrieregauche");
        roueLanceur =  hardwareMap.get(DcMotorEx.class, "motorlanceur");
        moissoneuse = hardwareMap.get(DcMotorEx.class, "moissoneuse");
        trieurColor = hardwareMap.get(Servo.class, "ejecttrieur");
        servoViolet = hardwareMap.get(Servo.class, "servoviolet");
        servoVert = hardwareMap.get(Servo.class, "servovert");
        servoPelle = hardwareMap.get(Servo.class, "servopelle");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        imu = hardwareMap.get(IMU.class, "imu");
        
        avantDroit.setDirection(DcMotorEx.Direction.REVERSE);
        arriereDroit.setDirection(DcMotorEx.Direction.REVERSE);
        
        avantDroit.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        avantDroit.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        
        avantGauche.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        avantGauche.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        
        arriereDroit.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arriereDroit.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        
        arriereGauche.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        arriereGauche.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        
        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo,usb)));
        imu.resetYaw();
        
        // Wait for the game to start (driver presses PLAY)
        servoVert.setPosition(BAS_VERT);
        servoViolet.setPosition(BAS_VIOLET);
        servoPelle.setPosition(HAUTPELLE);
        int duree = 0;


        waitForStart();
 
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double vitesseRoueAVG, vitesseRoueAVD, vitesseRoueARG, vitesseRoueARD;

            myAprilTagDetections = (myAprilTagProcessor.getDetections());
            int nbNotFound = 0;

            // etape = State.FIN;

            switch(etape){
                case SCAN_ORDRE:
                    myAprilTagDetections = (myAprilTagProcessor.getDetections());
                    for (AprilTagDetection myAprilTagDetection2 : myAprilTagDetections) {
                        myAprilTagDetection = myAprilTagDetection2;
                          if (myAprilTagDetection.id == 21){
                            ordre.add("G");
                            ordre.add("P");
                            ordre.add("P");
                            telemetry.addData("ordre","GPP");
                          }
                          else if (myAprilTagDetection.id == 22){
                            ordre.add("P");
                            ordre.add("G");
                            ordre.add("P");
                            telemetry.addData("ordre","PGP");
                          }
                          else if (myAprilTagDetection.id == 23){
                            ordre.add("P");
                            ordre.add("P");
                            ordre.add("G");
                            telemetry.addData("ordre","PPG");
                          }
                          else {
                              telemetry.addData("Ordre","Couleurs non detectées");
                          }
                  }
                  if (!ordre.isEmpty()){
                        etape = State.SE_PLACER;

                    }
                  telemetry.update();
                  break;

                case SE_PLACER:
                    int nbTags = 0;
                    for (AprilTagDetection myAprilTag : myAprilTagDetections){
                        if (myAprilTag.id == 20) {
                            double range = myAprilTag.ftcPose.range*2.54;
                            double bearing = myAprilTag.ftcPose.bearing;
                            double yaw = myAprilTag.ftcPose.yaw;
                            x = yaw;
                            y = range - 62;
                            z = bearing;
                            tagFound = true;
                            nbTags+=1;
                            //break;
                        }
                    }
                    if (nbTags == 0){tagFound = false;}
                    if (tagFound) {
                        x_OK = x > -5 && x < 5;
                        y_OK = y > -5 && y < 5;
                        z_OK = z > -3 && z < 3;
                        if (x_OK && y_OK && z_OK) {
                            stopMoving();
                            etape = State.TIR;
                            telemetry.addData("bleu", "Goto TIR");
                        } else {
                            telemetry.addData("range-62",y);
                            telemetry.addData("yaw",x);
                            telemetry.addData("bearing",z);
                            if (x_OK){
                                x=0;
                            }
                            else {
                                x = Range.clip(x * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                            }
                            if (y_OK){y = 0;}
                            else{y = Range.clip(y * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);}
                            if(z_OK){z=0;}
                            else{z = Range.clip(-z * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);}
                            moveRobot(x, y, z);
                        }

                    }
                    else {
                        telemetry.addLine("Recherche du Tag ...");
                        //stopMoving();
                        nbNotFound +=1;
                        moveRobot(x*0.7,y*0.7,z*0.7);
                    }
                    telemetry.update();
                    break;

                case TIR:
                    tir(ordre);
                    etape = State.FIN;
                    break;

                case FIN:
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f",x,y,z));
                    telemetry.addData("nb not found",nbNotFound);
                    displayVisionPortalData();
                    break;
            }
            
           
            
            
//             switch(etape){
//                 case GOTO_QR:
//                     translation("AVANT",100);
//                     translation("DROITE",40);
//                     etape = State.SCAN;
//                     break;
//
//                 case SCAN:
//                     myAprilTagDetections = (myAprilTagProcessor.getDetections());
//                     for (AprilTagDetection myAprilTagDetection2 : myAprilTagDetections) {
//                         myAprilTagDetection = myAprilTagDetection2;
//                           if (myAprilTagDetection.id == 21){
//                             ordre.add("G");
//                             ordre.add("P");
//                             ordre.add("P");
//                           }
//                           else if (myAprilTagDetection.id == 22){
//                             ordre.add("P");
//                             ordre.add("G");
//                             ordre.add("P");
//                           }
//                           else if (myAprilTagDetection.id == 23){
//                             ordre.add("P");
//                             ordre.add("P");
//                             ordre.add("G");
//                           }
//                           else {
//                               telemetry.addData("Ordre","Couleurs non detectées");
//                           }
//                   }
//                   if (!ordre.isEmpty()){
//                         etape = State.GOTO_TIR0;
//                     }
//
//                   break;
//
//                 case GOTO_TIR0:
//                     translation("AVANT",135);
//                     translation("GAUCHE",50);
//                     rotation("GAUCHE",45);
//                     translation("AVANT",60);
//                     // translation("GAUCHE",40);
//                     etape = State.TIR_BALLES;
//                     break;
//
//                 case SE_PLACER0:
//                     etape_intern = State.YAW;
//                     displayVisionPortalData();
//                     switch(etape_intern){
//                         case YAW:
//                             double yaw = 0.0;
//                             myAprilTagDetections = (myAprilTagProcessor.getDetections());
//                             for (AprilTagDetection myAprilTag : myAprilTagDetections){
//                                 if (myAprilTag.id == 20) {// BLEU
//                                     yaw = myAprilTag.ftcPose.yaw;
//                                 }
//                             }
//                             telemetry.addData("yaw",yaw);
//                             if(Math.abs(yaw)<5){
//                                 telemetry.addData("yaw","GOTO CENTR");
//                                 stopMoving();
//                                 etape_intern = State.CENTR;
//
//                             }
//                             else{
//                                 if(yaw > 0){ //GAUCHE
//                                     vitG =  -0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = 0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","gauche");
//                                 }
//                                 else{
//                                     vitG = 0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = -0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","droite");
//                                 }
//                                 avantDroit.setVelocity(vitD);
//                                 arriereDroit.setVelocity(vitD);
//                                 avantGauche.setVelocity(vitG);
//                                 arriereGauche.setVelocity(vitG);
//                             }
//                             // rotation(,);
//                             telemetry.update();
//                             // sleep(700);
//                             break;
//
//                         case DIST:
//                              double range = 0.0;
//
//                             for (AprilTagDetection myAprilTag : myAprilTagDetections){
//                                 if (myAprilTag.id == 20) {// range
//                                     range = myAprilTag.ftcPose.range*2.54-62;
//                                 }
//                                 }
//                             telemetry.addData("range",range);
//                             if(range >-5 & range <5){
//                                 stopMoving();
//                                 etape = State.TIR_BALLES;
//                                 telemetry.addData("range","GOTO TIR");
//                             }
//                             else{
//                                 if(range>0){   //positif
//                                     vitG = 0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = 0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","avant");
//                                 }
//                                 else{            //negatif
//                                     vitG = -0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = -0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","arriere");
//                                 }
//                                 avantDroit.setVelocity(vitG);
//                                 arriereDroit.setVelocity(vitD);
//                                 avantGauche.setVelocity(vitG);
//                                 arriereGauche.setVelocity(vitD);
//                             }
//                             telemetry.update();
//                             // sleep(700);
//                             break;
//                         case CENTR:
//                             double bearing = 0.0;
//                             for (AprilTagDetection myAprilTag : myAprilTagDetections){
//                                 if (myAprilTag.id == 20) {// bearing
//                                     bearing = myAprilTag.ftcPose.bearing;
//                                 }
//                             }
//                             telemetry.addData("bearing",bearing);
//                             if(bearing>-5& bearing<5){
//                                 // vitesseG = 0;
//                                 // vitesseD =  0;
//                                 stopMoving();
//                                 etape_intern = State.DIST;
//                                 telemetry.addData("bearing","GOTO DIST");
//                             }
//                             else{
//                                 if(bearing>0){   //positif
//                                     vitG = 0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = -0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","gauche");
//                                 }
//                                 else{            //negatif
//                                     vitG = -0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = 0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","droite");
//                                 }
//                                 avantDroit.setVelocity(vitG);
//                                 arriereDroit.setVelocity(vitD);
//                                 avantGauche.setVelocity(vitD);
//                                 arriereGauche.setVelocity(vitG);
//                             }
//                             telemetry.update();
//                             // sleep(700);
//                             break;
//                     }
//
//
//                 case TIR_BALLES:
//                     tir(ordre);
//                     rotation("GAUCHE",45);
//                     translation("ARRIERE",60);
//                     etape = State.IMUHELP;
//                     break;
//
//                 case RANGEE1:
//                     rotation("GAUCHE",45);
//                     translation("ARRIERE",60);
//                     translation("GAUCHE",60);
//                     etape = State.MANGERBALLES1;
//                     timer.reset();
//                     moissoneuse.setPower(-1);
//
//                     vitG = 0.25*VITESSE_MAX_MOTEUR;
//                     vitD = 0.25*VITESSE_MAX_MOTEUR;
//                     avantDroit.setVelocity(vitG);
//                     arriereDroit.setVelocity(vitD);
//                     avantGauche.setVelocity(vitG);
//                     arriereGauche.setVelocity(vitD);
//
//                     break;
//
//                 case MANGERBALLES1:
//                 temps = timer.milliseconds();
//                     telemetry.addData("duree",duree);
//                     telemetry.addData("timer",temps);
//                     if (temps > duree){
//                          telemetry.addData("FIN","FIN");
//                         stopMoving();
//                         if (temps > duree + 2000){
//                             moissoneuse.setPower(0);
//                             trieurColor.setPosition(POS_INIT_TRI);
//                             etape = State.GOTO_TIR1;
//                           telemetry.update();
//                             break;
//
//                         }
//                         telemetry.update();
//                     }
//                     colors = colorSensor.getNormalizedColors();
//                     hue = JavaUtil.colorToHue(colors.toColor());
//
//                     if (100<=hue && hue<200){//Vertes
//                         trieurColor.setPosition(1);
//                     }
//                     else if (200<=hue){//Violettes
//                         trieurColor.setPosition(0);
//                     }
//                     else if (hue < 5) { trieurColor.setPosition(POS_INIT_TRI);}
//                     telemetry.update();
//                     break;
//
//                 case GOTO_TIR1:
//                     translation("ARRIERE",60);
//                     translation("DROITE",60);
//                     rotation("DROITE",45);
//                     etape = State.SE_PLACER1;
//                     break;
//
//                 case SE_PLACER1:
//                     etape_intern = State.YAW;
//                     switch(etape_intern){
//                         case YAW:
//                             double yaw = 0.0;
//                             myAprilTagDetections = (myAprilTagProcessor.getDetections());
//                             for (AprilTagDetection myAprilTag : myAprilTagDetections){
//                                 if (myAprilTag.id == 20) {// BLEU
//                                     yaw = myAprilTag.ftcPose.yaw;
//                                 }
//                             }
//                             telemetry.addData("yaw",yaw);
//                             if(Math.abs(yaw)<5){
//                                 telemetry.addData("yaw","GOTO CENTR");
//                                 stopMoving();
//                                 etape_intern = State.CENTR;
//
//                             }
//                             else{
//                                 if(yaw > 0){ //GAUCHE
//                                     vitG =  -0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = 0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","gauche");
//                                 }
//                                 else{
//                                     vitG = 0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = -0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","droite");
//                                 }
//                                 avantDroit.setVelocity(vitD);
//                                 arriereDroit.setVelocity(vitD);
//                                 avantGauche.setVelocity(vitG);
//                                 arriereGauche.setVelocity(vitG);
//                             }
//                             // rotation(,);
//                             telemetry.update();
//                             // sleep(700);
//                             break;
//
//                         case DIST:
//                              double range = 0.0;
//
//                             for (AprilTagDetection myAprilTag : myAprilTagDetections){
//                                 if (myAprilTag.id == 20) {// range
//                                     range = myAprilTag.ftcPose.range*2.54-62;
//                                 }
//                                 }
//                             telemetry.addData("range",range);
//                             if(range >-5 && range <5){
//                                 stopMoving();
//                                 etape = State.TIRER1;
//                                 telemetry.addData("range","GOTO TIR");
//                             }
//                             else{
//                                 if(range>0){   //positif
//                                     vitG = 0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = 0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","avant");
//                                 }
//                                 else{            //negatif
//                                     vitG = -0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = -0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","arriere");
//                                 }
//                                 avantDroit.setVelocity(vitG);
//                                 arriereDroit.setVelocity(vitD);
//                                 avantGauche.setVelocity(vitG);
//                                 arriereGauche.setVelocity(vitD);
//                             }
//                             telemetry.update();
//                             // sleep(700);
//                             break;
//                         case CENTR:
//                             double bearing = 0.0;
//                             for (AprilTagDetection myAprilTag : myAprilTagDetections){
//                                 if (myAprilTag.id == 20) {// bearing
//                                     bearing = myAprilTag.ftcPose.bearing;
//                                 }
//                             }
//                             telemetry.addData("bearing",bearing);
//                             if(bearing>-5 & bearing<5){
//                                 // vitesseG = 0;
//                                 // vitesseD =  0;
//                                 stopMoving();
//                                 etape_intern = State.DIST;
//                                 telemetry.addData("bearing","GOTO DIST");
//                             }
//                             else{
//                                 if(bearing>0){   //positif
//                                     vitG = 0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = -0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","gauche");
//                                 }
//                                 else{            //negatif
//                                     vitG = -0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = 0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","droite");
//                                 }
//                                 avantDroit.setVelocity(vitG);
//                                 arriereDroit.setVelocity(vitD);
//                                 avantGauche.setVelocity(vitD);
//                                 arriereGauche.setVelocity(vitG);
//                             }
//                             telemetry.update();
//                             // sleep(700);
//                             break;
//                     }
//
//
//
//                 case TIRER1:
//                     tir(ordre);
//                     rotation("GAUCHE",45);
//                     etape = State.RANGEE2;
//                     break;
//
//                 case RANGEE2:
//                     translation("ARRIERE",60);
//                     translation("GAUCHE",150);
//                     etape = State.MANGERBALLES2;
//                     timer.reset();
//                     moissoneuse.setPower(-1);
//
//                     vitG = 0.25*VITESSE_MAX_MOTEUR;
//                     vitD = 0.25*VITESSE_MAX_MOTEUR;
//                     avantDroit.setVelocity(vitG);
//                     arriereDroit.setVelocity(vitD);
//                     avantGauche.setVelocity(vitG);
//                     arriereGauche.setVelocity(vitD);
//                     break;
//
//                 case MANGERBALLES2:
//                     temps = timer.milliseconds();
//                     telemetry.addData("duree",duree);
//                     telemetry.addData("timer",temps);
//                     if (temps > duree){
//                          telemetry.addData("FIN","FIN");
//                         stopMoving();
//                         if (temps > duree + 2000){
//                             moissoneuse.setPower(0);
//                             trieurColor.setPosition(POS_INIT_TRI);
//                             etape = State.GOTO_TIR2;
//                           telemetry.update();
//                             break;
//                         }
//                         telemetry.update();
//                     }
//                     colors = colorSensor.getNormalizedColors();
//                     hue = JavaUtil.colorToHue(colors.toColor());
//
//                     if (100<=hue && hue<200){//Vertes
//                         trieurColor.setPosition(1);
//                     }
//                     else if (200<=hue){//Violettes
//                         trieurColor.setPosition(0);
//                     }
//                     else if (hue < 5) { trieurColor.setPosition(POS_INIT_TRI);}
//                     telemetry.update();
//                     break;
//
//                 case GOTO_TIR2:
//                     translation("ARRIERE",60);
//                     translation("DROITE",90);
//                     rotation("DROITE",45);
//                     etape = State.SE_PLACER2;
//                     break;
//
//                 case SE_PLACER2:
//                     etape_intern = State.YAW;
//                     switch(etape_intern){
//                         case YAW:
//                             double yaw = 0.0;
//                             myAprilTagDetections = (myAprilTagProcessor.getDetections());
//                             for (AprilTagDetection myAprilTag : myAprilTagDetections){
//                                 if (myAprilTag.id == 20) {// BLEU
//                                     yaw = myAprilTag.ftcPose.yaw;
//                                 }
//                             }
//                             telemetry.addData("yaw",yaw);
//                             if(Math.abs(yaw)<5){
//                                 telemetry.addData("yaw","GOTO CENTR");
//                                 stopMoving();
//                                 etape_intern = State.CENTR;
//
//                             }
//                             else{
//                                 if(yaw > 0){ //GAUCHE
//                                     vitG =  -0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = 0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","gauche");
//                                 }
//                                 else{
//                                     vitG = 0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = -0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","droite");
//                                 }
//                                 avantDroit.setVelocity(vitD);
//                                 arriereDroit.setVelocity(vitD);
//                                 avantGauche.setVelocity(vitG);
//                                 arriereGauche.setVelocity(vitG);
//                             }
//                             // rotation(,);
//                             telemetry.update();
//                             // sleep(700);
//                             break;
//
//                         case DIST:
//                              double range = 0.0;
//
//                             for (AprilTagDetection myAprilTag : myAprilTagDetections){
//                                 if (myAprilTag.id == 20) {// range
//                                     range = myAprilTag.ftcPose.range*2.54-62;
//                                 }
//                                 }
//                             telemetry.addData("range",range);
//                             if(range >-5 & range <5){
//                                 stopMoving();
//                                 etape = State.TIRER2;
//                                 telemetry.addData("range","GOTO TIR");
//                             }
//                             else{
//                                 if(range>0){   //positif
//                                     vitG = 0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = 0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","avant");
//                                 }
//                                 else{            //negatif
//                                     vitG = -0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = -0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","arriere");
//                                 }
//                                 avantDroit.setVelocity(vitG);
//                                 arriereDroit.setVelocity(vitD);
//                                 avantGauche.setVelocity(vitG);
//                                 arriereGauche.setVelocity(vitD);
//                             }
//                             telemetry.update();
//                             // sleep(700);
//                             break;
//                         case CENTR:
//                             double bearing = 0.0;
//                             for (AprilTagDetection myAprilTag : myAprilTagDetections){
//                                 if (myAprilTag.id == 20) {// bearing
//                                     bearing = myAprilTag.ftcPose.bearing;
//                                 }
//                             }
//                             telemetry.addData("bearing",bearing);
//                             if(bearing>-5 & bearing<5){
//                                 // vitesseG = 0;
//                                 // vitesseD =  0;
//                                 stopMoving();
//                                 etape_intern = State.DIST;
//                                 telemetry.addData("bearing","GOTO DIST");
//                             }
//                             else{
//                                 if(bearing>0){   //positif
//                                     vitG = 0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = -0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","gauche");
//                                 }
//                                 else{            //negatif
//                                     vitG = -0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = 0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","droite");
//                                 }
//                                 avantDroit.setVelocity(vitG);
//                                 arriereDroit.setVelocity(vitD);
//                                 avantGauche.setVelocity(vitD);
//                                 arriereGauche.setVelocity(vitG);
//                             }
//                             telemetry.update();
//                             // sleep(700);
//                             break;
//                     }
//
//                 case TIRER2:
//                     tir(ordre);
//                     rotation("GAUCHE",45);
//                     etape = State.RANGEE3;
//                     break;
//
//                 case RANGEE3:
//                     translation("ARRIERE",60);
//                     translation("GAUCHE",180);
//                     etape = State.MANGERBALLES3;
//                     timer.reset();
//                     moissoneuse.setPower(-1);
//
//                     vitG = 0.25*VITESSE_MAX_MOTEUR;
//                     vitD = 0.25*VITESSE_MAX_MOTEUR;
//                     avantDroit.setVelocity(vitG);
//                     arriereDroit.setVelocity(vitD);
//                     avantGauche.setVelocity(vitG);
//                     arriereGauche.setVelocity(vitD);
//                     break;
//
//                 case MANGERBALLES3:
//                      temps = timer.milliseconds();
//                     telemetry.addData("duree",duree);
//                     telemetry.addData("timer",temps);
//                     if (temps > duree){
//                          telemetry.addData("FIN","FIN");
//                         stopMoving();
//                         if (temps > duree + 2000){
//                             moissoneuse.setPower(0);
//                             trieurColor.setPosition(POS_INIT_TRI);
//                             etape = State.GOTO_TIR3;
//                           telemetry.update();
//                             break;
//
//                         }
//                         telemetry.update();
//                     }
//                     colors = colorSensor.getNormalizedColors();
//                     hue = JavaUtil.colorToHue(colors.toColor());
//
//                     if (100<=hue && hue<200){//Vertes
//                         trieurColor.setPosition(1);
//                     }
//                     else if (200<=hue){//Violettes
//                         trieurColor.setPosition(0);
//                     }
//                     else if (hue < 5) { trieurColor.setPosition(POS_INIT_TRI);}
//                     telemetry.update();
//                     break;
//
//                 case GOTO_TIR3:
//                     translation("ARRIERE",60);
//                     translation("DROITE",180);
//                     rotation("DROITE",45);
//                     etape = State.SE_PLACER3;
//                     break;
//
//                 case SE_PLACER3:
//                     etape_intern = State.YAW;
//                     switch(etape_intern){
//                         case YAW:
//                             double yaw = 0.0;
//                             myAprilTagDetections = (myAprilTagProcessor.getDetections());
//                             for (AprilTagDetection myAprilTag : myAprilTagDetections){
//                                 if (myAprilTag.id == 20) {// BLEU
//                                     yaw = myAprilTag.ftcPose.yaw;
//                                 }
//                             }
//                             telemetry.addData("yaw",yaw);
//                             if(Math.abs(yaw)<5){
//                                 telemetry.addData("yaw","GOTO CENTR");
//                                 stopMoving();
//                                 etape_intern = State.CENTR;
//
//                             }
//                             else{
//                                 if(yaw > 0){ //GAUCHE
//                                     vitG =  -0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = 0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","gauche");
//                                 }
//                                 else{
//                                     vitG = 0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = -0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","droite");
//                                 }
//                                 avantDroit.setVelocity(vitD);
//                                 arriereDroit.setVelocity(vitD);
//                                 avantGauche.setVelocity(vitG);
//                                 arriereGauche.setVelocity(vitG);
//                             }
//                             // rotation(,);
//                             telemetry.update();
//                             // sleep(700);
//                             break;
//
//                         case DIST:
//                              double range = 0.0;
//
//                             for (AprilTagDetection myAprilTag : myAprilTagDetections){
//                                 if (myAprilTag.id == 20) {// range
//                                     range = myAprilTag.ftcPose.range*2.54-62;
//                                 }
//                                 }
//                             telemetry.addData("range",range);
//                             if(range >-5 & range <5){
//                                 stopMoving();
//                                 etape = State.TIRER3;
//                                 telemetry.addData("range","GOTO TIR");
//                             }
//                             else{
//                                 if(range>0){   //positif
//                                     vitG = 0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = 0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","avant");
//                                 }
//                                 else{            //negatif
//                                     vitG = -0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = -0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","arriere");
//                                 }
//                                 avantDroit.setVelocity(vitG);
//                                 arriereDroit.setVelocity(vitD);
//                                 avantGauche.setVelocity(vitG);
//                                 arriereGauche.setVelocity(vitD);
//                             }
//                             telemetry.update();
//                             // sleep(700);
//                             break;
//                         case CENTR:
//                             double bearing = 0.0;
//                             for (AprilTagDetection myAprilTag : myAprilTagDetections){
//                                 if (myAprilTag.id == 20) {// bearing
//                                     bearing = myAprilTag.ftcPose.bearing;
//                                 }
//                             }
//                             telemetry.addData("bearing",bearing);
//                             if(bearing>-5 & bearing<5){
//                                 // vitesseG = 0;
//                                 // vitesseD =  0;
//                                 stopMoving();
//                                 etape_intern = State.DIST;
//                                 telemetry.addData("bearing","GOTO DIST");
//                             }
//                             else{
//                                 if(bearing>0){   //positif
//                                     vitG = 0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = -0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","gauche");
//                                 }
//                                 else{            //negatif
//                                     vitG = -0.25*VITESSE_MAX_MOTEUR;
//                                     vitD = 0.25*VITESSE_MAX_MOTEUR;
//                                     telemetry.addData("direction","droite");
//                                 }
//                                 avantDroit.setVelocity(vitG);
//                                 arriereDroit.setVelocity(vitD);
//                                 avantGauche.setVelocity(vitD);
//                                 arriereGauche.setVelocity(vitG);
//                             }
//                             telemetry.update();
//                             // sleep(700);
//                             break;
//                     }
//
//                 case TIRER3:
//                     tir(ordre);
//                     rotation("GAUCHE",45);
//                     etape = State.IMUHELP;
//                     break;
//
//                 case IMUHELP:
//                     rotation("GAUCHE",45);
//                     translation("ARRIERE",40);
//                     etape = State.FIN;
//                     break;
//                 case FIN:
//                     break;
//
//              }
            
              //}
            
            
            
//             switch(etape){
//                 case DEPL1:
//                     translation("AVANT",55);
//                     stopMoving();
//                     etape = State.DETECT;
//
//                     break;
//                 case DETECT:
//                     myAprilTagDetections = (myAprilTagProcessor.getDetections());
//                     for (AprilTagDetection myAprilTagDetection2 : myAprilTagDetections) {
//                         myAprilTagDetection = myAprilTagDetection2;
//                           if (myAprilTagDetection.id == 21){
//                             ordre.add("G");
//                             ordre.add("P");
//                             ordre.add("P");
//                           }
//                           else if (myAprilTagDetection.id == 22){
//                             ordre.add("P");
//                             ordre.add("G");
//                             ordre.add("P");
//                           }
//                           else if (myAprilTagDetection.id == 23){
//                             ordre.add("P");
//                             ordre.add("P");
//                             ordre.add("G");
//                           }
//                           else {
//                               telemetry.addData("Ordre","Couleurs non detectées");
//                           }
//                   }
//
//                     if (!ordre.isEmpty()){
//                         etape = State.DEPL2;
//                     }
//                     break;
//
//                 case DEPL2:
//                     translation("GAUCHE",58);
//                     etape = State.MANGEBALLES;
//                     moissoneuse.setPower(-1);
//                     double vitesse =VITESSE_MAX_MOTEUR/20*Math.sqrt(2);// VITESSE_MAX_MOTEUR/8*Math.sqrt(2);
//                     duree = (int) (calculDuree(80)*3);
//                     avantDroit.setVelocity(vitesse);
//                     arriereDroit.setVelocity(vitesse);
//                     avantGauche.setVelocity(vitesse);
//                     arriereGauche.setVelocity(vitesse);
//                     timer.reset();
//                     break;
//
//                 case MANGEBALLES:
//                     double temps = timer.milliseconds();
//                     telemetry.addData("duree",duree);
//                     telemetry.addData("timer",temps);
//                     if (temps > duree){
//                         telemetry.addData("FIN","FIN");
//                         stopMoving();
//                         if (temps > duree + 2000){
//                             moissoneuse.setPower(0);
//                             trieurColor.setPosition(POS_INIT_TRI);
//                             etape = State.DEPL3;
//                             telemetry.update();
//                             break;
//
//                         }
//                         telemetry.update();
//                     }
//                     colors = colorSensor.getNormalizedColors();
//                     hue = JavaUtil.colorToHue(colors.toColor());
//
//                     if (100<=hue && hue<200){//Vertes
//                         trieurColor.setPosition(1);
//                     }
//                     else if (200<=hue){//Violettes
//                         trieurColor.setPosition(0);
//                     }
//                     else if (hue < 5) { trieurColor.setPosition(POS_INIT_TRI);}
//                     telemetry.update();
//                     break;
//
//                     case DEPL3:
//                     translation("ARRIERE",50);
//                     translation("DROITE",90);
//                     if (ordre.isEmpty()){
//                         telemetry.addData("ordre","vide");
//                     }
//                     else{
//                         telemetry.addData("ordre",ordre.get(0)+ordre.get(1)+ordre.get(2));
//                     }
//                     telemetry.update();
//                     etape = State.YAW;
//                     break;
//                 case YAW:
//                     double yaw = 0.0;
//                     myAprilTagDetections = (myAprilTagProcessor.getDetections());
//                     for (AprilTagDetection myAprilTag : myAprilTagDetections){
//                         if (myAprilTag.id == 20) {// BLEU
//                             yaw = myAprilTag.ftcPose.yaw;
//                         }
//                     }
//                     telemetry.addData("yaw",yaw);
//                     if(Math.abs(yaw)<5){
//                         telemetry.addData("yaw","GOTO CENTR");
//                         stopMoving();
//                         etape = State.CENTR;
//
//                     }
//                     else{
//                         if(yaw > 0){ //GAUCHE
//                             vitG =  -0.25*VITESSE_MAX_MOTEUR;
//                             vitD = 0.25*VITESSE_MAX_MOTEUR;
//                             telemetry.addData("direction","gauche");
//                         }
//                         else{
//                             vitG = 0.25*VITESSE_MAX_MOTEUR;
//                             vitD = -0.25*VITESSE_MAX_MOTEUR;
//                             telemetry.addData("direction","droite");
//                         }
//                         avantDroit.setVelocity(vitD);
//                         arriereDroit.setVelocity(vitD);
//                         avantGauche.setVelocity(vitG);
//                         arriereGauche.setVelocity(vitG);
//                     }
//                     // rotation(,);
//                     telemetry.update();
//                     // sleep(700);
//                     break;
//
//                 case DIST:
//                      double range = 0.0;
//
//                     for (AprilTagDetection myAprilTag : myAprilTagDetections){
//                         if (myAprilTag.id == 20) {// range
//                             range = myAprilTag.ftcPose.range*2.54-62;
//                         }
//                     }
//                     telemetry.addData("range",range);
//                     if(range >-5 & range <5){
//                         stopMoving();
//                         etape = State.TIR;
//                         telemetry.addData("range","GOTO TIR");
//                     }
//                     else{
//                         if(range>0){   //positif
//                             vitG = 0.25*VITESSE_MAX_MOTEUR;
//                             vitD = 0.25*VITESSE_MAX_MOTEUR;
//                             telemetry.addData("direction","avant");
//                         }
//                         else{            //negatif
//                             vitG = -0.25*VITESSE_MAX_MOTEUR;
//                             vitD = -0.25*VITESSE_MAX_MOTEUR;
//                             telemetry.addData("direction","arriere");
//                         }
//                         avantDroit.setVelocity(vitG);
//                         arriereDroit.setVelocity(vitD);
//                         avantGauche.setVelocity(vitG);
//                         arriereGauche.setVelocity(vitD);
//                     }
//                     telemetry.update();
//                     // sleep(700);
//                     break;
//                 case CENTR:
//                     double bearing = 0.0;
//                     for (AprilTagDetection myAprilTag : myAprilTagDetections){
//                         if (myAprilTag.id == 20) {// bearing
//                             bearing = myAprilTag.ftcPose.bearing;
//                         }
//                     }
//                     telemetry.addData("bearing",bearing);
//                     if(bearing>-10 & bearing<10){
//                         // vitesseG = 0;
//                         // vitesseD =  0;
//                         stopMoving();
//                         etape = State.DIST;
//                         telemetry.addData("bearing","GOTO DIST");
//                     }
//                     else{
//                         if(bearing>0){   //positif
//                             vitG = 0.25*VITESSE_MAX_MOTEUR;
//                             vitD = -0.25*VITESSE_MAX_MOTEUR;
//                             telemetry.addData("direction","gauche");
//                         }
//                         else{            //negatif
//                             vitG = -0.25*VITESSE_MAX_MOTEUR;
//                             vitD = 0.25*VITESSE_MAX_MOTEUR;
//                             telemetry.addData("direction","droite");
//                         }
//                         avantDroit.setVelocity(vitG);
//                         arriereDroit.setVelocity(vitD);
//                         avantGauche.setVelocity(vitD);
//                         arriereGauche.setVelocity(vitG);
//                     }
//                     telemetry.update();
//                     // sleep(700);
//                     break;
//
//                     case TIR :
//                         tir(ordre);
//                         etape = State.SUITE;
//                         break;
//                 case SUITE:
//                     displayVisionPortalData();
//                     break;
//
//             }
            
        
            
            
            
//             myAprilTagDetections = (myAprilTagProcessor.getDetections());
//         for (AprilTagDetection myAprilTagDetection2 : myAprilTagDetections) {
//             myAprilTagDetection = myAprilTagDetection2;
//             // telemetry.addData("ID", (myAprilTagDetection.id));
//             if (myAprilTagDetection.id == 21){
//                 telemetry.addData("Ordre","GPP");
//             }
//             else if (myAprilTagDetection.id == 22){
//                 telemetry.addData("Ordre","PGP");
//             }
//             else if (myAprilTagDetection.id == 23){
//                 telemetry.addData("Ordre","PPG");
//             }
//             else {
//                 telemetry.addData("Ordre","Couleurs non detectées");
//             }
//         }
//         telemetry.update();
    }
}
    
}
