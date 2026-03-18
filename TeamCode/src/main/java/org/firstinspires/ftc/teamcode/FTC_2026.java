/*
Copyright 2025 FIRST Tech Challenge Team FGC

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
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import java.util.List;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.robotcore.hardware.RCservo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp

public class FTC_2026 extends LinearOpMode {

    private IMU imu = null;
    private DcMotorEx arriereGauche = null;
    private DcMotorEx arriereDroit = null;
    private DcMotorEx avantGauche = null;
    private DcMotorEx avantDroit = null;
    private int VITESSE_MAX_MOTEUR = 2800; //600; // tick par seconde
    private int VELOCITY_MAX_HEX = 600;
    private DcMotorEx moissoneuse = null;
    private DcMotorEx roueLanceur = null;
    private DcMotorEx roueLanceur2 = null;
    private Servo trieurColor = null;
    private NormalizedRGBA colors;
    private NormalizedColorSensor colorSensor = null;
    private Servo servoViolet = null;
    private Servo servoVert = null;
    private Servo servoPelle = null;

    private boolean AVEC_IMU = false;
    private boolean rlBumperAlreadyPressed = false;
    private boolean start2AlreadyPressed = false;
    private boolean moissoneuseAlreadyPressed = false;
    private boolean moissoneuseMoissone = false;
    private boolean padRightTriAlreadyPressed = false;
    private boolean padLeftTriAlreadyPressed = false;
    private boolean circleLanceurAlreadyPressed = false;
    private boolean lanceurON = false;
    private boolean squarePelleAlreadyPressed = false;
    private boolean servoBouge = false;
    private boolean circlePelleAlreadyPressed = false;
    private boolean trianglePelleAlreadyPressed = false;
    private boolean upVioletAlreadyPressed = false;
    private boolean left2VioletAlreadyPressed = false;
    private boolean leftVioletAlreadyPressed = false;
    private boolean left2VertAlreadyPressed = false;
    private boolean right2VertAlreadyPressed = false;
    private boolean rightVertAlreadyPressed = false;
    private boolean triangle2VertAlreadyPressed = false;
    private boolean cross2VioletAlreadyPressed = false;

    private double POS_INIT_TRI = 0.47;
    // private double POS_TEST_TRI = 0.5;
    private double BAS_PELLE = 0.56;
    private double HAUT_PELLE = 0.4;
    private double hue = 0.0;

    private double posPelle = 0.06;
    private double posViolet = 0.2;
    private double posVert = 1;
    private double BLOCAGE_VIOLET = 0.42;
    private double COTE_TRIEUR_VERT = 1; ////////
    private double MILIEU_TRIEUR = 0.5; ///////
    private double COTE_TRIEUR_VIOLET = 0; ///////
    ///
    private double PASSAGE_VIOLET = 0.19;
    private double BLOCAGE_VERT = 0.53;
    private double PASSAGE_VERT = 0.8;
    private double VITESSE_LANCEUR = 0.7;
    private boolean triAuto = true;

    private double triVert ;
    private double triViolet ;


    private enum State {
        Idle,
        etat2,
        etat3,


    }

    private State etapeViolet = State.Idle;
    private State etapeVert = State.Idle;
    private State etapePelle = State.Idle;
    private ElapsedTime timer = new ElapsedTime();

    private VoltageSensor batterie;


    // private float r,g,b;

    // jai tres visiblement fait n'importe quoi, osseukour qu'ai-je fait

    @Override
    public void runOpMode() {

        telemetry.addData("Status_nouvaeu", "Initialized");


        batterie = hardwareMap.get(VoltageSensor.class, "Control Hub");
        telemetry.addData("batterie",batterie.getVoltage());
        avantDroit  = hardwareMap.get(DcMotorEx.class, "avantdroit");
        avantGauche  = hardwareMap.get(DcMotorEx.class, "avantgauche");
        arriereDroit = hardwareMap.get(DcMotorEx.class, "arrieredroit");
        arriereGauche = hardwareMap.get(DcMotorEx.class, "arrieregauche");
        moissoneuse = hardwareMap.get(DcMotorEx.class, "moissoneuse");
        roueLanceur =  hardwareMap.get(DcMotorEx.class, "motorlanceur");
        roueLanceur2 =  hardwareMap.get(DcMotorEx.class, "motorlanceur2");
        trieurColor = hardwareMap.get(Servo.class, "ejecttrieur");
        servoViolet = hardwareMap.get(Servo.class, "servoviolet");
        servoVert = hardwareMap.get(Servo.class, "servovert");
        servoPelle = hardwareMap.get(Servo.class, "servopelle");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        imu = hardwareMap.get(IMU.class, "imu");

        telemetry.update();

        RevHubOrientationOnRobot.LogoFacingDirection logo = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usb = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logo,usb)));
        imu.resetYaw();

        avantDroit.setDirection(DcMotorEx.Direction.REVERSE);
        arriereDroit.setDirection(DcMotorEx.Direction.REVERSE);

        avantDroit.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
        avantDroit.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        avantGauche.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
        avantGauche.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arriereDroit.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
        arriereDroit.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        arriereGauche.setMode(DcMotorEx.RunMode.RUN_USING_ENCODERS);
        arriereGauche.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        trieurColor.setPosition(POS_INIT_TRI);
        triVert = POS_INIT_TRI;
        triViolet = POS_INIT_TRI;

        posPelle = HAUT_PELLE;
        posVert = BLOCAGE_VERT;
        posViolet = BLOCAGE_VIOLET;
        servoPelle.setPosition(posPelle);
        servoViolet.setPosition(posViolet);
        servoVert.setPosition(posVert);




        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // r = colorSensor.red;
            // g = colorSensor.green;
            // b = colorSensor.blue;
            colors = colorSensor.getNormalizedColors();
            hue = JavaUtil.colorToHue(colors.toColor());
            // r = colors.red;
            // g = colors.green;
            // b = colors.blue;


            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            if (AVEC_IMU){
                double theta = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                double xTheta = x * Math.cos(-theta)-y* Math.sin(-theta);
                double yTheta = x* Math.sin(-theta)+y* Math.cos(-theta);
                x = xTheta;
                y = yTheta;
                telemetry.addData("le Theta", theta);
            }

            // calcule un peu compliqué pour ma personne
            double z = gamepad1.right_stick_x;
            double v_arg_avd;
            double v_ard_avg;
            double vitesseRotation;
            v_arg_avd = x* -Math.sqrt(2)/2 + y* Math.sqrt(2)/2 ;
            v_ard_avg = x* Math.sqrt(2)/2 + y* Math.sqrt(2)/2 ;
            double vitesseMax;
            double vitesseRotation_g = z ;
            double vitesseRotation_d = -z ;
            double vitesseRoueAVD = v_arg_avd+vitesseRotation_d;
            double vitesseRoueAVG = v_ard_avg+vitesseRotation_g;
            double vitesseRoueARG = v_arg_avd+vitesseRotation_g;
            double vitesseRoueARD = v_ard_avg+vitesseRotation_d;
            double maximum = Math.max(Math.max(Math.abs(vitesseRoueAVD),Math.abs(vitesseRoueARD)),Math.max(Math.abs(vitesseRoueAVG),Math.abs(vitesseRoueARG)));
            double rTrigger = gamepad1.right_trigger;

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

            // DEBUT RGB HSW BRDEL EXTRAORDINAIRE

            if(triAuto){
                if(100<=hue && hue<200){ // Vertes
                    trieurColor.setPosition(1);
                }
                else if(200<=hue){ //Violettes
                    trieurColor.setPosition(0);
                }
                else if (hue < 5){
                    trieurColor.setPosition(POS_INIT_TRI);
                }
            }
            // DEBUT CHANGER MODE  Manette 22222
            if (gamepad2.right_bumper && gamepad2.left_bumper){
                if (!rlBumperAlreadyPressed){
                    AVEC_IMU = !AVEC_IMU;
                    rlBumperAlreadyPressed = true;
                }
            }
            else {
                rlBumperAlreadyPressed = false;
            }
            // FIN CHANGER MODE  Manette 222222

            // DEBUT REINITIALISATION IMU      Manette 2222222
            if (gamepad2.right_bumper){
                if (!start2AlreadyPressed){
                    start2AlreadyPressed = !start2AlreadyPressed;
                    imu.resetYaw();
                }
            }
            else {
                start2AlreadyPressed = false;
            }
            // FIN REINITIALISATION IMU      Manette 2222222


            // DEBUT SWITCH CHARGERp VIPOLET       Manette111111111

//            switch(etapeViolet){
//                case Idle:
//                    if(gamepad1.dpad_left){
//                        etapeViolet = State.etat2;
//                    }
//                    break;
//
//                case etat2:
//                    servoViolet.setPosition(PASSAGE_VIOLET);
//                    servoPelle.setPosition(BAS_PELLE);
//                    posPelle = BAS_PELLE;
//                    timer.reset();
//                    etapeViolet = State.etat3;
//                    break;
//
//                case etat3:
//                    double temps = timer.milliseconds();
//                    double attente = 400 ;
//                    if(temps>attente){
//                        servoViolet.setPosition(BLOCAGE_VIOLET);
//                        etapeViolet = State.Idle;
//                    }
//                    break;
//            }


            //FIN SWITCH CHARGERp VIOLET           Manette11111111



            //DEBUT SWITCH CHARGER VERRT       Mannette111111111111
//            switch(etapeVert){
//                case Idle:
//                    if(gamepad1.dpad_right){
//                        etapeVert = State.etat2;
//                    }
//                    break;
//
//                case etat2:
//                    servoVert.setPosition(PASSAGE_VERT);
//                    servoPelle.setPosition(BAS_PELLE);
//                    posPelle = BAS_PELLE;
//                    timer.reset();
//                    etapeVert = State.etat3;
//                    break;
//
//                case etat3:
//                    double tempsVert = timer.milliseconds();
//                    double attenteVert = 1400 ;
//                    if(tempsVert>attenteVert){
//                        servoVert.setPosition(BLOCAGE_VERT);
//                        etapeVert = State.Idle;
//                    }
//                    break;
//            }

            if (gamepad1.dpadLeftWasPressed()){
                servoViolet.setPosition(PASSAGE_VIOLET);
                servoPelle.setPosition(BAS_PELLE);
                posPelle = BAS_PELLE;
            }
            if (gamepad1.dpadRightWasPressed()){
                servoVert.setPosition(PASSAGE_VERT);
                servoPelle.setPosition(BAS_PELLE);
                posPelle = BAS_PELLE;
            }
            //FIN SWITCH CHARGER VERT        Manette11111111111

            // DEBUT MOISSONEUSE CONTINU Manette 11111111
            if ( gamepad1.square & !moissoneuseAlreadyPressed){
                if (!moissoneuseMoissone){
                    moissoneuse.setPower(-1);
                }
                else {
                    moissoneuse.setPower(0);
                }
                moissoneuseAlreadyPressed = true;
                moissoneuseMoissone = !moissoneuseMoissone;
            }
            else if (!gamepad1.square){
                moissoneuseAlreadyPressed = false;
            }
            // FIN MOISSONEUSE CONTINU   Manette 1111111111




            // if (trieurColor.setPosition() == 0.5){
            //     trieurColor.setPosition(POS_INIT_TRI);
            // }

            //  FIN TRIEUR MNULE TEMPORAIRE   Manette   111111

            // REMETTRE A ZERO TRIEUR     Manette 11111111

            // if (gamepad1.dpad_down && !padLeftTriAlreadyPressed){
            //     padLeftTriAlreadyPressed = true;
            //     trieurColor.setPosition(POS_INIT_TRI);
            // }
            // else {
            //     padLeftTriAlreadyPressed = false;
            // }

            // FIN REMETTRE A ZERO TRIEUR   Manette 11111111

            // DEBUT ROUE LANCEUR   Manette 11111111

            if(gamepad1.circleWasPressed()){
//                if(!circleLanceurAlreadyPressed){
//                    circleLanceurAlreadyPressed = true;
                    if (lanceurON){
                        roueLanceur2.setPower(0);
                        roueLanceur.setPower(0);
                        lanceurON = false;
                    }
                    else {
                        roueLanceur.setPower(-VITESSE_LANCEUR);       //BATERIE 12,à 1, 58 cm
                        roueLanceur2.setPower(VITESSE_LANCEUR);      //BATERIE 12,à 0.9, 60 cm
                        lanceurON = true;
                    }

            }


            // FIN ROUE LANCEUR    Manette 11111111111

            // DEBUT SERVO PELLE           Manette 1111111111111111111111

            // if (gamepad1.dpad_up){
            //     if (!squarePelleAlreadyPressed){
            //         squarePelleAlreadyPressed = true;
            //             servoPelle.setPosition();


            //     }

            // }
            // else if (!gamepad2.dpad_up){
            //     squarePelleAlreadyPressed = false;
            // }


            // if (gamepad1.dpad_left){
            //     servoPelle.setPosition(0.7);
            // }

            //DEBUT PETIT A PETIT MONTER PELLE  Manette 22222222222222
            if (gamepad2.square && !squarePelleAlreadyPressed){

                squarePelleAlreadyPressed = true;
                posPelle = posPelle - 0.02;
                servoPelle.setPosition(posPelle);
            }

            else if (!gamepad2.square){

                squarePelleAlreadyPressed = false;

            }
            //FIN PETIT A PETIT MONTER PELLE  Manette 22222222222222


            // DEBUT PETIT A PETIT DESCENDRE PELLE   Manette 22222222222
            if (gamepad2.circle && !circlePelleAlreadyPressed){

                circlePelleAlreadyPressed = true;
                posPelle = posPelle + 0.02;
                servoPelle.setPosition(posPelle);
            }

            else if (!gamepad2.circle){

                circlePelleAlreadyPressed = false;

            }
            //FIN PETIT A PETIT DESCENDRE PELLE    Manette  22222222222222

            //DEBUT AUTO CATAPULTE (PELLE) Manette    1111111111

//            if (gamepad1.triangle && ! trianglePelleAlreadyPressed){
//                trianglePelleAlreadyPressed = true;
//                if (posPelle != HAUT_PELLE){posPelle = HAUT_PELLE;}
//                else {posPelle = BAS_PELLE;}
//                servoPelle.setPosition(posPelle);
//            }
//            else if (!gamepad1.triangle){
//                trianglePelleAlreadyPressed = false;
//            }

//            if (gamepad1.triangleWasPressed()){
//                if (posPelle != HAUT_PELLE){posPelle = HAUT_PELLE;}
//                else {posPelle = BAS_PELLE;}
//                servoPelle.setPosition(posPelle);
//            }
            switch (etapePelle){
                case Idle:
                    if (gamepad1.triangleWasPressed()){
                        if (posPelle == BAS_PELLE) {
                            etapePelle = State.etat2;
                        }
                        else {
                            posPelle = BAS_PELLE;
                            servoPelle.setPosition(BAS_PELLE);
                        }

                    }
                    break;
                case etat2:
                    servoViolet.setPosition(BLOCAGE_VIOLET);
                    servoVert.setPosition(BLOCAGE_VERT);
                    timer.reset();
                    etapePelle = State.etat3;
                    break;

                case etat3:
                    double tempsPelle = timer.milliseconds();
                    double attentePelle = 1000 ;
                    if(tempsPelle>attentePelle){
                        servoPelle.setPosition(HAUT_PELLE);
                        posPelle = HAUT_PELLE;
                        etapePelle = State.Idle;
                    }
                    break;
            }

            // FIN AUTO CATAPULTE          Manette    11111111111111


            // // DEBUT PETIT A PETIT COLOR TRIEUR
            // if(gamepad1.cross){
            //     trieurColor.setPosition(trieurColor.setPosition()+)
            // }
            // FIN PETIT A PETIT COLR TRIEUR

            // DEBUT BOUGER SERVO VIOLET PETIT A PETIT     Manette 222222222222
            //mettre en o
            if (gamepad2.dpad_up && !upVioletAlreadyPressed){

                upVioletAlreadyPressed = true;
                posViolet = posViolet + 0.02;
                servoViolet.setPosition(posViolet);
            }

            else if (!gamepad2.dpad_up){

                upVioletAlreadyPressed = false;

            }
            //mettre en ba
            if (gamepad2.dpad_left && !left2VioletAlreadyPressed){

                left2VioletAlreadyPressed = true;
                posViolet = posViolet - 0.02;
                servoViolet.setPosition(posViolet);
            }

            else if (!gamepad2.dpad_left){

                left2VioletAlreadyPressed = false;

            }
            // FIN BOUGER SERVO  VIOLET       Manette 222222222

            // DEBUT BOUGER SERVO VERT  PETIT A PETIT    Manette 222222222222
            //mettre en o
            if (gamepad2.dpad_right && !left2VertAlreadyPressed){

                left2VertAlreadyPressed = true;
                posVert = posVert + 0.02;
                servoVert.setPosition(posVert);
            }

            else if (!gamepad2.dpad_right){

                left2VertAlreadyPressed = false;

            }
            //mettre en ba
            if (gamepad2.dpad_down && !right2VertAlreadyPressed){

                right2VertAlreadyPressed = true;
                posVert = posVert - 0.02;
                servoVert.setPosition(posVert);
            }

            else if (!gamepad2.dpad_down){

                right2VertAlreadyPressed = false;

            }
            // FIN BOUGER SERVO  VERT       Manette 222222222

            //DEBUT PORTAIL AUTO VIOLET      Manette 11111111111

            // if (gamepad1.dpad_left && ! leftVioletAlreadyPressed){
            //     leftVioletAlreadyPressed = true;
            //     if (posViolet != PASSAGE_VIOLET){posViolet = PASSAGE_VIOLET;}
            //     else {posViolet = BLOCAGE_VIOLET;}
            //     servoViolet.setPosition(posViolet);
            // }
            // else if (!gamepad1.dpad_left){
            //     leftVioletAlreadyPressed = false;
            // }

            // FIN PORTAIL AUTO VIOLET    Manette 11111111111111

            //DEBUT PORTAIL AUTO VERT      Manette 11111111111

            // if (gamepad1.dpad_right && ! rightVertAlreadyPressed){
            //     rightVertAlreadyPressed = true;
            //     if (posVert != PASSAGE_VERT){posVert = PASSAGE_VERT;}
            //     else {posVert = BLOCAGE_VERT;}
            //     servoVert.setPosition(posVert);
            // }
            // else if (!gamepad1.dpad_right){
            //     rightVertAlreadyPressed = false;
            // }
            // FIN PORTAIL AUTO Manette 1111111111111111

            //DEBUT TRIEUR MANEL AUTO VERT      Manette 2222222222

            if (gamepad2.triangle && ! triangle2VertAlreadyPressed){
                triangle2VertAlreadyPressed = true;
                if (triVert != COTE_TRIEUR_VERT){
                    triAuto = false;
                    triVert = COTE_TRIEUR_VERT;}
                else {
                    triAuto = true;
                    triVert = POS_INIT_TRI;}
                trieurColor.setPosition(triVert);
            }
            else if (!gamepad2.triangle){
                triangle2VertAlreadyPressed = false;
            }
            // FIN TRIEUR AUTO VERT Manette 1111111111111111

            //DEBUT TRIEUR MANEL AUTO VIOLET     Manette 2222222222

            if (gamepad2.cross && ! cross2VioletAlreadyPressed){
                cross2VioletAlreadyPressed = true;
                if (triViolet != COTE_TRIEUR_VIOLET){
                    triAuto = false;
                    triViolet = COTE_TRIEUR_VIOLET;}
                else {
                    triAuto = true;
                    triViolet = POS_INIT_TRI;}
                trieurColor.setPosition(triViolet);
            }
            else if (!gamepad2.cross){
                cross2VioletAlreadyPressed = false;
            }
            // FIN TRIEUR AUTO VERT Manette 1111111111111111

            // DEBUT BOUGER SERVO TRIEUR Manette 1111111111
            //mettre a 0
            // if(gamepad1.triangle){
            //     trieurColor.setPosition(0);
            // }
            // // Mettre a 1
            // if(gamepad1.cross){
            //     trieurColor.setPosition(1);
            // }
            // FIN BOUGER SERVO TRIEUR        Manette 1111111111

            telemetry.addData("Status", "Running");

            telemetry.addData("xJoystickGauche", x);
            telemetry.addData("yJoystickGauche", y);
            telemetry.addData("zJoystickDroit", z);
            telemetry.addData("rTrigger", rTrigger);

            telemetry.addData("arriereGauche", arriereGauche.getVelocity());
            telemetry.addData("avantGauche", avantGauche.getVelocity());
            telemetry.addData("avantDroit", avantDroit.getVelocity());
            telemetry.addData("arriereDroit", arriereDroit.getVelocity());

            telemetry.addData("padRightTriAlreadyPressed", padRightTriAlreadyPressed);

            // telemetry.addData("R",r);
            // telemetry.addData("G", g);
            // telemetry.addData("B", b);
            telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
            telemetry.addData("Saturation", "%.3f", JavaUtil.colorToSaturation(colors.toColor()));
            telemetry.addData("Value", "%.3f", JavaUtil.colorToValue(colors.toColor()));

            telemetry.addData("combi mode IMU", rlBumperAlreadyPressed);
            telemetry.addData("mode IMU", AVEC_IMU);

            // telemetry.addData(" V roue AVD",vitesseRoueAVD);
            // telemetry.addData(" V roue AVG",vitesseRoueAVG);
            // telemetry.addData(" V roue ARG",vitesseRoueARG);
            // telemetry.addData(" V roue ARD",vitesseRoueARD);

            telemetry.addData("squarePelleAlreadyPressed",squarePelleAlreadyPressed);
            telemetry.addData("circlePelleAlreadyPressed",circlePelleAlreadyPressed);
            telemetry.addData("position portail violet",servoViolet.getPosition());
            telemetry.addData("pos violet",posViolet);
            telemetry.addData("pos vert",posVert);

            telemetry.addData("Position servo pelle",posPelle);

            telemetry.update();

        }
    }
}
