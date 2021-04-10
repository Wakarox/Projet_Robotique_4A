#include <motor.h>

#include <DFMobile.h>
#include <DFRobot_HuskyLens.h>
#include <HUSKYLENS.h>
#include <HUSKYLENSMindPlus.h>
#include <HuskyLensProtocolCore.h>
#include <PIDLoop.h>


#define DIRD 8
#define PWMD 9
#define DIRG 7
#define PWMG 6
HUSKYLENS huskylens;
//Cablage communication robot->caméra: HUSKYLENS green line >> SDA; blue line >> SCL
int ID1 = 1;
void printResult(HUSKYLENSResult result);
int left = 0, right = 0;

void setup() {
  Serial.begin(115200);
  // put your setup code here, to run once:
  Wire.begin();
  while (!huskylens.begin(Wire))
  {
    Serial.println(F("Begin failed!"));
    Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protol Type>>I2C)"));
    Serial.println(F("2.Please recheck the connection."));
    delay(1000);
  }
  huskylens.writeAlgorithm(ALGORITHM_LINE_TRACKING); //Switch the algorithm to line tracking.
  pinMode(DIRD, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(DIRG, OUTPUT);
  pinMode(PWMG, OUTPUT);

}

void loop() {
  //Variables : 
  float error_Target, error_Origin, speed_Ratio; //Erreurs angulaire, erreur linéaire, et ratio de vitesse
  int32_t Angular_Speed, Linear_Speed; //Vitesse angulaire et linéaire
  //Erreur de connexion à la caméra
  if (!huskylens.request(ID1)) {
    Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
  }
  //Erreur d'apprentissage
  else if (!huskylens.isLearned()) {
    Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
  }
  //Non détection de ligne
  else if (!huskylens.available()) {
    Serial.println(F("No arrow appears on the screen!"));
    cmd_robot(0, 0);
  }
  else //Si détection de ligne
  {
    //Lit et affiche les infos caméra
    HUSKYLENSResult result = huskylens.read();
    printResult(result);
    //Calculs des erreurs angulaire et linéaire et du ratio de vitesse
    error_Target = result.xTarget - 160;
    error_Origin = result.xOrigin - 160;
    speed_Ratio = ((result.yOrigin - result.yTarget) / 100);
    Serial.println(error_Target);
    Serial.println(error_Origin);
    //Calculs des vitesses angulaire et linéaire
    Angular_Speed = (int32_t)(error_Target * 0.5);
    Linear_Speed = (int32_t)((speed_Ratio * 150) - abs(error_Origin * 0.3));
    Serial.println(Angular_Speed);
    Serial.println(Linear_Speed);
    //Application des vitesses aux moteurs
    cmd_robot(Linear_Speed, Angular_Speed);
  }
} //Fonction de calcul des valeurs moteurs
void cmd_motor (int mG, int mD) {
  bool sensG = (mG > 0);
  bool sensD = (mD > 0);
  int valG = (mG < 0) ? -mG : +mG;
  int valD = (mD < 0) ? -mD : +mD;
  digitalWrite(DIRG, sensG); 
  analogWrite(PWMG, valG);
  digitalWrite(DIRD, sensD);
  analogWrite(PWMD, valD);
}
 //Fonction de commande des moteurs du robot
void cmd_robot(int l, int a) {
  int mG = (l + a) >> 1;
  int mD = (l - a) >> 1;
  cmd_motor(mG, mD);
}

// Recupération des infos caméra 
void printResult(HUSKYLENSResult result) {
  if (result.command == COMMAND_RETURN_ARROW) {
    Serial.println(String() + F("Arrow:xOrigin=") + result.xOrigin + F(",yOrigin=") + result.yOrigin + F(",xTarget=") + result.xTarget + F(",yTarget=") + result.yTarget + F(",ID=") + result.ID);
  }
  else {
    Serial.println("Object unknown!");
  }
}
