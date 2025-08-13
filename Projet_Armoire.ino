

#include <Wire.h>
#include <PCF8574.h>
#include <Keypad_I2C.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_TCS34725.h>
#include <Servo.h>

#define PCF8574_ADDR         0x20   // adresse du PCF8574 pour le clavier et sorties du moteur
#define LCD_ADDR             0x27   // adresse du LCD I2C


PCF8574 pcf(PCF8574_ADDR);
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// ---------------- KEYPAD I2C ----------------
// Layout 4x4
const byte ROWS = 4;
const byte COLS = 4;
char keys[ROWS][COLS] = {
  {'1','2','3','A'},
  {'4','5','6','B'},
  {'7','8','9','C'},
  {'*','0','#','D'}
};
// Les pins logiques (0..7) du PCF8574 reliés au keypad (map selon câblage)
byte rowPins[ROWS] = {0, 1, 2, 3};   // PCF8574 P0..P3 -> rows
byte colPins[COLS] = {4, 5, 6, 7};   // PCF8574 P4..P7 -> cols
Keypad_I2C keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS, PCF8574_ADDR, &Wire);

// ---------------- TCS34725 (capteur couleur) ----------------
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// ---------------- Servo tri ----------------
Servo servoTri;
const uint8_t SERVO_PIN = D5; // pin servo (pwm) - NodeMCU mapping ok
const int SERVO_OPEN = 90;    // position qui laisse passer
const int SERVO_BLOCK = 0;    // position qui éjecte (ou bloque)

// ---------------- Moteurs tapis via L298N ----------------
// IN pins branchés sur sorties du PCF8574 (P0..P7) selon ta dispo
const uint8_t DIR_A_PIN = 0; // PCF8574 P0 -> IN1
const uint8_t DIR_B_PIN = 1; // PCF8574 P1 -> IN2
const uint8_t DIR_C_PIN = 2; // PCF8574 P2 -> IN3
const uint8_t DIR_D_PIN = 3; // PCF8574 P3 -> IN4
// PWM pins (ENA / ENB) sur ESP8266
const uint8_t ENA_PIN = D1; // PWM pour moteur gauche
const uint8_t ENB_PIN = D2; // PWM pour moteur droite

// ---------------- Laser + LDR ----------------
const uint8_t LDR_PIN = A0;
const int THRESHOLD_LDR = 700; // à ajuster suivant tests (0..1023)

// ---------------- Paramètres / Codes ----------------
const String CODE_DEPOT = "1234";        // exemple code dépôt (ou paiement)
const String CODE_ROUGE = "0000";
const String CODE_VERT  = "1111";
const String CODE_BLEU  = "3333";

const unsigned long DUREE_TAPIS_DEPOT = 3000UL;   // ms : durée pour acheminer au bac
const unsigned long DUREE_TAPIS_RETRAIT = 3000UL; // ms : durée pour amener colis devant tri sensor

// ---------------- Variables ----------------
String saisie = "";
bool systemReady = false;

// -------------- PROTOTYPE: fonctions utilitaires --------------
void moteurAvant(int speed) {
  // sens avant : IN1=HIGH, IN2=LOW ; IN3=HIGH, IN4=LOW
  pcf.digitalWrite(DIR_A_PIN, HIGH);
  pcf.digitalWrite(DIR_B_PIN, LOW);
  pcf.digitalWrite(DIR_C_PIN, HIGH);
  pcf.digitalWrite(DIR_D_PIN, LOW);
  analogWrite(ENA_PIN, speed);
  analogWrite(ENB_PIN, speed);
}

void moteurArriere(int speed) {
  // sens inverse
  pcf.digitalWrite(DIR_A_PIN, LOW);
  pcf.digitalWrite(DIR_B_PIN, HIGH);
  pcf.digitalWrite(DIR_C_PIN, LOW);
  pcf.digitalWrite(DIR_D_PIN, HIGH);
  analogWrite(ENA_PIN, speed);
  analogWrite(ENB_PIN, speed);
}

void moteurStop() {
  pcf.digitalWrite(DIR_A_PIN, LOW);
  pcf.digitalWrite(DIR_B_PIN, LOW);
  pcf.digitalWrite(DIR_C_PIN, LOW);
  pcf.digitalWrite(DIR_D_PIN, LOW);
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);
}

void lcdShow(String a, String b = "") {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(a);
  lcd.setCursor(0,1); lcd.print(b);
}

// -------------- LECTURE CLAVIER (I2C) --------------
char lireTouche() {
  char k = keypad.getKey();
  if (k) return k;
  return NO_KEY;
}

// -------------- DETECTION COULEUR (TCS34725) --------------
String detecterCouleur() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  // On peut normaliser ou utiliser ratio
  // Simple heuristique : comparer valeurs RGB
  if (r > g && r > b && r > 100) return "ROUGE";
  if (g > r && g > b && g > 100) return "VERT";
  if (b > r && b > g && b > 100) return "BLEU";
  return "INCONNU";
}

// -------------- LECTURE CODE (4 chiffres) --------------
String lireCodeUtilisateur() {
  String code = "";
  lcdShow("Saisir code:", "----");
  while (code.length() < 4) {
    char k = lireTouche();
    if (k != NO_KEY) {
      if (k == '*') { // effacer
        code = "";
        lcdShow("Saisir code:", "----");
        delay(200);
        continue;
      } else if (k == '#') {
        // ignore si non complet
        continue;
      } else {
        code += k;
        // afficher masqué
        lcd.setCursor(0,1);
        for (uint8_t i=0;i<code.length();i++) lcd.print('*');
        for (uint8_t i=code.length(); i<4; i++) lcd.print('-');
        delay(150);
      }
    }
  }
  delay(200);
  return code;
}

// ---------------- PROCESSUS DEPOT ----------------
void processusDepot() {
  lcdShow("Mode DEPOT", "Appuyez # pour ok");

  while (true) {
    char k = lireTouche();
    if (k == '#') break;
    delay(50);
  }

  // informer client de deposer le colis sur la trappe d'entrée 
  lcdShow("Posez le colis", "Puis validez #");
  // on attend validation paiement via code 
  while (true) {
    char k = lireTouche();
    if (k == '#') { // lancer la saisie du code
      String code = lireCodeUtilisateur();
      if (code == CODE_DEPOT) {
        lcdShow("Paiement OK", "Acheminement...");
        delay(800);
        // lancement du tapis vers l'interieur
        moteurAvant(200); // vitesse 0..255
        delay(DUREE_TAPIS_DEPOT);
        moteurStop();
        lcdShow("Colis en bac", "Depot termine");
        delay(1000);
        break;
      } else {
        lcdShow("Code incorrect", "Reessayez");
        delay(1000);
        lcdShow("Posez le colis", "Puis validez #");
      }
    }
    delay(50);
  }
}

// ---------------- PROCESSUS RETRAIT ----------------
void processusRetrait() {
  lcdShow("Mode RETRAIT", "Appuyez #");
  while (true) {
    char k = lireTouche();
    if (k == '#') break;
    delay(50);
  }

  // saisir code client (4 chiffres)
  String code = lireCodeUtilisateur();
  // valider couleur attendue
  String couleurAttendue = "INVALIDE";
  if (code == CODE_ROUGE) couleurAttendue = "ROUGE";
  else if (code == CODE_VERT) couleurAttendue = "VERT";
  else if (code == CODE_BLEU) couleurAttendue = "BLEU";

  if (couleurAttendue == "INVALIDE") {
    lcdShow("Mauvais code", "Veuillez essayer");
    delay(1500);
    return;
  }

  lcdShow("Placez colis sur", "le tapis; validez #");
  // attendre validation qu'on a mis le colis sur le tapis
  while (true) {
    char k = lireTouche();
    if (k == '#') break;
    delay(50);
  }

  // On fait tourner le tapis en sens inverse vers la zone de tri
  lcdShow("Recherche colis", "Detection couleur...");
  moteurArriere(200);
  // on peut attendre une durée suffisante ou faire un loop avec detection continue
  unsigned long start = millis();
  bool trouve = false;
  while (millis() - start < DUREE_TAPIS_RETRAIT) {
    // on peut lire couleur pendant le passage: si détecte couleurAttendue -> on stop et tri
    String detected = detecterCouleur();
    if (detected == couleurAttendue) {
      trouve = true;
      break;
    }
    delay(150); // fréquence lecture
  }
  moteurStop();

  if (!trouve) {
    lcdShow("Colis non trouve", "Verifier couleur");
    delay(1500);
    return;
  }


  lcdShow("Colis trouvé", "Tri en cours...");
  // si couleur correspond: laisser passer
  // position servo pour laisser passer
  servoTri.write(SERVO_OPEN);
  // On met le tapis en sens avant pour amener devant la sortie jusqu'à barrière laser
  delay(500); // courte pause
  moteurAvant(180);

  // attendre détection barrière laser (LDR baisse valeur)
  while (analogRead(LDR_PIN) > THRESHOLD_LDR) {
    delay(50);
  }
  moteurStop();
  lcdShow("Retrait OK", "Prenez votre colis");
  delay(2000);
  // remettre servo en position blocage pour le prochain colis
  servoTri.write(SERVO_BLOCK);
}

// ---------------- SETUP & LOOP ----------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  // init PCF8574
  pcf.begin();
  // config PCF8574 pins en outputs (par défaut)
  for (uint8_t i=0;i<8;i++) pcf.pinMode(i, OUTPUT);

  // LCD
  lcd.init();
  lcd.backlight();

  // keypad
  keypad.begin();

  // TCS sensor
  if (!tcs.begin()) {
    lcdShow("TCS non detecte","");
    delay(2000);
  } else {
    lcdShow("Capteur couleur", "OK");
    delay(600);
  }

  // servo
  servoTri.attach(SERVO_PIN);
  servoTri.write(SERVO_BLOCK);

  // PWM pins config (ESP8266 analogWrite frequency default acceptable)
  analogWriteFreq(1000); // 1kHz
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
  analogWrite(ENA_PIN, 0);
  analogWrite(ENB_PIN, 0);

  lcdShow("Systeme Pret", "Choisir Mode A/B");
  delay(800);
}

void loop() {
  // Menu simple: touche 'A' -> Depot ; 'B' -> Retrait
  lcd.setCursor(0,1);
  lcd.print("A:Depot B:Retrait ");

  char k = lireTouche();
  if (k == 'A') {
    processusDepot();
    lcdShow("Termine", "Retour menu");
    delay(800);
  } else if (k == 'B') {
    processusRetrait();
    lcdShow("Termine", "Retour menu");
    delay(800);
  }
  delay(150);
}
