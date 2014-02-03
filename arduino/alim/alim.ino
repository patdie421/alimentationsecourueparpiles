#include <Arduino.h>

/*
 * Classes locales
 */
class MiniTimer
{
public:
  MiniTimer();
  void restart();
  int checkAndRestart(unsigned long s);
  int check(unsigned long s);
private:
  unsigned long previous;
};


class BlinkLeds 
{   
public:
  BlinkLeds(unsigned int i);
  void run();
  int getLedState() { 
    return ledState; 
  }
private:
  int interval;
  int ledState;
  unsigned long previousMillis;
};


#define __LCD // ligne à supprimer si pas d'afficheur LCD
#ifdef __LCD
#include <Wire.h>  // Pour un afficheur LCD I2C
// Librairie LCD à récupérer ici : 
// https://bitbucket.org/fmalpartida/new-liquidcrystal/downloads
// Remplacer la librairie LCD (liquidcrystal) par cette nouvelle Librairie

// à adapter en fonction de l'afficheur LCD (cf. doc new-liquidcrystal)
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcdScreen(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE); // pour mon afficheur, a adapter

LCD *myLCD = &lcdScreen; // pour que le code soit indépendant du type d'afficheur LCD

#define LCD_MAXPAGES      4 // nombre de page d'info pour l'affichage LCD
#define LCD_MAXBACKLIGHT 30 // durée de rétro-éclairage en seconde
#endif


/*
 * Macros
 */
// masquage/demasquage des interruptions INT0 et INT1
#define ENABLE_INT0_D2  EIMSK |=  (1 << INT0)  // enable interruption INT0 (PIN2)
#define DISABLE_INT0_D2 EIMSK &= ~(1 << INT0)  // disable interruption INT0 (PIN2)
/*
#define ENABLE_INT1_D3  EIMSK |=  (1 << INT1)  // enable interruption INT1 (PIN3)
#define DISABLE_INT1_D3 EIMSK &= ~(1 << INT1)  // disable interruption INT1 (PIN3)
*/
#define ENABLE_TIMER1_COMPA  TIMSK1 |=  (1 << OCIE1A); // interruption du timer1 activé 
#define DISABLE_TIMER1_COMPA TIMSK1 &= ~(1 << OCIE1A); // interruption du timer1 desactivé


/*
 * Constantes de paramétrage
 */
// durée minimum d'une impulsion pour les anti-rebonds
#define PULSE_INTERVAL 25

// seuil de déclenchement des actions
#define SEUIL_ALERT 3.75
#define SEUIL_HS    3.5

// durée d'attente avant de couper l'alimentation
#define GRACE_DELAY 60

// paramétrage des pin IO utilisés
const unsigned char status_led = 13;
const unsigned char vout_input_pin = A0;
const unsigned char vin1_input_pin = A1;
const unsigned char vin2_input_pin = A2;
const unsigned char temp_input_pin = A3;

const unsigned char vin1_status_pin = 4;
const unsigned char vin2_status_pin = 5;
const unsigned char vout_status_pin = 6;

const unsigned char halt_command_pin = 2; // entrée interruption
// const unsigned char halt_byhost_pin = 3; // entrée interruption

const unsigned char lcd_command_pin = 9;

const unsigned char shtn_operation_pin = 7;

const unsigned char halt_status_pin = 8;


/*
 * Variables et objets globaux
 */
volatile unsigned long halt_flag=0;
volatile unsigned long halt_host_flag=0;
BlinkLeds myBlinkLeds_1s(1000);
BlinkLeds myBlinkLeds_500ms(500);

unsigned long generalUpTime; // disponibilité de l'alimention
unsigned long upTimeVin1=0; // durée source 1 disponible
unsigned long upTimeVin2=0; // durée source 2 disponible
unsigned long vin1ActiveTime=0; // durée source 1 active
unsigned long vin2ActiveTime=0; // durée source 2 active


/*
 * différence "juste" entre deux chronos (tient compte d'un éventuel passage par 0 on est pas a l'abrit ...)
 */
inline unsigned long my_diff_millis(unsigned long chrono, unsigned long now)
{
  return now >= chrono ? now - chrono : 0xFFFFFFFF - chrono + now;
}


char *hruptime(unsigned long t)
{
  static char str[14];
  unsigned int j = t / 86400;
  t = t % 86400;
  unsigned char h = t / 3600;
  t = t % 3600;
  unsigned char m = t / 60;
  unsigned char s = t % 60;
  
  sprintf(str, "%d:%02d:%02d:%02d", j, h, m, s);

  return str;
}


MiniTimer::MiniTimer()
{
  previous=millis();
}


void MiniTimer::restart()
{
  previous=millis();
}


int MiniTimer::checkAndRestart(unsigned long t)
{
  unsigned long now = millis();
  if(my_diff_millis(previous, now) > t) {
    previous = now;
    return HIGH;
  } 
  else {
    return LOW;
  }
}


int MiniTimer::check(unsigned long t)
{
  if(my_diff_millis(previous, millis()) > t) {
    return HIGH;
  } 
  else {
    return LOW;
  }
}


BlinkLeds::BlinkLeds(unsigned int i)
{
  interval = i;
  ledState = LOW;
  previousMillis = 0;
}


void BlinkLeds::run()
{
  unsigned long currentMillis = millis();
  if(my_diff_millis(previousMillis, currentMillis) > interval) {
    previousMillis = currentMillis;
    ledState = !ledState;
  }
}


/*
 * Mesure la référence interne à 1v1 de l'ATmega
 */
unsigned int getInternal_1v1(void)
{
  ADMUX = 0x4E;                // Sélectionne la référence interne à 1v1 comme point de mesure, avec comme limite haute VCC
  ADCSRA |= (1 << ADEN);       // Active le convertisseur analogique -> numérique
  delay(2);                    // attendre le bon démarrage
  ADCSRA |= (1 << ADSC);       // Lance une conversion analogique -> numérique
  while(ADCSRA & (1 << ADSC)); // Attend la fin de la conversion
  return ADCL | (ADCH << 8);   // Récupère le résultat de la conversion
}


void upTimeTimerInit()
{
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 31250;            // compare match register 16MHz/256/2Hz => 1 fois par secondes
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}


volatile unsigned long upTimeInc = 0;
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  upTimeInc++;
}


/*
 * Gestionnaire d'interruption INT0/D2 : appuie sur bouton de plus de INTERVAL ms entraine demande d'arrêt de l'alimentation
 */
void halt_inter()
{
  volatile static unsigned long prev_chrono0=0; // dernière prise de chrono
  volatile static unsigned char prev_state_pin0=LOW; // état précédent de l'entrée 2

  char pin_state = PIND & (1 << 2); // avant tout, lecture de l'état INT0 (pin 2) de l'entrée sans digitalRead qui prend trop de temps
  unsigned long chrono=millis();    // maintenant qu'on connait l'état de l'entrée on peut prendre le "temps" de lire le temps actuel en millisecondes
  // notez que millis n'évolu pas pendant le déroullement de cette fonction car toutes les interruptions sont masquées 

    if(pin_state > prev_state_pin0)   // detection d'un front montant
  {
    prev_chrono0=chrono; // on prend le chrono
  }
  else if(pin_state < prev_state_pin0) // detection d'un front descendant
  {
    if(my_diff_millis(prev_chrono0, chrono) > PULSE_INTERVAL) // impulsion suffisament long ?
      halt_flag=1;
    // si le temps entre les deux fronts n'est pas suffisant, on considère que c'est du bruit ...
    // et on ne fait rien
  }
  // si pin_state == prev_state_pin0 on ne fait rien, mais cela ne devrait jamais arriver ...
  prev_state_pin0=pin_state;
}


/*
 * Traitement d'une demande d'arrêt "normal" et manuel de l'alimentation
 */
void manuel_halt()
{
  unsigned char i=GRACE_DELAY; // durée avant arret
  int led_blink=1000;
  int prev_pin_state=LOW;
  int pin_state=LOW;

  MiniTimer haltTimer;
  MiniTimer ledDisplayTimer;
#ifdef __LCD  
  MiniTimer lcdDisplayTimer;
#endif

  // à partir d'ici, arret dans 60 secondes, sauf nouvel appuie sur le bouton "Stop" => annulation
  // ou reappuie de plus de 2 secondes sur le bouton "Stop" => arret immediat
  digitalWrite(halt_status_pin, HIGH); // demande d'arrêt

  Serial.println("<HALT=MANUAL>");
#ifdef __LCD
  myLCD->backlight();
  myLCD->clear();
  myLCD->setCursor(0,0);
  myLCD->print("Arret demande :");
  myLCD->setCursor(0,1);
  myLCD->print("il reste ");
#endif

  while(1) // arrêt dans 1 minutes
  {
    // annulation de la procédure d'arrêt ?
    DISABLE_INT0_D2;
    if(halt_flag==1) 
    {
      halt_flag=0;
      digitalWrite(halt_status_pin, LOW);
      return;
    }
    ENABLE_INT0_D2;

    // arrêt immédiat ? (2 secondes sur le bouton stop
    pin_state = digitalRead(halt_command_pin);
    if(pin_state)
      if(prev_pin_state == LOW)
        haltTimer.restart();
      else
        if(haltTimer.checkAndRestart(2000))
          break;
    prev_pin_state = pin_state;

    if(haltTimer.check(30000))
      led_blink = 250;

    if(haltTimer.check(60000))
      break;

    // clignotement des leds et affichage divers
    if(ledDisplayTimer.checkAndRestart(led_blink))
    {
      int led_state = digitalRead(status_led);
      led_state = !led_state;        
      digitalWrite(status_led, led_state);
      digitalWrite(vin1_status_pin,led_state);
      digitalWrite(vin2_status_pin,led_state);
    }

#ifdef __LCD     
    if(lcdDisplayTimer.checkAndRestart(500))
    { // affichage toutes les 1/2 seconde
      i--;

      myLCD->setCursor(9,1); //Start at character 0 on line 0
      myLCD->print(i);
      myLCD->print(" s  ");
    }

    // affiche l'indicateur "bouton appuyé" en bas à droite de l'écran
    if(pin_state)
    {
      myLCD->setCursor(15,1);
      myLCD->print("*");
    }
    else
    {
      myLCD->setCursor(15,1);
      myLCD->print(" ");
    }
#endif
/*
    DISABLE_INT1_D3;
    if(halt_host_flag == 1)
      break;
    ENABLE_INT1_D3;
*/
  }

  // arret maintenant !!!
#ifdef __LCD
  myLCD->clear();
  myLCD->noBacklight();
#endif

  Serial.println("<HALT=NOW>");
  delay(5);

  digitalWrite(status_led, LOW);
  digitalWrite(vin1_status_pin, LOW);
  digitalWrite(vin2_status_pin, LOW);
  digitalWrite(vout_status_pin, LOW);
  digitalWrite(shtn_operation_pin, LOW);

  for(;;);
}


/*
 * Traitement demande d'arrêt d'urgence de l'alimentation
 */
void auto_halt()
{
#ifdef __LCD
  myLCD->backlight();
  myLCD->clear();
#endif

  DISABLE_INT0_D2;

  Serial.println("<HALT=AUTO>");
  digitalWrite(halt_status_pin, HIGH); // demande d'arrêt

  for(int i=0;i<(GRACE_DELAY*4);i++) // arrêt dans ~1 minutes (240*2*125ms)
  {
    // on attend 1 seconde en faisant clignoter "rapidement" toutes les leds
    delay(125);
    digitalWrite(status_led, HIGH);
    digitalWrite(vin1_status_pin,HIGH);
    digitalWrite(vin2_status_pin,HIGH);
    delay(125);
    digitalWrite(status_led, LOW);
    digitalWrite(vin1_status_pin,LOW);
    digitalWrite(vin2_status_pin,LOW);

#ifdef __LCD     
    myLCD->setCursor(0,0); //Start at character 0 on line 0
    myLCD->print("Arret dans ");
    myLCD->print(GRACE_DELAY-(int)(i/4)-1);
    myLCD->print(" s  ");
#endif
/*
    DISABLE_INT1_D3;
    if(halt_host_flag == 1)
    {
      delay(1000);
      break;
    }
    ENABLE_INT1_D3;
*/
  }

#ifdef __LCD
  myLCD->clear();
  myLCD->noBacklight();
#endif
  Serial.println("<HALT=NOW>");
  delay(5);

  digitalWrite(status_led, LOW);
  digitalWrite(vin1_status_pin, LOW);
  digitalWrite(vin2_status_pin, LOW);
  digitalWrite(vout_status_pin, LOW);
  digitalWrite(shtn_operation_pin, LOW);
  for(;;); // boucle sans fin ... jusqu'à plus d'alimentation électrique
}


void halt_host()
{
  // DISABLE_INT1_D3;
#ifdef __LCD
  myLCD->noBacklight();
  myLCD->clear();
#endif
  Serial.println("<HALT=HOST>");
  delay(1000);
  Serial.println("<HALT=NOW>");
  delay(5);

  digitalWrite(status_led, LOW);
  digitalWrite(vin1_status_pin, LOW);
  digitalWrite(vin2_status_pin, LOW);
  digitalWrite(vout_status_pin, LOW);
  digitalWrite(shtn_operation_pin, LOW);
  for(;;); // boucle sans fin ... jusqu'à plus d'alimentation électrique
}

/*
void host_halt_inter()
{
  halt_host_flag = 1;
}
*/

void update_vin_led_status(float vin, int vin_status_pin)
{
  int vin_led_value;
  if(vin < SEUIL_ALERT)
  {
    if(vin < SEUIL_HS)
      vin_led_value=LOW;
    else
      vin_led_value=myBlinkLeds_1s.getLedState();
  }
  else
    vin_led_value=HIGH;
  digitalWrite(vin_status_pin,vin_led_value);
}


void update_vout_led_status(float vin1, float vin2, int status_pin)
{
  if(vin1 < SEUIL_HS || vin2 < SEUIL_HS)
    digitalWrite(status_pin,myBlinkLeds_500ms.getLedState()); // clignotement rapide. 1 seule source disponible
  else if(vin1 < SEUIL_ALERT || vin2 < SEUIL_ALERT)
    digitalWrite(status_pin,myBlinkLeds_1s.getLedState());
  else
    digitalWrite(status_pin,HIGH);
}


void update_uptimes(float vin1, float vin2)
{
  static MiniTimer upTimesTimer;

  if(upTimesTimer.checkAndRestart(1000))
  {
    int t;
    DISABLE_TIMER1_COMPA;
    t=upTimeInc;
    upTimeInc=0;
    ENABLE_TIMER1_COMPA;

    generalUpTime+=t;

    if(vin1 > SEUIL_HS)
      upTimeVin1+=t;
    if(vin2 > SEUIL_HS)
      upTimeVin2+=t;

    if(vin1 > vin2)
      vin1ActiveTime+=t;
    else
      vin2ActiveTime+=t;
  }
}


void display_serial(float vcc1, float vcc2, float vout, float real_vcc, float temp)
{
  static MiniTimer serialDisplayTimer;

  if(serialDisplayTimer.checkAndRestart(5000))
  { // affichage toutes les 5 secondes
    Serial.print("<VCC=");
    Serial.print(real_vcc);
    Serial.print(";VIN1=");
    Serial.print(vcc1);
    Serial.print(";VIN2=");
    Serial.print(vcc2);
    Serial.print(";VOUT=");
    Serial.print(vout);
    Serial.print(";UTG=");
    Serial.print(generalUpTime);
    Serial.print(";UT1=");
    Serial.print(upTimeVin1);
    Serial.print(";UT2=");
    Serial.print(upTimeVin2);
    Serial.print(";AT1=");
    Serial.print(vin1ActiveTime);
    Serial.print(";AT2=");
    Serial.print(vin2ActiveTime);
    Serial.print(";T=");
    Serial.print(temp);
    Serial.print(">\n");
  }
}


#ifdef __LCD     
int lcdBackLight = 0;
void display_lcd_pages(float vcc1, float vcc2, float vout, float real_vcc, float temp)
{
  static int lcd_current_page = 1; // on commence par la première page
  static MiniTimer lcdDisplayTimer;

  int lcdDelay = 1000; // rafraichissement toutes les secondes à priori

  if(digitalRead(lcd_command_pin))
  {
    delay(PULSE_INTERVAL); // debounce du pauvre
    if(digitalRead(lcd_command_pin))
    {
      MiniTimer pageButtonDelay;

      myLCD->setCursor(15,1);
      myLCD->print("*");

      while(digitalRead(lcd_command_pin))
      {
        if(pageButtonDelay.checkAndRestart(500))
          break;
      };

      myLCD->setCursor(15,1);
      myLCD->print(" ");

      if(lcdBackLight<=LCD_MAXBACKLIGHT)
      {
        lcd_current_page++;
        if(lcd_current_page>LCD_MAXPAGES)
          lcd_current_page=1;
      }   
      lcdBackLight=0;
      lcdDelay=0;
    }
  }

  char *s;
  if(lcdDisplayTimer.checkAndRestart(lcdDelay))
  { // affichage toutes les 1 secondes ou immédiatement
    if(lcd_current_page==1) {
      myLCD->clear();
      myLCD->setCursor(0,0);
      myLCD->print("Vo=");
      myLCD->print(vout);
      myLCD->print("  ");
      myLCD->print("Vr=");
      myLCD->print(real_vcc);
      myLCD->print("V");
      s=hruptime(generalUpTime);
      myLCD->setCursor((12-strlen(s))/2,1);
      myLCD->print("upT=");
      myLCD->print(s);
    }    
    if(lcd_current_page==2) {
      myLCD->clear();
      myLCD->setCursor(0,0);
      myLCD->print("V1=");
      myLCD->print(vcc1);
      myLCD->print("  ");
      myLCD->print("V2=");
      myLCD->print(vcc2);
      myLCD->setCursor(0,1);
    }
    else if(lcd_current_page==3) {
      myLCD->clear();
      s=hruptime(upTimeVin1);
      myLCD->setCursor((12-strlen(s))/2,0);
      myLCD->print("UV1=");
      myLCD->print(s);
      s=hruptime(upTimeVin2);
      myLCD->setCursor((12-strlen(s))/2,1);
      myLCD->print("UV2=");
      myLCD->print(s);
    }
    else if(lcd_current_page==4) {
      myLCD->clear();
      s=hruptime(vin1ActiveTime);
      myLCD->setCursor((12-strlen(s))/2,0);
      myLCD->print("AV1=");
      myLCD->print(s);
      s=hruptime(vin2ActiveTime);
      myLCD->setCursor((12-strlen(s))/2,1);
      myLCD->print("AV2=");
      myLCD->print(s);
    }

    if(!lcdBackLight)
      myLCD->backlight();
    if(lcdBackLight > LCD_MAXBACKLIGHT)
    {
      myLCD->noBacklight();
      myLCD->clear();
    }
    else
      lcdBackLight++;
  }
}
#endif


void setup()
{
  pinMode(status_led, OUTPUT);
  pinMode(shtn_operation_pin, OUTPUT);

  digitalWrite(shtn_operation_pin, LOW);

  // attendre 2 secondes après allumage
  for(int i=0;i<8;i++) // 8 fois 250 ms = 2 secondes
  {
    digitalWrite(status_led, HIGH);
    delay(125);
    digitalWrite(status_led, LOW);
    delay(125);
  }
  // si on est toujours alimenté après 2 secondes (le boutton n'a pas été relaché), on "active" au plus vite l'alimentation génerale
  digitalWrite(shtn_operation_pin, HIGH);

  digitalWrite(status_led, HIGH); // on est en service

  // configuration des IO
  pinMode(vin1_input_pin, INPUT);
  pinMode(vin2_input_pin, INPUT);
  pinMode(vout_input_pin, INPUT);
  pinMode(temp_input_pin, INPUT);
  pinMode(vin1_status_pin, OUTPUT);
  pinMode(vin2_status_pin, OUTPUT);
  pinMode(vout_status_pin, OUTPUT);
  pinMode(halt_command_pin, INPUT);
//  pinMode(halt_byhost_pin, INPUT);
  pinMode(halt_status_pin, OUTPUT);
  pinMode(lcd_command_pin, INPUT);

  attachInterrupt(INT0,halt_inter, CHANGE);
/*
  attachInterrupt(INT1,host_halt_inter, RISING);
*/
  upTimeTimerInit();

  /* Initialisation du port série */
  Serial.begin(9600);

#ifdef __LCD
  /* initialisation LCD */
  Wire.begin();
  delay(10);
  myLCD->begin(16,2);
  myLCD->clear();
  myLCD->backlight();
  myLCD->setCursor(2,0);
  myLCD->print("Alimentation");
  myLCD->setCursor(2,1);
  myLCD->print("en  fonction");
  delay(1000);
#endif
}


void loop()
{
  myBlinkLeds_1s.run();
  myBlinkLeds_500ms.run();

  // récupération de la tension de référence = tension d'alimentation réel de l'ATmega
  float real_vcc = (1023.0 * 1.1) / (float)getInternal_1v1() - 0.1;

  // récupération des tensions d'entrées/sortie
  float vin1 = analogRead(vin1_input_pin)/1023.0*real_vcc*2;
  float vin2 = analogRead(vin2_input_pin)/1023.0*real_vcc*2;
  float vout = analogRead(vout_input_pin)/1023.0*real_vcc*2;

  // température du tmp36
  float temp = analogRead(temp_input_pin)/1023.0*real_vcc * 100 - 50;   // ºC = 100 * V - 50

  // controle et traitement des demandes d'arrêt
  // arrêt manuel
  DISABLE_INT0_D2;
  if(halt_flag == 1)
  {
    // remise a zero du drapeau pour pouvoir trapper le prochain appuie sur "stop"
    halt_flag=0;
    // démarrage d'un processus d'arrêt manuel
    manuel_halt();
    // si on passe ici, c’est que l’arrêt a été annulé
#ifdef __LCD  
    // on réinitialise le compteur pour l'affichage LCD
    lcdBackLight=0;
#endif
  }
  ENABLE_INT0_D2;

  // demande d'arret par l'hote
/*
  DISABLE_INT1_D3;
  if(halt_host_flag == 1)
  {
    halt_host_flag=0;
    halt_host();
  }
  ENABLE_INT1_D3;
*/
  // arrêt auto
  // vcc2 et vcc1 < SEUIL_ALERT => plus assez de jus on s'arret dans 60 secondes ...
  if(vin1 < SEUIL_ALERT && vin2 < SEUIL_ALERT)
    auto_halt(); // arret d'urgence

  // les divers signalisations
  // actvité ATmaga
  digitalWrite(status_led, myBlinkLeds_500ms.getLedState()); // clignotement de la led "activité" (D13) de l'ATmega

  // mise à jour éventuelle des leds d'état
  update_vin_led_status(vin1, vin1_status_pin);
  update_vin_led_status(vin2, vin2_status_pin);
  update_vout_led_status(vin1, vin2, vout_status_pin);

  // mise à jour des "uptimes"
  update_uptimes(vin1, vin2);

  // information éventuelle via interface serie
  display_serial(vin1, vin2, vout, real_vcc, temp);

#ifdef __LCD
  // gestion de l'affichage LCD
  display_lcd_pages(vin1, vin2, vout, real_vcc, temp);
#endif
}

