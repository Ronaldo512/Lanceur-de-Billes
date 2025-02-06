// regulation de vitesse, determination des paramètres du corecteur d'une regulation de vitesse
// La vitesse est commandée en entrant une valeur entre  -3500 et 3500 à partir du moniteur série (tr/mn)
// Une commande 10000 initialise le système


#include <MsTimer2.h>
   
// BP modification vitesse moteur 

#define nombreInit 10000
#define pasDeNombre 20000

// Codeur incrémental
#define codeurInterruption 0
#define codeurPin1 2
#define codeurPin2 3
long impulsionsCodeur = 0;
#define nombreDentsCodeur 5
boolean sensTrigo=LOW;
int moyennePeriodeVitesseImpulsions=0;
int moyenneVitesseCpt=0;
long totalVitesseImpulsions=0;
 
// Moteur CC (Module moteur alimenté en 9V)
#define directionMoteurIN1  4
#define directionMoteurIN2  5
#define pwmMoteur  6


// PID vitesse
float KpV = 0.1; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Valeur a regler
float  KiV = 0.1; // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! Valeur a regler
float  actionPV;
float  actionIV = 0;
long ecartV = 0;
long commandeVpwm;
volatile long  consV =0;
long  mesV;
long consVMin=450;
long consVSeuil=3000;
long commandeVpwmMax=255; //Pour un module moteur alimenté en 9V

// envoi des données en ms pour affichage moniteur serie
#define TSDATA 1000
unsigned long tempsDernierenvoi = 0;
unsigned long tempsCourant = 0;
long nombreRecu=0;
 
// Cadence d'échantillonnage en ms
float dt = 0.02;  

// vitesse d'impulsion  
unsigned long tempsDebutComptage = 0;
unsigned long tempsFinComptage = 0;
long tempsComptage = 0;
volatile long vitesseImpulsions = 0;


// Initialisations
void setup(void) 
{
  // Codeur incrémental
  pinMode(codeurPin1, INPUT);      // entrée digitale pin A codeur
  pinMode(codeurPin2, INPUT);  // pour le sens de rotation;
  attachInterrupt(codeurInterruption, interruptionCodeur, RISING); 
 //Boutons poussoir 

  // Moteur CC
  pinMode(directionMoteurIN1, OUTPUT);
  pinMode(directionMoteurIN2, OUTPUT);
  pinMode(pwmMoteur, OUTPUT); 
  // Liaison série
  Serial.begin(9600);
  Serial.flush(); 
  // La routine cyclique est exécutée à cadence fixe
MsTimer2::set(dt*1000, cyclique); // période dt ms 
MsTimer2::start(); // active Timer2 
}
 
// Boucle principale
void loop()
{
 // Calcul du gain du groupe Moteur

// Ecriture des données sur la liaison série
ecritureData(); 
nombreRecu=recevoirNombre(); // appel de la fonction recevoirNombre
if (nombreRecu!=pasDeNombre) { if (nombreRecu == nombreInit) { Serial.println("Commande initialisee "); consV = 0; actionIV=0; impulsionsCodeur =0; vitesseImpulsions=0; moyenneVitesseCpt=0; totalVitesseImpulsions=0;}
                         else if (abs(nombreRecu)>consVSeuil) Serial.println (" nombre trop grand!");
                         else  { // si un nombre valide a été reçu affiche le nombre reçu sur le port série
                         Serial.print("commande recue =");
                         Serial.println (nombreRecu);
                         consV =  nombreRecu;}
                       }
}
 
void cyclique()
{ 
  
if ((micros() - tempsFinComptage> 5000000/vitesseImpulsions)&&(consV==0)) vitesseImpulsions=0;  
/******* Régulation PID vitesse ********/

    // Ecart de vitesse de rotation moteur (en tr/mn) entre la consigne et la mesure 
    mesV = (moyennePeriodeVitesseImpulsions * 60/nombreDentsCodeur);moyenneVitesseCpt=0; totalVitesseImpulsions=0;
    ecartV = consV - mesV; 
    // Terme proportionnel
    actionPV = KpV * ecartV; 
     // Calcul de la commande. 
    commandeVpwm = actionPV + actionIV;
    if (consV==0) {commandeVpwm=0; vitesseImpulsions=0; actionIV =0;}
    // Terme intégral calculé avec une période de retard
    actionIV = actionIV + KiV*dt*ecartV;
    
  if (commandeVpwm>commandeVpwmMax) {commandeVpwm = commandeVpwmMax; actionIV= actionIV - KiV*dt*ecartV;};
  if (commandeVpwm<-commandeVpwmMax) {commandeVpwm = -commandeVpwmMax; actionIV= actionIV - KiV*dt*ecartV;};
  
  
  CommandeMoteur(commandeVpwm, abs(commandeVpwm));

}

// Routine d'ecriture des données en sortie toutes les TSDATA millisecondes 
void ecritureData(void) 
{ 
  tempsCourant = millis();
  if (tempsCourant-tempsDernierenvoi >= TSDATA) 
  {
    Serial.print("Cons V: ");    Serial.print(consV);
    Serial.print("; mesure V: ");    Serial.print(mesV);
    Serial.print("; action P: ");    Serial.print(actionPV);
    Serial.print("; action I: ");    Serial.print(actionIV);
    Serial.print("; codeur: ");    Serial.print(impulsionsCodeur);
    Serial.print("; Commande PWM: ");    Serial.println(commandeVpwm);
    tempsDernierenvoi = tempsCourant;
  } 
}

// fonction de reception d'un nombre sur le port série
long recevoirNombre() 
{   
  int octetRecu=0; // variable pour octet recu
  boolean signe=true; // variable locale signe nombre recu
  int compt = 0;
  int nombre =0;
 if (Serial.available()==0) return(pasDeNombre);
    else {while (Serial.available()>0) 
           { // tant qu'un octet en réception
           octetRecu=Serial.read(); // Lit le 1er octet reçu et le met dans la variable
           if ((octetRecu=='-') && (compt==0))signe=false; // si Octet reçu est le - et si c'est le 1er caractère reçu - signe négatif
           compt=compt+1; // incrémente compt
           octetRecu=octetRecu-48; // transfo valeur ASCII en valeur décimale        
           // calcul du nombre à partir des valeurs reçues  
           if ((octetRecu>=0)&&(octetRecu<=9)) nombre = (nombre*10)+octetRecu;
           delay(2); // pause pour laisser le temps à la fonction available de recevoir octet suivant
           } // fin tant que octet réception
         if (signe==false) nombre=nombre*(-1); // prise en compte signe négatif
         return(nombre); // renvoie le nombre calculé 
        } 
} // fin fonction recevoirNombree

// Routine de commande moteur 
void CommandeMoteur(int sens, int vitesse)
{
   if (sens>0) {digitalWrite(directionMoteurIN1, HIGH);
                         digitalWrite(directionMoteurIN2, LOW);
                         analogWrite(pwmMoteur, vitesse);}
   if (sens<0) {digitalWrite(directionMoteurIN1, LOW);
                         digitalWrite(directionMoteurIN2, HIGH);
                         analogWrite(pwmMoteur, vitesse);}
   if (sens==0){digitalWrite(directionMoteurIN1, LOW);
                         digitalWrite(directionMoteurIN2, LOW);
                         analogWrite(pwmMoteur, 0);}
}
 
// Routine d'interruption du codeur incrémental
void interruptionCodeur()
{  
    tempsFinComptage=micros();
    tempsComptage = tempsFinComptage-tempsDebutComptage;    
    tempsDebutComptage = tempsFinComptage;
     if (tempsComptage > 1000) // Elimine les impulsions dues à des rebonds 
    {
    sensTrigo = digitalRead(codeurPin2);
    if (sensTrigo==HIGH) {impulsionsCodeur--; vitesseImpulsions = -1000000/tempsComptage;} //sens trigo
    if (sensTrigo==LOW) {impulsionsCodeur++; vitesseImpulsions = 1000000/tempsComptage;}   //sens horloge
    moyenneVitesseCpt++; totalVitesseImpulsions=totalVitesseImpulsions+vitesseImpulsions; moyennePeriodeVitesseImpulsions=totalVitesseImpulsions/moyenneVitesseCpt;
    }   
   
}
 

