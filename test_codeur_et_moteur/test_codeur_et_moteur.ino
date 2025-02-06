// test codeur, determination des seuils de commande du moteur et identification du moteur.
// La vitesse est commandée en entrant une valeur entre  -255 et 255 à partir du moniteur série (Byte pwm)


#include <MsTimer2.h>

   
// BP modification vitesse moteur 

#define nombreInit 10000
#define pasDeNombre 9999
#define identification 20000

// Codeur incrémental
#define codeurPin1 2
#define codeurPin2 3
long impulsionsCodeur=0;
unsigned long tempsDebutComptage = 0;
unsigned long tempsFinComptage = 0;
long tempsComptage = 0;
boolean sensTrigo ;
long vitesseImpulsions = 0;
int nombreDentsCodeur = 5;
int moyennePeriodeVitesseImpulsions=0;
int moyenneVitesseCpt=0;
long totalVitesseImpulsions=0;

 
// Moteur CC
#define directionMoteurIN1  4
#define directionMoteurIN2  5
#define pwmMoteur  6
int vitesseMoteur=0 ; // pwm 0 à 255

// Cadence d'envoi des données en ms pour affichage moniteur serie
#define TSDATA 1000
unsigned long tempsDernierenvoi = 0;
unsigned long tempsCourant = 0;
int nombreRecu = 0; 
int affichage =1;
int nombrePointsRecu;

 
// Cadence d'échantillonnage en ms
int dt = 50; // en milli secondes
int x=0; // variable d'incrementation pour l'identification


// tableau de valeurs pour identification
volatile int vitesseArray[100];
volatile int codeurArray[100];


// Initialisations
void setup(void) 
{
  // Codeur incrémental
  pinMode(codeurPin1, INPUT);      // entrée digitale pin A codeur
  pinMode(codeurPin2, INPUT);      // entrée digitale pin B codeur 
  pinMode(8, OUTPUT);  // testCodeur, temporaire
  attachInterrupt(0, interruptionCodeur, RISING);


  // Moteur CC
  pinMode(directionMoteurIN1, OUTPUT);
  pinMode(directionMoteurIN2, OUTPUT);
  pinMode(pwmMoteur, OUTPUT);
 
  // Liaison série
  Serial.begin(9600);
  Serial.flush();
 
// La routine cyclique est exécutée à cadence fixe
MsTimer2::set(dt, cyclique); // période dt ms 
MsTimer2::start(); // active Timer2 
}
 
// Boucle principale
void loop()
{
// Ecriture des données sur la liaison série
nombreRecu=recevoirNombre();    // appel de la fonction recevoirNombre
if (affichage==1) ecritureData();
   if (affichage ==6) {for(int i =1; i<=nombrePointsRecu; i++) {Serial.print(i);Serial.print(";");Serial.print(vitesseArray[i]*60/nombreDentsCodeur); Serial.print(";"); Serial.println(codeurArray[i]);}affichage=7;}
if (nombreRecu!=pasDeNombre)
  {   
   if (nombreRecu==nombreInit) {Serial.println("Commande initialisee "); vitesseMoteur = 0;impulsionsCodeur=0;affichage=1;MsTimer2::set(dt, cyclique); MsTimer2::start();moyenneVitesseCpt=0; totalVitesseImpulsions=0;}
   else if ((affichage==1 || affichage==7)  && nombreRecu==20000) {Serial.println (" identification!"); Serial.println (" Entrer un nombre de points entre 1 et 99"); vitesseMoteur = 0;impulsionsCodeur=0;affichage=2;}
   else if (affichage==1 && abs(nombreRecu)>255) {Serial.println (" nombre trop grand!");affichage=1;}                              
   else if (affichage==1 && abs(nombreRecu)<=255)  { // si un nombre valide a été reçu affiche le nombre reçu sur le port série
                                               Serial.print("commande recue ="); Serial.println (nombreRecu); vitesseMoteur =  nombreRecu; affichage=1;}
   else if (affichage==2 && abs(nombreRecu)>99) {Serial.println (" nombre trop grand!");}                              
   else if (affichage==2 && abs(nombreRecu)<100) {Serial.print(" nombre de points recu = "); Serial.println (nombreRecu); nombrePointsRecu= nombreRecu; affichage=3; 
                                                    Serial.println("Entrer une periode d'acquisition  entre 10 et 100 ms ");}
   else if (affichage==3 && nombreRecu>100) {Serial.println (" nombre trop grand!");}
   else if (affichage==3 && nombreRecu<10) {Serial.println (" nombre trop petit!");}  
   else if (affichage==3 && nombreRecu>=10 && nombreRecu<=100 ) {Serial.print("Periode recue ="); Serial.println (nombreRecu); MsTimer2::set(nombreRecu, cyclique); MsTimer2::start(); affichage=4; 
                                                                    Serial.println("Entrer une commande PWM  entre 1 et 255 ");}
   else if (affichage==4 && abs(nombreRecu)>255) {Serial.println (" nombre trop grand!");} 
   else if (affichage==4 && abs(nombreRecu)<=255) {Serial.print("commande recue ="); Serial.println (nombreRecu); vitesseMoteur = nombreRecu; 
                                                   Serial.println("N Point ; vit(tr/mn) ; Codeur "); affichage=5;moyenneVitesseCpt=0; totalVitesseImpulsions=0;}
   }
}
 
void cyclique()
{
CommandeMoteur(vitesseMoteur, abs(vitesseMoteur));
if (affichage ==5 && x<nombrePointsRecu) {x++; vitesseArray[x]=moyennePeriodeVitesseImpulsions; codeurArray[x]=impulsionsCodeur; moyenneVitesseCpt=0; totalVitesseImpulsions=0;}
else if (affichage ==5 && x>=nombrePointsRecu) {x=0;  affichage=6;vitesseMoteur=0; MsTimer2::set(dt, cyclique); MsTimer2::start();}
if ((micros() - tempsFinComptage> 5000000/vitesseImpulsions)) vitesseImpulsions=0;
}

// Routine d'ecriture des données en sortie toutes les TSDATA millisecondes 
void ecritureData(void) 
{ 
   tempsCourant = millis();
  if (tempsCourant-tempsDernierenvoi >= TSDATA) {
    Serial.print("Cde (pwm): ");
    Serial.print(vitesseMoteur);
    Serial.print("; mes vit (tr/mn): ");
    Serial.print(moyennePeriodeVitesseImpulsions*60/nombreDentsCodeur); moyenneVitesseCpt=0; totalVitesseImpulsions=0;
    Serial.print("; Comptage Codeur A: ");
    Serial.print(impulsionsCodeur);
    Serial.print("; Etat Cod A: ");
    Serial.print(digitalRead(codeurPin1));
    Serial.print("; Etat Cod B: ");
    Serial.println(digitalRead(codeurPin2));
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
