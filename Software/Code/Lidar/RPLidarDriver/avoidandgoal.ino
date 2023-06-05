/*
Fonctionne pour arduino DUE et MEGA
 */
int Max_Turn_Speed = 10;//10 cm/s
int Robot_largeur=40;// 40 cm
int Marge_Securite=10;//10 cm
int Turn_Intensity =1.7;//
int Max_Speed=20;// vitesse maximale
int Min_Impact_Time=1; // 1 seconde
int Turn_Resistance=2;// permet de pondérer le chemin le plus droit
int ExtraMarge=1;//1 cm

typedef struct 
  {
      float distance;
      int angle;
      
  } Progres;

typedef struct 
  {
      int vitesse;
      float vcourbe;
      
  } Record;

Record DrivePair;
Progres Prog;
unsigned char data[20];
const int DesiredRPM=300;  // Setting Desired RPM Here.
const int MotorPWMPin=4;
int inByte = 0;         // incoming serial byte
unsigned char Data_status=0;
unsigned char Data_4deg_index=0;
unsigned char Data_loop_index=0;
unsigned char SpeedRPHhighbyte=0; // 
unsigned char SpeedRPHLowbyte=0;

int Start,End;
int Mesures[360];//ensemble des mesures du lidar

int SpeedRPH=0;
const unsigned char PWM4dutyMax=255;
const unsigned char PWM4dutyMin=170;
unsigned char PWM4duty=PWM4dutyMin;  // have to set a default value make motor start spining

 void setup() {
    pinMode(MotorPWMPin, OUTPUT); 
    Serial.begin(115200);  // USB serial
    Serial3.begin(115200);  // XV-11 LDS data 

  // prints title with ending line break 
  Serial.println("Arduino Neato XV-11 Motor control board v0.1 by Cheng-Lung Lee"); 
  
  // Pick your magic number and drive your motor , 178 is 178/255*5V=3.49V
    analogWrite(MotorPWMPin, PWM4duty );  
}

void loop() {
    // if we get a valid byte from LDS, read it and send it to USB-serial
  if (Serial3.available() > 0) {
    // get incoming byte:
    inByte = Serial3.read();
   // Serial.write(inByte);// uniquement si on veut visualiser les données LIDAR avec le code python
    decodeData(inByte);
    
    if (End) {// On a récupéré 360 données LIDAR
     PILOTE(-20,2000,1);
     Serial.println("DrivePair :");
     Serial.println(DrivePair.vitesse);
     Serial.println(DrivePair.vcourbe);
     Serial.println("Prog :");
     Serial.println(Prog.distance);
     Serial.println(Prog.angle);
     Start=0;
     End=0;
    }
  }

}

void decodeData(unsigned char inByte){
  switch (Data_status){
  case 0: // no header
    if (inByte==0xFA){
     Data_status=1;
     data[0]=inByte;
     Data_loop_index=1;
    }
    break;
  case 1: // Find 2nd FA
    if (Data_loop_index==22){
      if (inByte==0xFA)
      {
        Data_status=2;
        data[0]=inByte;
        Data_loop_index=1;
      } 
      else // if not FA search again
      Data_status=0;
    }
    else{
      Data_loop_index++;
    }
    
    break;
  case 2: // Read data out
     if (Data_loop_index==22){
      if (inByte==0xFA)
      {
        Data_loop_index=1;
        data[0]=inByte;
      } 
      else // if not FA search again
      Data_status=0;
    }
    else{
      readData(inByte);
      Data_loop_index++;
    }
    
    break;
  }
  
}

int checksum()
{
    int data_list[10]={0,0,0,0,0,0,0,0,0,0};
   
    for (int t=0;t<10;t++)
     data_list[t]=( data[2*t] + (data[2*t+1]<<8) );
     
    // compute the checksum on 32 bits
    long chk32 = 0;
    for (int d=0 ; d<10;d++)
       chk32 = (chk32 << 1) + data_list[d];
        
    // return a value wrapped around on 15bits, and truncated to still fit into 15 bits
    long checksum = (chk32 & 0x7FFF) + ( chk32 >> 15 ); // wrap around to fit into 15 bits
    int checksum1 = checksum & 0x7FFF; // truncate to 15 bits
    return ( checksum1 );
}

int x,x1;
int Check;
void readData(unsigned char inByte){
int temp;
  if (Data_loop_index<20)
   data[Data_loop_index]=inByte;
  else{
    if (Data_loop_index==20) 
     x = inByte;
    else{
     x1 = inByte;
     Check = x | ( x1  << 8);
     if (Check==checksum()){
      Data_4deg_index=data[1]-0xA0;
      if (Data_4deg_index==0)
       Start=1;
      if (Start)
       if (Data_4deg_index==89)
        End=1;
      SpeedRPHLowbyte=data[2];
      SpeedRPHhighbyte=data[3];
      SpeedRPH=(SpeedRPHhighbyte<<8)|SpeedRPHLowbyte; 
      SpeedControl ( DesiredRPM ) ;    
      if ((data[5] & 0x80)!=1)//si la donnée est bonne
       Mesures[4*Data_4deg_index]= data[4] | (( data[5] & 0x3f) << 8);
      if ((data[9] & 0x80)!=1)//si la donnée est bonne
       Mesures[4*Data_4deg_index+1]=data[8] | (( data[9] & 0x3f) << 8);
      if ((data[13] & 0x80)!=1)//si la donnée est bonne
       Mesures[4*Data_4deg_index+2]=data[12] | (( data[13] & 0x3f) << 8);
      if ((data[17] & 0x80)!=1)//si la donnée est bonne
       Mesures[4*Data_4deg_index+3]=data[16] | (( data[17] & 0x3f) << 8);
     }
   }
 }
  
}


// Very simple speed control
void SpeedControl ( int RPMinput)
{
 if (Data_4deg_index%30==0) {  // I only do 3 updat I feel it is good enough for now
  if (SpeedRPH<RPMinput*60)
     if (PWM4duty<PWM4dutyMax) PWM4duty++; // limit the max PWM make sure it don't overflow and make LDS stop working
  if (SpeedRPH>RPMinput*60)
     if(PWM4duty>PWM4dutyMin) PWM4duty--;  //Have to limit the lowest pwm keep motor running
  }     
  analogWrite(MotorPWMPin, PWM4duty ); // update value
}

/*********************************************************************************************/
/************************* Evitement D'obstacle et recherche de But **************************/
/*********************************************************************************************/


float DistanceInter(int largeur, float alpha, float theta)
{
  float temp;
  float alpha_rad=alpha * PI / 180.0;
  float theta_rad=theta * PI / 180.0;

  if (theta > alpha){
   alpha_rad = -1*alpha_rad;
   theta_rad = -1*theta_rad ;
  }
  temp=((largeur/2)*((sin(PI/2-alpha_rad))/(sin(alpha_rad - theta_rad))));
  return temp;  
  
}

float Min_Dist(int largeur,int Ang_gauche,int Ang_droit)
{
  float distance=0;
  float distance2=100000;
  float temp;
  int theta;
  for (int thetap=-88; thetap <= 88; thetap++){
    theta=(thetap<0)? 360+thetap : thetap;
   
   if (thetap < Ang_droit){
     temp=DistanceInter(largeur,Ang_droit,thetap);
    if (Mesures[theta]<temp)
      distance=Mesures[theta];
    else
      distance=10000;
   }
   else
    if ((thetap > Ang_droit) && (thetap < Ang_gauche))
      distance=Mesures[theta];
    else
     if (thetap > Ang_gauche)
      if(Mesures[theta]<DistanceInter(largeur,Ang_gauche,thetap))
        distance=Mesures[theta];
      else
        distance=10000;
  
   if ((distance2>distance))
        distance2=distance;    
   
  }
  
  return distance2;
}
 
float VitesseCourbe (int Angle)
{ 
  float angle_rad;
  int signe=1;
  if( Angle <0)
   signe=-1;
  angle_rad=Angle * PI / 180.0;
  float temp =pow(abs(angle_rad/PI)*2,1/Turn_Intensity);
  return (signe * Max_Turn_Speed * temp);
}

void EnAvant(int AngBut,float DistBut,int FlagBut)
{
  int W = Robot_largeur + Marge_Securite;
  int alpha = Direction(W,AngBut,DistBut);
  DrivePair.vitesse = Vitesse_Avant(W,alpha,DistBut,FlagBut);
  DrivePair.vcourbe= VitesseCourbe(alpha);
}

// Calcul de la vitesse de déplacement
float Vitesse_Avant(int largeur,int AngBut,float DistBut, int FlagBut)
{
  int angle_gauche=max(0,AngBut);
  int angle_droit=min(0,AngBut);
  float Dist_Secure=Min_Dist(largeur,angle_gauche,angle_droit)-2*Marge_Securite;
  float distance=0;
  
  if(FlagBut)
   distance=min(DistBut,Dist_Secure);
  else
   distance=Dist_Secure;

  float vitesse = min(Max_Speed,(distance/Min_Impact_Time));
  return vitesse;  
}

//Calcul de la plus grande distance de déplacement
float Progression(int largeur,int AngBut,float DistBut, int alpha)
{
  float temp;
  float alpha_rad,AngBut_rad,temp1;
  AngBut_rad = AngBut * PI / 180.0;
  alpha_rad = alpha * PI / 180.0;
  
  temp =min(DistBut, Min_Dist(largeur+ExtraMarge,alpha,alpha));
  temp1= pow(cos(abs(AngBut_rad-alpha_rad)),Turn_Resistance);

  return (temp*temp1);
}

// calcul la meilleur direction de déplacement
int Direction(int largeur,int AngBut,float DistBut)
{
  float temp;
  Prog.distance=-1;
  for (int alpha=-45; alpha <= 45; alpha++){
   temp=Progression(largeur,AngBut,DistBut,alpha);
  
   if (temp>Prog.distance){
    Prog.distance=temp;
    Prog.angle=alpha;

   }
  }
  return Prog.angle;
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
// Calcul la vitesse de rotation du robot vers le but
void Orientation(int AngBut)
{
  int signe=1;
  DrivePair.vitesse=0;
  if (AngBut<0) 
   signe=-1;
  DrivePair.vcourbe= signe*Max_Turn_Speed;
}
 
// Calcul la direction et la vitesse de déplacement
void PILOTE(int AngBut,float DistBut, int FlagBut)
{
  if (abs(AngBut)<(90))
   EnAvant(AngBut,DistBut,FlagBut);
  else
   Orientation(AngBut);
}

