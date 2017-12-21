#include <Adafruit_GFX.h>   // Core graphics library
#include <RGBmatrixPanel.h> // Hardware-specific library


#define CLK 8  // MUST be on PORTB! (Use pin 11 on Mega)
#define LAT A3
#define OE  9
#define A   A0
#define B   A1
#define C   A2
RGBmatrixPanel matrix(A, B, C, CLK, LAT, OE, false);

const int buttonPin = 13;
int ledState = HIGH;         // the current state of the output pin
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW; 
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 200; 

int beatShape[8][2];

int pos;
void setup() {
  pinMode(buttonPin, INPUT);
  pos=0;
  
  beatShape[0][0]=0;
  beatShape[0][1]=11;
  
  beatShape[1][0]=18;
  beatShape[1][1]=11;
  
  beatShape[2][0]=21;
  beatShape[2][1]=0;
  
  beatShape[3][0]=24;
  beatShape[3][1]=14;
  
  beatShape[4][0]=27;
  beatShape[4][1]=7;
  
  beatShape[5][0]=29;
  beatShape[5][1]=11;
  
  beatShape[6][0]=31;
  beatShape[6][1]=11;
  
  beatShape[7][0]=0;
  beatShape[7][1]=0;

  matrix.begin();
  
}

void drawLines(int pos){
  for(int n = 0; n< 7; n++){
    if(n < 6){
      matrix.drawLine(beatShape[n][0], beatShape[n][1], beatShape[n+1][0], beatShape[n+1][1], matrix.Color333(3, 7, 7));
    }
  }
}

int getLineN(int x){
  for(int n = 0; n< 7; n++){
    if(n < 6){
    if( beatShape[n][0] <= x && x <= beatShape[n+1][0] ){
      return n;
    }
    }
  }
}

int getY(int x){
  int n = getLineN(x);
  int x0 = beatShape[n][0];
  int y0 = beatShape[n][1];
  int x1 = beatShape[n+1][0];
  int y1 = beatShape[n+1][1];
  
  float f = (y1 - y0)* 1.0 / (x1 - x0);
  
  if(f==0.0){
    return y0;
  }else{
    int Y = y0 + (f * (x - x0));
    return Y;
  }
  
}


int rate = 1;
int frameWidth = 10;
int s0,f0,s1,f1;

void drawBeep(int x){
  int y = getY(x);
  matrix.fillRect(x-1, y-1, 3, 3, matrix.Color333(7, 7, 7));
}

void drawBeepStatic(int x){
  int y = 11;
  matrix.fillRect(x-1, y-1, 3, 3, matrix.Color333(7, 7, 7));
}

void drawFrame(int pos){
  s0 = pos;
  if(pos+frameWidth> 31){
    f0 = 31;
    s1 = 0;
    f1 = 31 - pos;
    
    matrix.fillRect(s0, 0, f0, 16, matrix.Color333(0, 0, 0));
    matrix.fillRect(s1, 0, f1, 16, matrix.Color333(0, 0, 0));
    
  }else{
    s1 = f1  = 0;
    f0 = pos+frameWidth;
    matrix.fillRect(s0, 0, f0, 16, matrix.Color333(0, 0, 0));
  }
}

void drawWholeLine(int n){
  matrix.drawLine(beatShape[n][0], beatShape[n][1], beatShape[n+1][0], beatShape[n+1][1], matrix.Color333(3, 7, 7));
}

void drawFrom(int pos, int n){
  int y = getY(pos);
  matrix.drawLine(pos, y, beatShape[n+1][0], beatShape[n+1][1], matrix.Color333(3, 7, 7));
}

void drawTo(int pos,int n){
  int y = getY(pos);
  matrix.drawLine(beatShape[n][0], beatShape[n][1],pos, y,  matrix.Color333(3, 7, 7));
}

void drawLinesV2(int pos0){
  int s0 = pos;
  if(pos0+frameWidth> 31){
    int pos1 = frameWidth - 31 + pos;
    int n0 = getLineN(pos0);
    int n1 = getLineN(pos1);
    drawFrom(pos0,n0);
    drawTo(pos1,n1);
  }else{
    int pos1 = frameWidth + pos;
    int n0 = getLineN(pos0);
    int n1 = getLineN(pos1);
    drawFrom(pos0,n0);
    drawTo(pos1,n1);
  }
}

void loop() {

  pos+=rate;
  
  if(pos > 31){
    pos-=31;
  }
  
  buttonState = digitalRead(buttonPin);
  if(buttonState){
    matrix.drawLine(0, 11, 31, 11, matrix.Color333(7, 3, 3));
    drawBeepStatic(pos);
    delay(30);
    matrix.fillScreen(matrix.Color333(0, 0, 0));
  }else{
    int yy = getLineN(pos);
    drawLines(pos);
    drawFrame(pos);
    //Serial.println(pos);
    Serial.println(yy);
    drawBeep(pos);
    delay(10);
    getY(pos);
    //fill the screen with 'black'
    matrix.fillScreen(matrix.Color333(0, 0, 0));
  }
}
