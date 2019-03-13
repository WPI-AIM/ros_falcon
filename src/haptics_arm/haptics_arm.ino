 
 #define DEBUG 1
 


 #define BUFFER_SIZE 64
 uint16_t serialBufferPos;
 char buffer[BUFFER_SIZE]; //this is the buffer where we store incoming text from the computer

 void setup() {
    Serial.begin(115200);
    serialBufferPos = 0;
    counter = 0;
    
    //configure the PCA
    Wire.begin(); // Initiate the Wire library

    /* pca.init(); */

    /* pca.thrustersOff(); */

    alive = millis();
    delay(1);
}


void loop() {

  //this is all the stuff we do while the jetson is talking to us
  if (Serial.available() > 0) {


    // Read next byte from serial into buffer
    buffer[serialBufferPos] = Serial.read();

    #ifdef DEBUG
     Serial.print("buffer is: ");
     Serial.println(buffer);
    #endif

    // Check if we've reached exclamation
    if (buffer[serialBufferPos] == '!') {
          Serial.println("Got your command");
          Serial.println(buffer[0]);
          Serial.println(buffer[1]);
          Serial.println(buffer[2]);          
          // Reset buffer position
          serialBufferPos = 0;
          buffer[0] = 0;

    } else {
      #ifdef DEBUG

      Serial.print("Buffer pos ");
      Serial.println(serialBufferPos);
      Serial.println(buffer[serialBufferPos]);
      
      #endif
      serialBufferPos++;
    }
    
  }

    // update the timer
}//end

