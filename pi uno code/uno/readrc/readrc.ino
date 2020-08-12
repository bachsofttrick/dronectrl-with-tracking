///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you 
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Declaring global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5;
unsigned long loop_timer;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, timer_channel_5, esc_loop_timer;
int receiver_input_1, receiver_input_2, receiver_input_3, receiver_input_4, receiver_input_5;
unsigned long timer_1, timer_2, timer_3, timer_4, timer_5, current_time;
int center = 1500;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  DDRB |= B00100000;                                                        //Configure digital port 13 as output.
  DDRD |= B11110000;                                                        //Configure digital port 4, 5, 6 and 7 as output.
  //Use the led on the Arduino for startup indication.
  PORTB |= B00100000;                                                       //Turn on the warning led.
  
  Serial.begin(9600);                                                       //Setup Serial to read data
  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT4);                                                  //Set PCINT4 (digital input 12)to trigger an interrupt on state change.

  //When everything is done, turn off the led.
  PORTB &= B11011111;                                                       //Turn off the warning led.
  loop_timer = micros();                                                    //Set the timer for the next loop.
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  //The refresh rate is 100Hz. That means the esc's need there pulse every 10ms.
  while(micros() - loop_timer < 10000);                                     //We wait until 10000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = receiver_input_1 + loop_timer;                          //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = receiver_input_2 + loop_timer;                          //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = receiver_input_3 + loop_timer;                          //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = receiver_input_4 + loop_timer;                          //Calculate the time of the faling edge of the esc-4 pulse.
  
  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer) PORTD &= B11101111;               //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer) PORTD &= B11011111;               //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer) PORTD &= B10111111;               //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer) PORTD &= B01111111;               //Set digital output 7 to low if the time is expired.
  }
  read_signal();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 8, 9, 10, 11 or 12 changed state. This is used to read the receiver signals. 
//More information about this subroutine can be found in this video:
//https://youtu.be/bENjl1KQbvo
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input_1 = current_time - timer_1;                              //Channel 1 is current_time - timer_1.
    if(receiver_input_1 > 2500) receiver_input_1 = 0;
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                    //Is input 9 high?
    if(last_channel_2 == 0){                                                //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input_2 = current_time - timer_2;                              //Channel 2 is current_time - timer_2.
    if(receiver_input_2 > 2500) receiver_input_2 = 0;
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(last_channel_3 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input_3 = current_time - timer_3;                              //Channel 3 is current_time - timer_3.
    if(receiver_input_3 > 2500) receiver_input_3 = 0;
  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                                    //Is input 11 high?
    if(last_channel_4 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input_4 = current_time - timer_4;                              //Channel 4 is current_time - timer_4.
    if(receiver_input_4 > 2500) receiver_input_4 = 0;
  }
  //Channel 5=========================================
  if(PINB & B00010000 ){                                                    //Is input 12 high?
    if(last_channel_5 == 0){                                                //Input 12 changed from 0 to 1.
      last_channel_5 = 1;                                                   //Remember current input state.
      timer_5 = current_time;                                               //Set timer_5 to current_time.
    }
  }
  else if(last_channel_5 == 1){                                             //Input 12 is not high and changed from 1 to 0.
    last_channel_5 = 0;                                                     //Remember current input state.
    receiver_input_5 = current_time - timer_5;                              //Channel 5 is current_time - timer_5.
    if(receiver_input_5 > 2500) receiver_input_5 = 0;
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for reading signal
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void read_signal(){
    Serial.print("8:");
    if(receiver_input_1 - (center - 20) < 0) Serial.print("vvv");
    else if(receiver_input_1 - (center + 20) > 0) Serial.print("^^^");
    else Serial.print("-+-");
    Serial.print(receiver_input_1);
    
    Serial.print(" 9:");
    if(receiver_input_2 - (center - 20) < 0) Serial.print(">>>");
    else if(receiver_input_2 - (center + 20) > 0) Serial.print("<<<");
    else Serial.print("-+-");
    Serial.print(receiver_input_2);
    
    Serial.print(" 10:");
    if(receiver_input_3 - (center - 20) < 0) Serial.print("vvv");
    else if(receiver_input_3 - (center + 20) > 0) Serial.print("^^^");
    else Serial.print("-+-");
    Serial.print(receiver_input_3);
    
    Serial.print(" 11:");
    if(receiver_input_4 - (center - 20) < 0) Serial.print(">>>");
    else if(receiver_input_4 - (center + 20) > 0) Serial.print("<<<");
    else Serial.print("-+-");
    Serial.print(receiver_input_4);

    Serial.print(" 12:");
    if(receiver_input_5 - (center - 20) < 0) Serial.print("vvv");
    else if(receiver_input_5 - (center + 20) > 0) Serial.print("^^^");
    else Serial.print("-+-");
    Serial.println(receiver_input_5);
}
