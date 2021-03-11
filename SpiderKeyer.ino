/*  Spider Keyer (version 1.xx) by Petr OK1FIG, Dec 2018, May 2019, Mar 2021
    This code was developed for Arduino Nano. Other Arduinos were not tested.
    The goal is to work seamlessly with HamRacer contesting software:
    http://ok1fig.nagano.cz/HamRacer/HamRacer.htm
    For more details on the Spider Keyer go here:
    http://ok1fig.nagano.cz/SpiderKeyer/SpiderKeyer.htm
*/

#include <EEPROM.h>
#define VERSION "1.36" // Mar 2021

// Pin assigment. This complies to HW: http://ok1fig.nagano.cz/SpiderKeyer/SpiderKeyer.htm
#define PADDLE_LEFT 2  // Must be 2 or 3 because of interrupts
#define PADDLE_RIGHT 3 // Must be 2 or 3 because of interrupts
#define PTT_TX_1 9  // PTT
#define TX_KEY_LINE_1 10 // Keying
#define SIDETONE_LINE 11 // Buzzer
#define POTENTIOMETER A0  // Speed potentiometer (0 to 5 V)

#define BUTTON 6 // the button to send message or tune

#define MY_HEADER 43690 // Binary 1010 1010 1010 1010

#define POT_CHANGE_THRESHOLD 0.9
#define POT_CHECK_INTERVAL 150
// For other values look into set_defaults()


unsigned int wpm;  // CW speed
unsigned int manual_wpm_limit;  // Speed limit for manual sending, in wpm. 0: switched off
unsigned int wpm_dif; // Difference introduced by a buffered speed change command
unsigned int wpm_dif_manual; // Difference to get slower manual sending
byte keyer_mode;   // IAMBIC A, B
unsigned int dah_to_dit_ratio;
byte length_wordspace;
byte weighting;
unsigned int sidetone_automatic; // In Hz
unsigned int sidetone_manual; // In Hz
byte sending_mode; // MANUAL_SENDING, AUTOMATIC_SENDING
byte last_sending_mode;
unsigned int ptt_tail_time;
unsigned int ptt_lead_time;
byte dit_buffer;
byte dah_buffer;
byte being_sent;     // SENDING_NOTHING, SENDING_DIT, SENDING_DAH
byte key_state;      // 0 = key up, 1 = key down
byte ptt_state;     // 0: RX, 1: TX
unsigned long ptt_time;
byte length_letterspace;
float ptt_hang_time_wordspace_units;
byte iambic_flag;
byte pot_wpm_low_value;
byte pot_wpm_high_value;
byte last_pot_wpm_read;
unsigned long last_pot_check_time;
int pot_full_scale_reading;
byte incoming_serial_byte;
byte esc_char_rcvd_command;
byte send_buffer[256]; // Standard buffer for online transmitting. Must be exatly 256 as indexes are overflowing bytes
byte command_buf[2]; // Immediate command
bool paddles_trigger_ptt; // Paddles are allowed to trigger PTT, or not
bool ptt_forced_on; // Low-level PTT down (via immy command)
byte key_forced_down; // Low-level key down (via immy command)  0: key up; 1: key down; 2 : key+ptt down
bool speed_set_by_pot;
// indexes to circular buffer:
byte put_at; // Points to buffer where to store next char
byte get_at; // Poinst to buffer from where to get the next char
bool send_feedback;
byte enabled_features; // bits: 0 (val 1): ptt, 1 (val 2): key, 2 (val 4): speed potentiometer, ..., ... 7: ...
bool paddles_touched;
bool paddles_swapped; // For left-handed OPs and for OPs from Ricany u Prahy
long btn_toggle; // Debounce the message sending button
volatile bool interrupt_sending;
byte response_needed;

// The commands that can be sent from PC, either buffered or immediate:
#define CMD_FIRST 1
#define CMD_SET_PTT 1 // Toggle PTT
#define CMD_SET_KEY 2 // Toggle Key
#define CMD_SPEED_CHANGE 3
#define CMD_SET_LEAD_TIME 4
#define CMD_SET_TAIL_TIME 5
#define CMD_SET_HANG_TIME 6
#define CMD_SET_WEIGHTING 7
#define CMD_ENABLE_FEATURES 8 // Bitwise!
#define CMD_SET_PADDLES_TRIGGER_PTT 9
#define CMD_SET_SIDETONE_AUTOMATIC 10
#define CMD_SET_SIDETONE_MANUAL 11
#define CMD_SET_IAMBIC_MODE 12
// #define CMD_RESERVED 13
#define CMD_BREAK_IMMY 14
#define CMD_RESET 15
#define CMD_PING 16
#define CMD_GET_SIGNATURE 17
#define CMD_BEEP 18
#define CMD_SET_FEEDBACK 19
#define CMD_SET_LOW_LIMIT 20
#define CMD_SET_HIGH_LIMIT 21
#define CMD_SET_MANUAL_SENDING_LIMIT 22
#define CMD_SET_PADDLES_SWAPPED 23
#define CMD_SAVE_CONFIG 24
#define CMD_STORE_MSG 25
#define CMD_LAST 25

// Every immediate command must preceeded with this char:
#define ESCAPE_CHAR 27
// Values used internally:
#define IAMBIC_A 1
#define IAMBIC_B 2
#define SENDING_NOTHING 0
#define SENDING_DIT 1
#define SENDING_DAH 2
#define UNDEFINED_SENDING 0
#define AUTOMATIC_SENDING 1
#define MANUAL_SENDING 2

// --------------------------------------------------------------------------------------------
void setup() {
  pinMode (PADDLE_LEFT, INPUT_PULLUP);
  pinMode (PADDLE_RIGHT, INPUT_PULLUP);
  pinMode (TX_KEY_LINE_1 , OUTPUT);

  pinMode (BUTTON, INPUT_PULLUP);

  digitalWrite (TX_KEY_LINE_1, LOW);
  pinMode (PTT_TX_1, OUTPUT);
  digitalWrite (PTT_TX_1, LOW);
  pinMode (SIDETONE_LINE, OUTPUT);
  digitalWrite (SIDETONE_LINE, LOW);
  pinMode(POTENTIOMETER, INPUT);

  set_defaults();
  reset_com();

  Serial.begin(57600); // primary_serial_port_baud_rate

  attachInterrupt(digitalPinToInterrupt(PADDLE_LEFT), paddle_down, FALLING);
  attachInterrupt(digitalPinToInterrupt(PADDLE_RIGHT), paddle_down, FALLING);


  // Short low-pitch beep to indicate Arduino Nano was started/restarted:
  tone(SIDETONE_LINE, 50);
  delay(30);
  noTone(SIDETONE_LINE);
}

// --------------------------------------------------------------------------------------------
void loop() {
  check_interruption();
  check_dit_paddle();
  check_dah_paddle();
  service_dit_dah_buffers();
  check_serial();
  service_dit_dah_buffers();
  service_send_buffer();
  check_ptt_tail();
  check_potentiometer();
  check_button();

}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
void reset_com() {
  /* Makes a total reset, puts all to inactive and resets the status flags. Should be called
    in need to fetch the device to a "defined state". */
  digitalWrite (TX_KEY_LINE_1, LOW);
  digitalWrite (PTT_TX_1, LOW);
  noTone(SIDETONE_LINE);
  esc_char_rcvd_command = 0;
  ptt_forced_on = false;
  key_forced_down = 0;
  put_at = 0;
  get_at = 0;
  dit_buffer = 0;     // Used for buffering paddle hits in iambic operation
  dah_buffer = 0;     // Used for buffering paddle hits in iambic operation
  being_sent = 0;     // One of: SENDING_NOTHING, SENDING_DIT, SENDING_DAH
  key_state = 0;      // 0 = key up, 1 = key down
  ptt_time = 0;       // Time when PTT was set on
  ptt_state = 0;
  iambic_flag = 0;
  wpm_dif = 0;
  wpm_dif_manual = 0;
  paddles_touched = false;
  sending_mode = MANUAL_SENDING;
  last_sending_mode = MANUAL_SENDING;
  interrupt_sending = false;
  response_needed = false;
}

// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void set_defaults() {
  keyer_mode = IAMBIC_B;
  sidetone_automatic = 750; // Hz
  sidetone_manual = 750; // Hz
  ptt_tail_time = 5;
  ptt_lead_time = 30; // mSec
  length_letterspace = 3; // dash to dot is 3:1   //default_length_letterspace;
  ptt_hang_time_wordspace_units = 0.9;
//  last_sending_mode = MANUAL_SENDING;
  pot_wpm_low_value = 15;
  pot_wpm_high_value = 40;
  last_pot_wpm_read = 0;
  last_pot_check_time = 0;
  pot_full_scale_reading = 1023; //default_pot_full_scale_reading;
  paddles_trigger_ptt = true;
  ptt_forced_on = false; // todo: move to reset_config?
  key_forced_down = 0;
  speed_set_by_pot = true;
  send_feedback = false;
  wpm_dif = 0;
  wpm_dif_manual = 0;
  dah_to_dit_ratio = 300; // 3:1
  length_wordspace = 8;
  weighting = 50; // Normal keying
  wpm = pot_value_wpm();
  manual_wpm_limit = 0; // Switched off
  enabled_features = 255; // All features enabled
  paddles_swapped = false;
  btn_toggle = 0;

  // Override the above values by the values from EEPROM on condition EEPROM was already written to:
  load_config();


}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void check_potentiometer() {
  if (bitRead(enabled_features, 2) == 1)
    if ((millis() - last_pot_check_time) > POT_CHECK_INTERVAL) {
      byte pot_value_wpm_read = pot_value_wpm();
      if ((abs(pot_value_wpm_read - last_pot_wpm_read) > POT_CHANGE_THRESHOLD)) {
        speed_set_by_pot = true;
        speed_set(pot_value_wpm_read);
        last_pot_wpm_read = pot_value_wpm_read;
      }
      last_pot_check_time = millis();
    }
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
void check_serial() {
  int bytes_rcvd = 0;

  while ((Serial.available() > 0)) {
    incoming_serial_byte = Serial.read();
    check_com();
    if (bytes_rcvd == 0) {
      delay(10); // Enable the following chars to be received in one row
      send_response(); // When HR sends a string, it awaits a confirmation within some 250 mSec, or so
    } // In order the lead time doesn't occur between first and second char

    bytes_rcvd++;
  }

}
// --------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------
void service_send_buffer() {
  if (send_buffer_bytes() > 0) {
    if (send_buffer[get_at] >= 32) { // Printable character, send it
      send_char(send_buffer[get_at]);
      remove_from_send_buffer();
    } else if ((send_buffer[get_at] >= CMD_FIRST) && (send_buffer[get_at] <= CMD_LAST)) { // Buffered or immediate command
      if (send_buffer_bytes() > 1) {
        process_command(send_buffer[get_at], send_buffer[(get_at + 1) % 256], true);
        remove_from_send_buffer();
        remove_from_send_buffer();
      }
    }
  } else { // Buffer was just exhausted, sending turns to MANUAL again. Necessary for smooth hand-keying that can follow
    sending_mode = MANUAL_SENDING;
    last_sending_mode = MANUAL_SENDING;
    
  }
  
}
// --------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------
void speed_set(int wpm_set) {
  if (wpm_set < pot_wpm_low_value) wpm_set = pot_wpm_low_value;
  if (wpm_set > pot_wpm_high_value) wpm_set = pot_wpm_high_value;
  wpm = wpm_set;
  // Send the reporting bytes when PTT goes to RX. Sending response while hand keying is in progress makes some irregularites in keying.
  if (sending_mode == MANUAL_SENDING && ptt_state)
    response_needed = true;
  else
    send_response();
}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void send_dit() {
  being_sent = SENDING_DIT;
  key(1);
  loop_element_lengths((1.0 * (float(weighting) / 50)), wpm + wpm_dif + wpm_dif_manual);
  key(0);
  loop_element_lengths((2.0 - (float(weighting) / 50)), wpm + wpm_dif + wpm_dif_manual);
  being_sent = SENDING_NOTHING;
  last_sending_mode = sending_mode;
}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void send_dah() {

  being_sent = SENDING_DAH;
  key(1);
  loop_element_lengths((float(dah_to_dit_ratio / 100.0) * (float(weighting) / 50)), wpm + wpm_dif + wpm_dif_manual);
  key(0);
  loop_element_lengths((4.0 - (3.0 * (float(weighting) / 50))), wpm + wpm_dif + wpm_dif_manual);
  being_sent = SENDING_NOTHING;
  last_sending_mode = sending_mode;
}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void key(int state) {

  if ((state) && (key_state == 0)) {

    if (key_forced_down != 1)
      if (paddles_trigger_ptt || sending_mode == AUTOMATIC_SENDING) {
        ptt(true);
      }

    if (bitRead(enabled_features, 1 /*key*/) == 1)
      digitalWrite (TX_KEY_LINE_1, HIGH);

    if (sending_mode == MANUAL_SENDING) {
      if (sidetone_manual > 0)
        tone(SIDETONE_LINE, sidetone_manual);
    }
    else {
      if (sidetone_automatic > 0)
        tone(SIDETONE_LINE, sidetone_automatic);
    }
    key_state = 1;
  } else {
    if ((state == 0) && (key_state)) {
      digitalWrite (TX_KEY_LINE_1, LOW);
      if (key_forced_down != 1)
        if (paddles_trigger_ptt || sending_mode == AUTOMATIC_SENDING)
          ptt(true);
    }
    if (sidetone_automatic > 0 || sidetone_manual > 0)
      noTone(SIDETONE_LINE);
    key_state = 0;
  }
  check_ptt_tail();
}
// --------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------
void loop_element_lengths(float lengths, int speed_wpm_in) {

  float element_length;
  element_length = 1200 / speed_wpm_in;

  unsigned long ticks = long(element_length * lengths);
  unsigned long start = millis();
  while (((millis() - start) < ticks)) {
    check_ptt_tail();

    if ((keyer_mode == IAMBIC_A) && (digitalRead(PADDLE_LEFT) == LOW ) && (digitalRead(PADDLE_RIGHT) == LOW )) {
      iambic_flag = 1;
    }

    if (being_sent == SENDING_DIT) {
      check_dah_paddle();
    } else {
      if (being_sent == SENDING_DAH) {
        check_dit_paddle();
      } else {
        check_dah_paddle();
        check_dit_paddle();
      }
    }

    if (interrupt_sending)
      break;
  }

  if ((keyer_mode == IAMBIC_A) && (iambic_flag) && (digitalRead(PADDLE_LEFT) == HIGH ) && (digitalRead(PADDLE_RIGHT) == HIGH )) {
    iambic_flag = 0;
    dit_buffer = 0;
    dah_buffer = 0;
  }
}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void service_dit_dah_buffers()
{
  if ((keyer_mode == IAMBIC_A) && (iambic_flag) && (digitalRead(PADDLE_LEFT) == HIGH) && (digitalRead(PADDLE_RIGHT) == HIGH)) {
    iambic_flag = 0;
    dit_buffer = 0;
    dah_buffer = 0;
  } else {
    if (dit_buffer) {
      dit_buffer = 0;
      sending_mode = MANUAL_SENDING;
      send_dit();
    }
    if (dah_buffer) {
      dah_buffer = 0;
      sending_mode = MANUAL_SENDING;
      send_dah();
    }
  }
}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
byte pot_value_wpm()
{
  int pot_read = analogRead(POTENTIOMETER);
  byte return_value = map(pot_read, 0, pot_full_scale_reading, pot_wpm_low_value, pot_wpm_high_value);
  return return_value;
}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void ptt(bool state) {
  if (state != ptt_state) {
    ptt_state = state;
    if (state) {  // going to TX
      if (bitRead(enabled_features, 0) == 1)
        digitalWrite (PTT_TX_1, HIGH);
      delay(ptt_lead_time);
    } else {    // going to RX
      digitalWrite (PTT_TX_1, LOW);
      wpm_dif = 0; // Relasing PTT will cancel buffered speed change

      if (response_needed) {
        response_needed = false;
        send_response();
      }
    }

  }
  if (state)
    ptt_time = millis();
}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
void check_ptt_tail() {
  if (key_state) {
    ptt_time = millis();
  } else {
    if ((ptt_state) && (! ptt_forced_on)) {
      if (last_sending_mode == MANUAL_SENDING) {
        if ((millis() - ptt_time) >= (((length_wordspace * ptt_hang_time_wordspace_units)*float(1200 / wpm)) + ptt_tail_time)) {
          ptt(false);
        }
      } else {
        if ((millis() - ptt_time) > ptt_tail_time) {
          if (send_buffer_bytes() == 0)
            ptt(false);
        }
      }
    }
  }

}


// --------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------
void check_dit_paddle() {
  bool was_forced;
 // if (paddle_intr_pending) 
 //   exit;

  if ((digitalRead(PADDLE_LEFT) == LOW) && (! paddles_swapped) ||
      (digitalRead(PADDLE_RIGHT) == LOW) && (paddles_swapped)) {
    if (! interrupt_sending) {
      dit_buffer = 1;
    }
    was_forced = key_forced_down;

    paddles_touched = true;
    key_forced_down = 0;


    if (wpm > manual_wpm_limit && manual_wpm_limit > 0)
      wpm_dif_manual = manual_wpm_limit - wpm;

    // Send response when key was forced. It is necessary for updating the Tune button in HR:
    if (was_forced)
      send_response();


  }
}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void check_dah_paddle() {
  bool was_forced;

 // if (paddle_intr_pending) 
 //   exit;

  if ((digitalRead(PADDLE_RIGHT) == LOW) && (! paddles_swapped) ||
      (digitalRead(PADDLE_LEFT) == LOW) && (paddles_swapped)) {
    if (! interrupt_sending) {
      dah_buffer = 1;
    }
    was_forced = key_forced_down;


    paddles_touched = true;
    key_forced_down = 0;

    if (wpm > manual_wpm_limit && manual_wpm_limit > 0)
      wpm_dif_manual = manual_wpm_limit - wpm;

    // Send response when key was forced. It is necessary for updating the Tune button in HR:
    if (was_forced)
      send_response();

  }
}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void send_the_dits_and_dahs(String const  char_to_send) {
  int i, len;
  sending_mode = AUTOMATIC_SENDING;
  i = 0;
  len =  char_to_send.length();

  while ((i < len)  && (sending_mode == AUTOMATIC_SENDING))
  {
    if ( char_to_send[i] == '.')
      send_dit();
    else if ( char_to_send[i] == '-')
      send_dah();
    i++;
  }

}
// --------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------
void send_char(byte cw_char)
{
  String to_send = "";
  if ((cw_char == 10) || (cw_char == 13)) {
    return;
  }

  sending_mode = AUTOMATIC_SENDING;

  switch (cw_char) {
    case '|':
      loop_element_lengths(0.5, wpm);
      return;
      break;

    case 'A':  to_send = ".-"; break;
    case 'B':  to_send = "-..."; break;
    case 'C':  to_send = "-.-."; break;
    case 'D':  to_send = "-.."; break;
    case 'E':  to_send = "."; break;
    case 'F':  to_send = "..-."; break;
    case 'G':  to_send = "--."; break;
    case 'H':  to_send = "...."; break;
    case 'I':  to_send = ".."; break;
    case 'J':  to_send = ".---"; break;
    case 'K':  to_send = "-.-"; break;
    case 'L':  to_send = ".-.."; break;
    case 'M':  to_send = "--"; break;
    case 'N':  to_send = "-."; break;
    case 'O':  to_send = "---"; break;
    case 'P':  to_send = ".--."; break;
    case 'Q':  to_send = "--.-"; break;
    case 'R':  to_send = ".-."; break;
    case 'S':  to_send = "..."; break;
    case 'T':  to_send = "-"; break;
    case 'U':  to_send = "..-"; break;
    case 'V':  to_send = "...-"; break;
    case 'W':  to_send = ".--"; break;
    case 'X':  to_send = "-..-"; break;
    case 'Y':  to_send = "-.--"; break;
    case 'Z':  to_send = "--.."; break;

    case '0':  to_send = "-----"; break;
    case '1':  to_send = ".----"; break;
    case '2':  to_send = "..---"; break;
    case '3':  to_send = "...--"; break;
    case '4':  to_send = "....-"; break;
    case '5':  to_send = "....."; break;
    case '6':  to_send = "-...."; break;
    case '7':  to_send = "--..."; break;
    case '8':  to_send = "---.."; break;
    case '9':  to_send = "----."; break;

    case '=':  to_send = "-...-"; break;
    case '/':  to_send = "-..-."; break;
    case ' ': loop_element_lengths((length_wordspace - length_letterspace - 2), wpm); break;
    case '*':  to_send = "-...-.-"; break;
    case '.':  to_send = ".-.-.-"; break;
    case ',':  to_send = "--..--"; break;
    case '\'':  to_send = ".----."; break;// apostrophe
    case '!':  to_send = "-.-.--"; break;
    case '?':  to_send = "..--.."; break;
    case '(':  to_send = "-.--."; break;
    case ')':  to_send = "-.--.-"; break;
    case '&':  to_send = ".-..."; break;
    case ':':  to_send = "---..."; break;
    case ';':  to_send = "-.-.-."; break;
    case '+':  to_send = ".-.-."; break;
    case '-':  to_send = "-....-"; break;
    case '_':  to_send = "..--.-"; break;
    case '"':  to_send = ".-..-."; break;
    case '$':  to_send = "...-..-"; break;
    case '@':  to_send = ".--.-."; break;
    case '<':  to_send = ".-.-."; break; // AR
    case '>':  to_send = "...-.-"; break; // SK
  }

  wpm_dif_manual = 0; // Discard speed change for manual sending

  send_the_dits_and_dahs(to_send);
  loop_element_lengths((length_letterspace - 1), wpm); //this is minus one because send_dit and send_dah have a trailing element space

}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void clear_send_buffer() {
  put_at = 0;
  get_at = 0;
  send_response();
}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void remove_from_send_buffer() {
  if (get_at != put_at) {
    get_at++;
    if (get_at == put_at) { // Last char was just removed, send response
      send_response();
    }
  }
}
// --------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------
void add_to_send_buffer(byte incoming_serial_byte) {
  paddles_touched = false;
  send_buffer[put_at] = incoming_serial_byte;
  put_at++;
}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void process_command(byte command, byte data, bool buffered_command ) {
  byte i, j;
  switch (command)
  {
    case CMD_SET_PTT : // PTT, low-level, rate to use
      clear_send_buffer();
      if (data) {
        ptt_forced_on = true;
        ptt(true);
      } else {
        ptt(false);
        ptt_forced_on = false;
      }
      break;
    case CMD_SET_KEY : // Key, low-level, rate to use
      clear_send_buffer();
      key_forced_down = data; // 0, 1, or 2
      if (data > 0) {
        key(true); paddles_touched = false;
      } else {
        key(false);
      }
      send_response(); // Response is sent only when KEY is toggled manually
      break;
    case CMD_SPEED_CHANGE : // Speed
      if (data == 255) {      // Value of 255 returns to the value of potentiometer
        speed_set_by_pot = true;
        speed_set(pot_value_wpm());
      }
      else {
        if (data == 0)             // Value 0 clears the buffered speed change
          wpm_dif = 0;
        else if (buffered_command)  // Other value, if buffered, changes speed temporarily
          wpm_dif = data - wpm;
        else {
          speed_set(data);  // First, report back the changed speed
          speed_set_by_pot = false;
          speed_set(data); // Second, report that it was switched to the PC-controlled way
        }
      }
      break;
    case CMD_BREAK_IMMY : // Speed
      reset_com();
      break;
    case CMD_SET_LEAD_TIME:
      ptt_lead_time = data * 5;
      break;
    case CMD_SET_TAIL_TIME:
      ptt_tail_time = data * 5;
      break;
    case CMD_SET_HANG_TIME:
      ptt_hang_time_wordspace_units = float(data) / 100;
      break;
    case CMD_SET_WEIGHTING:
      weighting = data;
      break;
    case CMD_ENABLE_FEATURES:
      enabled_features = data; // This is bitwise!
      break;
    case CMD_SET_PADDLES_TRIGGER_PTT:
      reset_com();
      paddles_trigger_ptt = data;
      break;
    case CMD_SET_SIDETONE_AUTOMATIC:
      sidetone_automatic = data * 10;
      break;
    case CMD_SET_SIDETONE_MANUAL:
      sidetone_manual = data * 10;
      break;
    case CMD_SET_IAMBIC_MODE:
      if (data == 0) {
        keyer_mode = IAMBIC_A;
      } else if (data == 1) {
        keyer_mode = IAMBIC_B;
      }
      break;
    case CMD_RESET:
      setup();
      break;
    case CMD_PING:
      send_response_bytes();
      break;
    case CMD_BEEP:
      tone(SIDETONE_LINE, 750);
      delay(10);
      noTone(SIDETONE_LINE);
      break;
    case CMD_GET_SIGNATURE:
      Serial.print("Spider Keyer (Arduino Nano) by OK1FIG ["); Serial.print(VERSION); Serial.print("]");  // Don't change this to work seamlessly with HamRacer
      break;
    case CMD_SET_FEEDBACK:
      send_feedback = data > 0;
      send_response();
      break;
    case CMD_SET_LOW_LIMIT :
      if (data == 0) {      // Value of 0 returns to the default value
        pot_wpm_low_value = 15;
      } else {
        pot_wpm_low_value = data;
      }
      break;
    case CMD_SET_HIGH_LIMIT :
      if (data == 0) {      // Value of 0 returns to the default value
        pot_wpm_high_value = 40;
      } else {
        pot_wpm_high_value = data;
      }
      break;

    case CMD_SET_MANUAL_SENDING_LIMIT :
      manual_wpm_limit = data;
      break;

    case CMD_SET_PADDLES_SWAPPED :
      paddles_swapped = data != 0;
      break;

    case CMD_SAVE_CONFIG :
      save_config(data);
      break;

    case CMD_STORE_MSG :
      store_message(data);
      break;

  }
}

// --------------------------------------------------------------------------------------------
void check_com() {
  if (esc_char_rcvd_command == 0) {
    if (incoming_serial_byte != ESCAPE_CHAR) {
      add_to_send_buffer(incoming_serial_byte); // Add even CR, it serves as a flag in the buffer
    } else {     // ESC arrived
      esc_char_rcvd_command = 1;
    }
  } else if (esc_char_rcvd_command == 1) { // Esc char passed, now it is the command byte
    command_buf[0] = incoming_serial_byte;
    esc_char_rcvd_command++;
  } else {
    command_buf[1] = incoming_serial_byte;
    process_command(command_buf[0], command_buf[1], false);
    esc_char_rcvd_command = 0;
  }
}
// --------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------
void send_response() {
  if (send_feedback) {
    send_response_bytes();
  }
}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void send_response_bytes() {
  byte byte1 = 0;
  byte byte2 = 0;
  bitSet(byte1, 7); // The highest bit is always 1

  if (false)  //RESERVED
    bitSet(byte1, 6); //  64

  if ((send_buffer_bytes() > 0) || (esc_char_rcvd_command > 0))
    bitSet(byte1, 5); // any chars buffered? // 32

  if (ptt_state)
    bitSet(byte1, 4); // PTT currently down // 16

  if (key_state)
    bitSet(byte1, 3); // Key currently down  // 8

  if (paddles_touched) // 4
    bitSet(byte1, 2); // Sending was interrupted by touching the paddles

  // For the speed report, use only lower 6 bits
  if (speed_set_by_pot)
    if (wpm <= 64) // MSB bit must remain 0
      byte2 = wpm;

  Serial.write(byte1);
  Serial.write(byte2);

}
// --------------------------------------------------------------------------------------------



// --------------------------------------------------------------------------------------------
byte send_buffer_bytes() { // Number of chars in the buffer to send
  if (put_at >= get_at)
    return put_at - get_at;
  else
    return put_at - get_at % 256;
}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void paddle_down() {
  // Mind this is an interrupt routine!
  if (sending_mode == AUTOMATIC_SENDING) {
    interrupt_sending = true;
  }
}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void save_config(byte data) {
  unsigned int hdr;
  int ptr = 0;

  if (data == 0) {
    hdr = 65535;
    EEPROM.put(ptr, hdr); ptr += sizeof(hdr);
  }
  else {
    hdr = MY_HEADER;
    // Write a header to recognize that EEPROM was written:
    EEPROM.put(ptr, hdr); ptr += sizeof(hdr);
    // Write useful data:
    EEPROM.put(ptr, keyer_mode); ptr += sizeof(keyer_mode);
    EEPROM.put(ptr, sidetone_manual); ptr += sizeof(sidetone_manual);
    EEPROM.put(ptr, sidetone_automatic); ptr += sizeof(sidetone_automatic);
    EEPROM.put(ptr, ptt_tail_time); ptr += sizeof(ptt_tail_time);
    EEPROM.put(ptr, ptt_lead_time); ptr += sizeof(ptt_lead_time);
    EEPROM.put(ptr, ptt_hang_time_wordspace_units); ptr += sizeof(ptt_hang_time_wordspace_units);
    EEPROM.put(ptr, pot_wpm_low_value); ptr += sizeof(pot_wpm_low_value);
    EEPROM.put(ptr, pot_wpm_high_value); ptr += sizeof(pot_wpm_high_value);
    EEPROM.put(ptr, paddles_trigger_ptt); ptr += sizeof(paddles_trigger_ptt);
    EEPROM.put(ptr, weighting); ptr += sizeof(weighting);
    EEPROM.put(ptr, paddles_swapped); ptr += sizeof(paddles_swapped);


  }
  // Beep R:
  tone(SIDETONE_LINE, 400); delay(50); noTone(SIDETONE_LINE); delay(50);
  tone(SIDETONE_LINE, 400); delay(150); noTone(SIDETONE_LINE); delay(50);
  tone(SIDETONE_LINE, 400); delay(50); noTone(SIDETONE_LINE); delay(50);

}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void load_config() {
  unsigned int hdr = 0;
  int ptr = 0;
  EEPROM.get(ptr, hdr); ptr += sizeof(hdr);

  if (hdr == MY_HEADER) {  // The EEPROM was already written
    // Read data:
    EEPROM.get(ptr, keyer_mode); ptr += sizeof(keyer_mode);
    EEPROM.get(ptr, sidetone_manual); ptr += sizeof(sidetone_manual);
    EEPROM.get(ptr, sidetone_automatic); ptr += sizeof(sidetone_automatic);
    EEPROM.get(ptr, ptt_tail_time); ptr += sizeof(ptt_tail_time);
    EEPROM.get(ptr, ptt_lead_time); ptr += sizeof(ptt_lead_time);
    EEPROM.get(ptr, ptt_hang_time_wordspace_units); ptr += sizeof(ptt_hang_time_wordspace_units);
    EEPROM.get(ptr, pot_wpm_low_value); ptr += sizeof(pot_wpm_low_value);
    EEPROM.get(ptr, pot_wpm_high_value); ptr += sizeof(pot_wpm_high_value);
    EEPROM.get(ptr, paddles_trigger_ptt); ptr += sizeof(paddles_trigger_ptt);
    EEPROM.get(ptr, weighting); ptr += sizeof(weighting);
    EEPROM.get(ptr, paddles_swapped); ptr += sizeof(paddles_swapped);
  }

}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void store_message(byte data) {
  // Stores button-message to EEPROM. Use half of the EEPROM size, start at the middle, at 256:
  int i;
  if (data == 0) {   // Mark the whole as unused
    for (i = 256; i < 512; i++) {
      EEPROM.write(i, 0xFF);
    }
  }
  else {
    i = 256;
    while (EEPROM.read(i) != 0xFF && i < 511) i++; // Find first unoccupied position
    if (i < 512) EEPROM.write(i, data);
  }
}
// --------------------------------------------------------------------------------------------


// --------------------------------------------------------------------------------------------
void check_button() {
  // Short button hit sends the message stored in EEPROM. Long press does "TUNE" (key down incl. PTT):
  long ms = millis();
  if (digitalRead(BUTTON) == LOW) {  // Button down
    if (btn_toggle == 0)
      btn_toggle = ms;
    else if (put_at != get_at) { // Some sending is underway
      clear_send_buffer();
      btn_toggle = -1;
    } else if (ms - btn_toggle > 500 && btn_toggle > 0) { // The button kept down for abt half a second
      clear_send_buffer();
      key_forced_down = 2; // Do key and PTT
      key(true);
      paddles_touched = false;
      btn_toggle = -1;
    }
  } else {  // Button released
    if (btn_toggle > 0) {
      //  ms = ms - btn_toggle;
      if (ms - btn_toggle > 10 && ms - btn_toggle < 500) { // The button shortly hit
        if (put_at == get_at) { // Send button message only if nothing else to trasmit is in the buffer
          send_message();
        }
      }
    }

    if (btn_toggle == -1) key(false);
    btn_toggle = 0;
  }
}
// --------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------
// Copies the button-message from EEPROM to the standard to-be-sent buffer:
void send_message() {
  int i = 256;
  while (EEPROM.read(i) != 0xFF && i < 512) {
    add_to_send_buffer(EEPROM.read(i));
    i++;
  }
}
// --------------------------------------------------------------------------------------------

// --------------------------------------------------------------------------------------------
// The paddle that interrupted sending was just released
void check_interruption() {

  if (interrupt_sending) { // The flag was set in ISR routine

    // The interrupting paddle was released:
    if ((digitalRead(PADDLE_LEFT) == HIGH) && (digitalRead(PADDLE_RIGHT) == HIGH)) {
      interrupt_sending = false;
      wpm_dif = 0;
      paddles_touched = true;
      send_response();  // Notify ctrling appl. immy
    }

    // The interrupting paddle was just depressed:
    if ((digitalRead(PADDLE_LEFT) == LOW) || (digitalRead(PADDLE_RIGHT) == LOW)) {
      put_at = 0;
      get_at = 0;
    }

    
  }
    

}
// --------------------------------------------------------------------------------------------
