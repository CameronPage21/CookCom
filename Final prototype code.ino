// This wristband has address 2 on the network (orange wristband)
#define WRISTBAND_id 2
// The network consists of 6 wristbands total, maximum possible is 8
#define NETWORK_SIZE 6

/* ----------------------------------------------------------------------
GENERAL BROADCAST FORMAT:

Recipient-Sender-Purpose
I.E: "01-04-3"

Recipient (string) = "00", "01", "02", ... , "XX". "00" means everyone.
Sender (string) = "01", "02", ... , "XX". "00" means the broadcast station.
Purpose (string):
  - 0 = tell recipient to start vibrating
  - 1 = tell recipient to stop vibrating
  - 2 = request connection with recipient
  - 3 = accept connection with sender
  - 4 = send message acknowledgement buzz to recipient
  - 5 = tell recipient to start vibrating (reply mode)
  - 6 = tell recipient to stop vibrating (reply mode)

-----------------------------------------------------------------------*/

// Include libraries
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <STBLE.h>


// ---------------------- BLE setup from TinyCircuits example code ----------------------

//Debug output adds extra flash and memory requirements!
#ifndef BLE_DEBUG
#define BLE_DEBUG true
#endif

/*
#if defined (ARDUINO_ARCH_AVR)
#define SerialMonitorInterface Serial
#elif defined(ARDUINO_ARCH_SAMD)
#define SerialMonitorInterface SerialUSB
#endif
*/

uint8_t ble_rx_buffer[21];
uint8_t ble_rx_buffer_len = 0;
uint8_t ble_connection_state = false;
#define PIPE_UART_OVER_BTLE_UART_TX_TX 0

#if BLE_DEBUG
#include <stdio.h>
char sprintbuff[100];
#define PRINTF(...) {sprintf(sprintbuff,__VA_ARGS__);SerialUSB.print(sprintbuff);}
#else
#define PRINTF(...)
#endif


volatile uint8_t set_connectable = 1;
uint16_t connection_handle = 0;


#define  ADV_INTERVAL_MIN_MS  50
#define  ADV_INTERVAL_MAX_MS  100


int connected = FALSE;

// ------------------------------------------------------------------------------------


// Define pins
#define LED_PIN 6
#define TOUCH_SENSOR_PIN 7
#define MOTOR_PIN 5

// Define physical parameters
#define MOTOR_STRENGTH_PERCENT 0.8
#define LED_COUNT 2
#define PULSE_RATE 50     // Number of milliseconds between LED pulsing pattern increments
#define CYCLE_RATE 750    // The rate that the menu cycles through options when contacting a recipient
#define CONNECTION_HANDSHAKE_TIMEOUT 4000    // Wristbands attempting to send a message will wait this long
                                             // before deciding that the recipient isn't available
#define INACTIVITY_FOR_BROADCAST_FINISH 4000 // Number of milliseconds the wristband waits before assuming
                                             // that the sender has finished with their message
#define INITIAL_REPLY_DEADLINE 6000          // Number of seconds a recipient has after the preceeding
                                             // broadcast finishes to start a reply
#define REPLY_DEADLINE_AFTER_STARTED 4000    // Amount of time the user has to send a message before it times
                                             // out, after their first touch sensor press
#define ALTERNATE_RATE 400          // Rate at which the LEDs alternate during the replies time window
#define HAPTIC_FEEDBACK_LENGTH 50   // Amount of time that the vibration motor is active during haptic pulses

// Define variables
bool TouchState = 0;                    // Current state of the touch sensor
bool TouchStatePrevious = 0;            // State of the touch sensor last time it was polled
unsigned long lastLEDpulse;             // Time in millis() since previous increment in LED pulse pattern
float currentLEDmultiplier = 5;         // Current brightness multiplier value for a pulsing LED
bool pulseDirection = 1;                // 1 = going from dim to bright, 0 = going from bright to dim
unsigned long tic;                      // Counter variable when measuring time using millis()
unsigned long tic2;                     // Second counter variable when measuring time using millis()
int cycle = 0;                          // Used in senderMenu() to track cycles through the menu structure
int contactID = 0;                      // Used in senderMenu() to track the ID corresponding to LED colour
String reply;                           // Used in checkIfConnected() to track if recipient has responded yet
unsigned long mostRecentMessage = 0;    // Time in millis() of most recent incoming message
unsigned long mostRecentMotor = 0;      // Time in millis() of most recent command to start vibration motor
bool motorTurning = 0;                  // 1 if the vibration motor is on, 0 if not
uint32_t currentColour;                 // Current colour to alternate or pulse whilst receiving
bool alternatingLEDCurrent = 0;         // Tracks which of the two LEDs is lit currently during alternation
bool wristShake = 0;                    // Tracks whether a wrist shake has been recently detected
int sendTo;                             // Used to track who the user wants to send a message to
int replyDeadline;                      // Amount of time the user has to send a message before it times out
String main_loop_receive;               // Used in the main loop to store received tranmissions
bool debugMotorNotify = 0;              // Used for reducing debugging serial print spam

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800); // Define LED strip object

void setup() {
  // Set up I/O
  pinMode(TOUCH_SENSOR_PIN, INPUT);
  pinMode(MOTOR_PIN, OUTPUT);
  digitalWrite(MOTOR_PIN, HIGH);

  // Initialise serial monitor
  SerialUSB.begin(9600);

  // Initialise BLE module
  BLEsetup();

  // Initialise Neopixel strip
  strip.begin();
  strip.show(); // Initialise all pixels as off
  while (!SerialUSB);
}

// Define colours
uint32_t white = strip.Color(255, 255, 255);    // id = 0, all wristbands
uint32_t red = strip.Color(255, 0, 0);          // id = 1
uint32_t orange = strip.Color(255, 128, 0);     // id = 2
uint32_t green = strip.Color(0, 255, 0);        // id = 3
uint32_t cyan = strip.Color(0, 255, 255);       // id = 4
uint32_t blue = strip.Color(0, 0, 255);         // id = 5
uint32_t purple = strip.Color(128, 0, 255);     // id = 6
uint32_t pink = strip.Color(255, 0, 255);       // id = 7
uint32_t yellow = strip.Color(255, 255, 0);     // id = 8
uint32_t colourArray[] = {white, red, orange, green, cyan, blue, purple, pink, yellow};


// --------------------------------START OF BLUETOOTH MODULE CODE-------------------------------------------

int BLEsetup() {
  int ret;

  HCI_Init();
  /* Init SPI interface */
  BNRG_SPI_Init();
  /* Reset BlueNRG/BlueNRG-MS SPI interface */
  BlueNRG_RST();

  uint8_t bdaddr[] = {0x12, 0x34, 0x00, 0xE1, 0x80, 0x02};

  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);

  if (ret) {
    PRINTF("Setting BD_ADDR failed.\n");
  }

  ret = aci_gatt_init();

  if (ret) {
    PRINTF("GATT_Init failed.\n");
  }

  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);

  if (ret) {
    PRINTF("GAP_Init failed.\n");
  }

  const char *name = "BlueNRG";

  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0, strlen(name), (uint8_t *)name);

  if (ret) {
    PRINTF("aci_gatt_update_char_value failed.\n");
  } else {
    PRINTF("BLE Stack Initialized.\n");
  }

  ret = Add_UART_Service();

  if (ret == BLE_STATUS_SUCCESS) {
    PRINTF("UART service added successfully.\n");
  } else {
    PRINTF("Error while adding UART service.\n");
  }

  /* +4 dBm output power */
  ret = aci_hal_set_tx_power_level(1, 3);
}

void aci_loop() {
  HCI_Process();
  ble_connection_state = connected;
  if (set_connectable) {
    setConnectable();
    set_connectable = 0;
  }
  if (HCI_Queue_Empty()) {
    //Enter_LP_Sleep_Mode();
  }
}

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
  do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
  }while(0)

#define COPY_UART_SERVICE_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x6E, 0x40, 0x00, 0x01, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E)
#define COPY_UART_TX_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x6E, 0x40, 0x00, 0x02, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E)
#define COPY_UART_RX_CHAR_UUID(uuid_struct)  COPY_UUID_128(uuid_struct,0x6E, 0x40, 0x00, 0x03, 0xB5, 0xA3, 0xF3, 0x93, 0xE0, 0xA9, 0xE5, 0x0E, 0x24, 0xDC, 0xCA, 0x9E)

uint16_t UARTServHandle, UARTTXCharHandle, UARTRXCharHandle;

uint8_t Add_UART_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];

  COPY_UART_SERVICE_UUID(uuid);
  ret = aci_gatt_add_serv(UUID_TYPE_128,  uuid, PRIMARY_SERVICE, 7, &UARTServHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  COPY_UART_TX_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(UARTServHandle, UUID_TYPE_128, uuid, 20, CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, GATT_NOTIFY_ATTRIBUTE_WRITE,
                           16, 1, &UARTTXCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  COPY_UART_RX_CHAR_UUID(uuid);
  ret =  aci_gatt_add_char(UARTServHandle, UUID_TYPE_128, uuid, 20, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0,
                           16, 1, &UARTRXCharHandle);
  if (ret != BLE_STATUS_SUCCESS) goto fail;

  return BLE_STATUS_SUCCESS;

fail:
  PRINTF("Error while adding UART service.\n");
  return BLE_STATUS_ERROR ;

}

uint8_t lib_aci_send_data(uint8_t ignore, uint8_t* sendBuffer, uint8_t sendLength) {
  return !Write_UART_TX((char*)sendBuffer, sendLength);
}

uint8_t Write_UART_TX(char* TXdata, uint8_t datasize)
{
  tBleStatus ret;

  ret = aci_gatt_update_char_value(UARTServHandle, UARTRXCharHandle, 0, datasize, (uint8_t *)TXdata);

  if (ret != BLE_STATUS_SUCCESS) {
    PRINTF("Error while updating UART characteristic.\n") ;
    return BLE_STATUS_ERROR ;
  }
  return BLE_STATUS_SUCCESS;

}

void Read_Request_CB(uint16_t handle)
{
  /*if(handle == UARTTXCharHandle + 1)
    {

    }
    else if(handle == UARTRXCharHandle + 1)
    {


    }*/

  if (connection_handle != 0)
    aci_gatt_allow_read(connection_handle);
}


void setConnectable(void)
{
  tBleStatus ret;

  const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME, 'B', 'l', 'u', 'e', 'N', 'R', 'G'};

  hci_le_set_scan_resp_data(0, NULL);
  PRINTF("General Discoverable Mode.\n");

  ret = aci_gap_set_discoverable(ADV_IND,
                                 (ADV_INTERVAL_MIN_MS * 1000) / 625, (ADV_INTERVAL_MAX_MS * 1000) / 625,
                                 STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE,
                                 sizeof(local_name), local_name, 0, NULL, 0, 0);

  if (ret != BLE_STATUS_SUCCESS)
    PRINTF("%d\n", (uint8_t)ret);

}

void Attribute_Modified_CB(uint16_t handle, uint8_t data_length, uint8_t *att_data)
{
  if (handle == UARTTXCharHandle + 1) {
    int i;
    for (i = 0; i < data_length; i++) {
      ble_rx_buffer[i] = att_data[i];
    }
    ble_rx_buffer[i] = '\0';
    ble_rx_buffer_len = data_length;
  }
}

void GAP_ConnectionComplete_CB(uint8_t addr[6], uint16_t handle) {

  connected = TRUE;
  connection_handle = handle;

  PRINTF("Connected to device:");
  for (int i = 5; i > 0; i--) {
    PRINTF("%02X-", addr[i]);
  }
  PRINTF("%02X\r\n", addr[0]);
}

void GAP_DisconnectionComplete_CB(void) {
  connected = FALSE;
  PRINTF("Disconnected\n");
  /* Make the device connectable again. */
  set_connectable = TRUE;
}



void HCI_Event_CB(void *pckt)
{
  hci_uart_pckt *hci_pckt = (hci_uart_pckt *)pckt;
  hci_event_pckt *event_pckt = (hci_event_pckt*)hci_pckt->data;

  if (hci_pckt->type != HCI_EVENT_PKT)
    return;

  switch (event_pckt->evt) {

    case EVT_DISCONN_COMPLETE:
      {
        //evt_disconn_complete *evt = (void *)event_pckt->data;
        GAP_DisconnectionComplete_CB();
      }
      break;

    case EVT_LE_META_EVENT:
      {
        evt_le_meta_event *evt = (evt_le_meta_event *)event_pckt->data;

        switch (evt->subevent) {
          case EVT_LE_CONN_COMPLETE:
            {
              evt_le_connection_complete *cc = (evt_le_connection_complete *)evt->data;
              GAP_ConnectionComplete_CB(cc->peer_bdaddr, cc->handle);
            }
            break;
        }
      }
      break;

    case EVT_VENDOR:
      {
        evt_blue_aci *blue_evt = (evt_blue_aci *)event_pckt->data;
        switch (blue_evt->ecode) {

          case EVT_BLUE_GATT_READ_PERMIT_REQ:
            {
              evt_gatt_read_permit_req *pr = (evt_gatt_read_permit_req *)blue_evt->data;
              Read_Request_CB(pr->attr_handle);
            }
            break;

          case EVT_BLUE_GATT_ATTRIBUTE_MODIFIED:
            {
              evt_gatt_attr_modified_IDB05A1 *evt = (evt_gatt_attr_modified_IDB05A1*)blue_evt->data;
              Attribute_Modified_CB(evt->attr_handle, evt->data_length, evt->att_data);
            }
            break;
        }
      }
      break;
  }

}

// --------------------------------END OF BLUETOOTH MODULE CODE-------------------------------------------


// --------------------------------- HELPER FUNCTIONS ----------------------------------------------------


// Function that converts from int number ID to string in format "07"
String intToStr(int ID) {
  if ((ID < 10) && (ID >= 0)) {
    return "0" + String(ID);
  } else if ((ID >= 10) && (ID < 100)) {
    return String(ID);
  }
}


// Function that makes the LED increment in its pulse routine
void pulseIncrement(uint32_t colour) {
  //SerialUSB.println("Running pulseIncrement");
  strip.setPixelColor(0, colour); // Assign the correct colour to both LEDs
  strip.setPixelColor(1, colour);

  strip.setBrightness(currentLEDmultiplier * 25);

  // Adjust the brightness by 10%, either brighter or dimmer, depending on pulseDirection
  if (pulseDirection) {
    currentLEDmultiplier += 1;
  } else {
    currentLEDmultiplier -= 1;
  }

  // If the brightness is at max or min, reverse the pulse direction
  if ((currentLEDmultiplier == 10) || (currentLEDmultiplier == 0)) {
    pulseDirection = !pulseDirection;
  }
  //SerialUSB.println("New brightness: " + String(currentLEDmultiplier));

  // Push the changes to the LED strip 
  strip.show();
}

// Function that makes the LEDs alternate back and forth
void alternateLED(uint32_t colour) {
  //SerialUSB.println("Running alternateLED");
  if (alternatingLEDCurrent) { // Check which LED is currently lit
    strip.setPixelColor(0, 0);
    strip.setPixelColor(1, colour); // Make the other LED be the one that is lit
  } else {
    strip.setPixelColor(0, colour);
    strip.setPixelColor(1, 0);
  }
  strip.show(); // Push the new configuration to the LED array
  alternatingLEDCurrent = !alternatingLEDCurrent; // Record that the LEDs have swapped
}


// Function to create a broadcast using the general format laid out
// at the top of this script
void broadcastFormatted(int recipient, int sender, int purpose) {
  //SerialUSB.println("Running broadcastFormatted, sending to " + String(recipient));
  String formatted = intToStr(recipient) + "-" + intToStr(sender) + "-" + String(purpose);
  //SerialUSB.println(formatted);
  broadcast(formatted);
}


// Use BLE module to send a message
void broadcast(String message) {
  // aci_loop(); // Send via BLE

  // Truncate message if longer than 20 characters
  int messageLength = message.length();
  uint8_t sendLength = 0;
  sendLength = messageLength;
  if (messageLength > 20) {
    message = message.substring(0, 20);
    sendLength = 20;
  }
  
  // Add message to the BLE module's send buffer
  uint8_t sendBuffer[21];
  for (int i = 0; i < sendLength; i++) {
    sendBuffer[i] = message[i];
  }
  sendBuffer[sendLength] = '\0'; // Add terminator string
  sendLength++;

  /*
  for(int i = 0; i < 21; i++) {
    SerialUSB.print(sendBuffer[i]);
  }
  SerialUSB.println(sendBuffer[21]);
  SerialUSB.println("Send length: " + String(sendLength));
  */

  if (!lib_aci_send_data(PIPE_UART_OVER_BTLE_UART_TX_TX, (uint8_t*)sendBuffer, sendLength)) {
    SerialUSB.println(F("TX dropped!"));
  }
  aci_loop(); // Send via BLE

  SerialUSB.println("[Sent] " + message);
}

// Use BLE module to scan for new incoming messages
String receive() {
  aci_loop(); // Refresh BLE buffers
  if (ble_rx_buffer_len) { // Check if data is available
    ble_rx_buffer_len = 0; // Clear after reading
    SerialUSB.println("[Received] " + String((char*)ble_rx_buffer));
    return (char*)ble_rx_buffer;
    
  } else {
    return "No reply"; // Return 0 if there is no message to receive
  }
}

// Helper function for senderMenu() that advances the LED to the next colour in the menu
// cycle and gives the vibration motor a small buzz
void advanceLED() {
  strip.setPixelColor(1, colourArray[contactID]);
  strip.show();
  motorOn();
  delay(50);
  motorOff();
}


void motorOn() { // Turns the vibration motor on
  // Only say this once
  if (debugMotorNotify == 0) {
    //SerialUSB.println("Motor on");
    debugMotorNotify = 1;
  }
  digitalWrite(MOTOR_PIN, LOW);
  mostRecentMotor = millis();
  motorTurning = 1;
}


void motorOff() { // Turns the vibration motor off
  // Only say this once
  if (debugMotorNotify == 1) {
    //SerialUSB.println("Motor off");
    debugMotorNotify = 0;
  }
  if (motorTurning) {
    digitalWrite(MOTOR_PIN, HIGH);
    motorTurning = 0;
  }
}


// Function that ensures the motor isn't left running for longer than 3 seconds
// I.e. if wireless connection drops in between motor start and stop commands
void backupMotorOff() {
  if (motorTurning && ((millis() - mostRecentMotor) > 3000)) {
    digitalWrite(MOTOR_PIN, HIGH);
    motorTurning = 0;
  }
}


// Checks if the touch sensor is currently being pressed
int TouchSensorPressed() {
  if (digitalRead(TOUCH_SENSOR_PIN)) {
    return 1;
  } else {
    return 0;
  }
}


// Takes a reading of the touch sensor and determines if a change in state
// has occured since the previous reading
int TouchSensorPoll() {
  TouchState = TouchSensorPressed();
  if (TouchState != TouchStatePrevious) {
    TouchStatePrevious = TouchState;
    if (TouchState) {
      return 1;
    } else {
      return 2;
    }
  } else {
    TouchStatePrevious = TouchState;
    return 0;
  }
}

// Converts an address in the form "01" or "24" to 1 or 24 respectively
int addressStrToInt(String address) {
  //SerialUSB.println(address.toInt());
  return address.toInt();
}


// -------------------------------- MAIN LOOP FUNCTIONS -----------------------------------------


// Function that cycles through the available message recipients by cycling LED colours
int senderMenu() {
  SerialUSB.println("Starting menu function");
  cycle = 0;
  contactID = 0;
  advanceLED();
  delay(CYCLE_RATE); // "Broadcast to all" option spends twice as long displayed the first time
  tic = millis();

  // While loop runs until the users lets go of the sensor, or the menu cycles through twice
  while (TouchSensorPressed() && (cycle < 2)) {

    // Skip the option for the colour of this wristband
    if (contactID == WRISTBAND_id) {
      contactID++;
      advanceLED();
    }

    // Check if enough time has passed to cycle to the next menu item
    if ((millis() - tic) > CYCLE_RATE) {
      SerialUSB.println("--- Menu option " + String(contactID));

      // Update the variables tracking the position and cycle number of the menu
      tic = millis();
      contactID++;
      if (contactID >= NETWORK_SIZE) {
        contactID = 0;
        cycle++;
      }
      // Move to the next LED colour unless two menu cycles have passed
      if (cycle < 2) {
        advanceLED();
      }
    }
  }
  // If the whole menu is cycled through more than twice, turn the LEDs off and return -1
  if (cycle >= 2) {
    strip.setPixelColor(1, 0, 0, 0);
    strip.show();
    while(TouchSensorPressed());
    SerialUSB.println("No wristband selected! Returning to main loop.");
    return -1;
  // Otherwise, return the ID corresponding to the current LED colour
  } else {
    SerialUSB.println("Selected wristband " + String(contactID));
    return contactID;
  }
}


// Send a message to a given wristband ID, if that wristband is available
void sendMessage(int ID) {
  if (checkIfConnected(ID)) {
    SerialUSB.println("Sending message to wristband " + String(ID) + ". (0 = broadcast to all)");
    int pollResult;
    currentColour = colourArray[ID];

    motorOn();
    delay(100);
    motorOff();    // Two short buzzes to prompt the user to start sending their message
    delay(100);
    motorOn();
    delay(100);
    motorOff();

    tic = millis();
    mostRecentMessage = millis(); // Reset the inactivity timeout
    alternatingLEDCurrent = 0;
    alternateLED(currentColour); // Set up the alternating LED pattern
    replyDeadline = INITIAL_REPLY_DEADLINE;

    // Loop until the user has finished replying, n secs after the most recent touch input
    while ((millis() - mostRecentMessage) < replyDeadline) {
      
      if ((millis() - tic) > ALTERNATE_RATE) { // Alternate LEDs if enough time has passed
        alternateLED(currentColour);
        tic = millis();
      }

      // Turn vibration motor off after a haptic pulse is finished
      if ((millis() - tic2) > HAPTIC_FEEDBACK_LENGTH) {
        motorOff();
      }        

      pollResult = TouchSensorPoll();
      detectWristShake();

      // If the touch state changes to pressed, send a message to the recipient wristband
      // to start vibrating
      if (pollResult == 1) {
        broadcastFormatted(ID, WRISTBAND_id, 0);
        SerialUSB.println("Commanding recipient wristband (" + String(ID) + ") to turn vibration motor ON");
        motorOn(); // Vibrate the motor on this wristband for haptic feedback

        // Lower the message timeout after the initial command has been sent
        replyDeadline = REPLY_DEADLINE_AFTER_STARTED;
        tic2 = millis(); // Haptic feedback vibrations timer

      // If the touch state changes to unpressed, send a message to the recipient wristband
      // to stop vibrating, and reset the inactivity timeout
      } else if (pollResult == 2) {
        broadcastFormatted(ID, WRISTBAND_id, 1);
        SerialUSB.println("Commanding recipient wristband to turn vibration motor OFF");
        mostRecentMessage = millis();
      }
      backupMotorOff();
    }
    SerialUSB.println("Exit send mode");

  } else { 
    // If the wristband failed to connect to its intended sender, notify the user

    for (int i = 0; i < 3; i++) { // Flash the LEDs red and buzz the motors 3 times
      strip.setPixelColor(0, red);
      strip.setPixelColor(1, red);
      strip.show();
      motorOn();
      delay(300);

      strip.setPixelColor(0, 0, 0, 0);
      strip.setPixelColor(1, 0, 0, 0);
      strip.show();
      motorOff();
      delay(300);
    }
  }

  motorOff(); // Ensure vibration motor and LEDs are off after message is finished
  strip.setPixelColor(0, 0, 0, 0);
  strip.setPixelColor(1, 0, 0, 0);
  strip.show();
}


// Sends a connection request to the desired recipient to see if they're in wireless
// communication range and not already busy. Times out after CONNECTION_HANDSHAKE_TIMEOUT
int checkIfConnected(int ID) {
  if (ID == 0) {
    broadcastFormatted(ID, WRISTBAND_id, 2); // Send a broadcast request to everyone
    SerialUSB.println("Broadcasting to all, so availability check skipped");
    return 1; // Skip the connection check if broadcasting to everyone
  }
  SerialUSB.println("Contacting wristband " + String(ID) + " to check for availability");

  // Send a broadcast that asks the recipient wristband to respond if it's available
  reply = "No reply";
  broadcastFormatted(ID, WRISTBAND_id, 2);

  // Wait up to CONNECTION_HANDSHAKE_DELAY milliseconds for a response
  tic = millis();
  while ((millis() - tic) < CONNECTION_HANDSHAKE_TIMEOUT) {
    reply = receive();

    // If there is a reply, check that the reply is an acknowledgment from the correct
    // wristband recipient. If so, return 1
    if (reply != "No reply") {
      if ((intToStr(WRISTBAND_id) + "-" + intToStr(ID) + "-" + "3") == reply) {
        SerialUSB.println("Availability check successful, entering send mode");
        return 1;
      }
      // If the reply wasn't an acknowledgement, continue listening
      reply = "No reply";
    }
  }
  SerialUSB.println("Tried to connect to wristband " + String(ID) + ", but there was no response");
  return 0; // If no acknowledgement is received within the timeout period, return 0
}


// Function that checks incoming messages during the main loop for actionable information
void analyseReceivedMessage(String message) {
  SerialUSB.println("Analysing message received from wristband " + String(addressStrToInt(message.substring(3,5))));

  if ((message.substring(0,2) == "00") && (message.substring(6) == "2")) { // Check if the message is addressed to everyone
    // If so, enter broadcast receiving mode without an option to reply
    receiveReplyMessage(addressStrToInt(message.substring(3,5)), 0);
    SerialUSB.println("Received message was a broadcast to all, entering receive mode");

  // Check if the message is addressed to this wristband
  } else if (message.substring(0,2) == String(intToStr(WRISTBAND_id))) {

    // If the received message is a connection request, accept it and then enter receive mode
    if (message.substring(6) == "2") {
      
      // Don't give the option to reply if the message is a connection request from the base station
      if (message.substring(3,7) == "00-2") {
        SerialUSB.println("Received message was an availability request from the base station, entering receive mode");
        receiveReplyMessage(0, 0);

      } else { // Otherwise, do give the option to reply
        broadcastFormatted(addressStrToInt(message.substring(3,5)), WRISTBAND_id, 3);
        SerialUSB.println("Received message was an availability request from wristband " + String(addressStrToInt(message.substring(3,5))) + ", entering receive mode");
        receiveReplyMessage(addressStrToInt(message.substring(3,5)), 1);
      }
      
    // If the received message is a reply saying to start vibrating, start vibrating
    } else if (message.substring(6) == "5") {
      SerialUSB.println("Received message was a reply to a previous message, commanding the vibration motor to turn ON");
      //SerialUSB.println("Reply buzz on");
      motorOn();

    // If the received message is a reply saying to stop vibrating, stop vibrating
    } else if (message.substring(6) == "6") {
      SerialUSB.println("Received message was a reply to a previous message, commanding the vibration motor to turn OFF");
      //SerialUSB.println("Reply buzz off");
      motorOff();
    }
  }
}


// Function that loops whilst the wristband is receiving or replying to a message
void receiveReplyMessage(int senderID, bool optionToReply) {
  SerialUSB.println("Receiving message from wristband " + String(senderID) + ". (0 = base station)");
  String recentReceived;
  int pollResult;
  tic = millis();
  mostRecentMessage = millis(); // Reset the inactivity timeout
  currentColour = colourArray[senderID];
  pulseIncrement(currentColour);

  // Loop until the most recent message component was sent a certain amount of time ago
  while ((millis() - mostRecentMessage) < INACTIVITY_FOR_BROADCAST_FINISH) {

    if ((millis() - tic) > PULSE_RATE) { // Pulse the LEDs if enough time has passed
      pulseIncrement(currentColour);
      tic = millis();
    }
    recentReceived = receive(); // Scan for recent incoming messages

    // If a recent message (from either the sender, or as a broadcast to all) tells the 
    // vibration motor to turn on, turn it on
    if ((recentReceived == (intToStr(WRISTBAND_id) + "-" + intToStr(senderID) + "-" + "0")) ||
      (recentReceived == ("00-" + intToStr(senderID) + "-" + "0"))) {

      SerialUSB.println("Received a command - motor ON");
      motorOn();
      mostRecentMessage = millis(); // Reset the inactivity timeout

      // If a recent message (from either the sender, or as a broadcast to all) tells the 
      // vibration motor to turn off, turn it off
    } else if ((recentReceived == (intToStr(WRISTBAND_id) + "-" + intToStr(senderID) + "-" + "1")) ||
      (recentReceived == ("00-" + intToStr(senderID) + "-" + "1"))) {

      SerialUSB.println("Received a command - motor OFF");
      motorOff();
      mostRecentMessage = millis(); // Reset the inactivity timeout
    }
  }
  SerialUSB.println("Exit receive mode");

  strip.setPixelColor(0, 0); // Turn both LEDs off
  strip.setPixelColor(1, 0); // No strip.show() because alternateLED() overwrites

  // If the preceeding message wasn't a broadcast to all wristbands, the user can reply
  if (optionToReply) {
    SerialUSB.println("The message was not from the base station, or a broadcast to all, so replying is permitted.");
    SerialUSB.println("Enter reply mode, sending replies to wristband " + String(senderID));
    motorOn();
    delay(50);
    motorOff();    // Two short buzzes to prompt the user to reply if they want
    delay(50);
    motorOff();

    tic = millis();
    mostRecentMessage = millis(); // Reset the inactivity timeout
    alternatingLEDCurrent = 0;
    alternateLED(currentColour); // Set up the alternating LED pattern
    replyDeadline = INITIAL_REPLY_DEADLINE;

    // Loop until the user has finished replying, n secs after the most recent touch input
    while ((millis() - mostRecentMessage) < replyDeadline) {
      
      if ((millis() - tic) > ALTERNATE_RATE) { // Alternate LEDs if enough time has passed
        alternateLED(currentColour);
        tic = millis();
      }

      // Turn vibration motor off after a haptic pulse is finished
      if ((millis() - tic2) > HAPTIC_FEEDBACK_LENGTH) {
        motorOff();
      }        

      pollResult = TouchSensorPoll();
      detectWristShake();

      // If the touch state changes to pressed, send a message to the original wristband
      // to start vibrating
      if (pollResult == 1) {
        broadcastFormatted(senderID, WRISTBAND_id, 5);
        motorOn(); // Vibrate the motor on this wristband for haptic feedback

        // Lower the reply timeout after the initial command has been sent
        replyDeadline = REPLY_DEADLINE_AFTER_STARTED;
        tic2 = millis(); // Haptic feedback vibrations timer

      // If the touch state changes to unpressed, send a message to the original wristband
      // to stop vibrating, and reset the inactivity timeout
      } else if (pollResult == 2) {
        broadcastFormatted(senderID, WRISTBAND_id, 6);
        mostRecentMessage = millis();
        
      // Check if a wrist shaking motion has been detected, signifying acknowledgment of
      // the original message
      } else if (wristShake) {
        wristShake = 0;
        broadcastFormatted(senderID, WRISTBAND_id, 4);
        mostRecentMessage = millis();
        replyDeadline = 0; // Causes the while loop to end next time around
      }

      backupMotorOff();
    }
    SerialUSB.println("Exit reply mode");
  }

  motorOff(); // Ensure vibration motor and LEDs are off after message is finished
  strip.setPixelColor(0, 0, 0, 0);
  strip.setPixelColor(1, 0, 0, 0);
  strip.show();
}


// Function that makes the variable wristShake = 1 if a wrist shaking gesture is
// detected. This signifies acknowledgement of a previous message
void detectWristShake() {
  wristShake = 0;
}


// ---------------------------------- MAIN LOOP --------------------------------------------


void loop() {

  // Turns the vibration motor off after 3 secs if a turn off command isn't received
  backupMotorOff();

  // Processes recent commands in the bluetooth module
  aci_loop();

  // Scans for incoming messages. If a received message contains information, analyse it
  main_loop_receive = receive();
  if (main_loop_receive != "No reply") {
    // Analyses the message and then runs the appropriate function based on the result
    analyseReceivedMessage(main_loop_receive);
  }
  
  // If the user wants to send a message, run the menu function that allows them to select
  // who they want to send to
  if (TouchSensorPressed()) {
    sendTo = senderMenu();

    // If the user selected a valid person from the menu function, contact that person
    if (sendTo != -1) {
      // Checks whether the person is available, then allows the user to type out a message
      sendMessage(sendTo);
    }
  }
}
