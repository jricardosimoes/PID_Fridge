#include <PID_v1.h>
#include <LiquidCrystal.h> 
#include <OneWire.h> 
#include <DallasTemperature.h>

// Porta do pino de sinal do DS18B20
#define ONE_WIRE_BUS 3

// Define uma instancia do oneWire para comunicacao com o sensor
OneWire oneWire(ONE_WIRE_BUS);

DallasTemperature sensors( & oneWire);
DeviceAddress sensor1;

float target = 18.0;
float sensor = 0.0;
int running_pid = 0;
unsigned long millis_since_last_try_on = 0;
unsigned long millis_interval = 1000 * 60 * 5; //1 min
unsigned long pause_after_off = 3 * 60; // min
unsigned long last_rl_off = 0;

// select the pins used on the LCD panel
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// define some values used by the panel and buttons
int lcd_key = 0;
int adc_key_in = 0;

#define btnRIGHT 0
# define btnUP 1
# define btnDOWN 2
# define btnLEFT 3
# define btnSELECT 4
# define btnNONE 5

# define RELAY_PIN 2

//PID
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
double Kp = 2, Ki = 5, Kd = 1;
PID myPID( & Input, & Output, & Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 5000;
unsigned long windowStartTime;

// read the buttons
int read_LCD_buttons() {
    adc_key_in = analogRead(0); // read the value from the sensor 
    if (adc_key_in > 1000) return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result
    if (adc_key_in < 50) return btnRIGHT;
    if (adc_key_in < 250) return btnUP;
    if (adc_key_in < 450) return btnDOWN;
    if (adc_key_in < 650) return btnLEFT;
    if (adc_key_in < 850) return btnSELECT;

    return btnNONE; // when all others fail, return this...
}

void mostra_endereco_sensor(DeviceAddress deviceAddress) {
    for (uint8_t i = 0; i < 8; i++) {
        // Adiciona zeros se necessário
        if (deviceAddress[i] < 16) Serial.print("0");
        Serial.print(deviceAddress[i], HEX);
    }
}

void setup() {
    Serial.begin(9600);
    pinMode(RELAY_PIN, OUTPUT);

    lcd.begin(16, 2); // start the library
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("PID_Fridge 1.0");
    lcd.setCursor(0, 1);
    lcd.print("Starting...");

    delay(1000);

    sensors.begin();
    // Localiza e mostra enderecos dos sensores
    Serial.println("Localizando sensores DS18B20...");
    Serial.print("Foram encontrados ");
    Serial.print(sensors.getDeviceCount(), DEC);
    Serial.println(" sensores.");
    if (!sensors.getAddress(sensor1, 0))
        Serial.println("Sensores nao encontrados !");
    // Mostra o endereco do sensor encontrado no barramento
    Serial.println("Endereco sensor: ");
    mostra_endereco_sensor(sensor1);

    //PID
    Serial.println("Configurando PID...");
    windowStartTime = millis();
    //initialize the variables we're linked to
    Setpoint = 18;
    Serial.print("Setpoint: ");
    Serial.print(Setpoint);
    //tell the PID to range between 0 and the full window size
    myPID.SetOutputLimits(0, WindowSize);
    //turn the PID on
    myPID.SetMode(AUTOMATIC);

    Serial.println("Configurando...");
    config_pid();
}

void loop() {
    sensors.requestTemperatures();
    Input = sensors.getTempC(sensor1);
    myPID.Compute();

    running_lcd();

    processa_key_menu();

    /************************************************
     * turn the output pin on/off based on pid output
     ************************************************/
    if (millis() - windowStartTime > WindowSize) { //time to shift the Relay Window
        windowStartTime += WindowSize;
    }

    if (Output < millis() - windowStartTime) {
        digitalWrite(RELAY_PIN, HIGH);
        running_pid = 1;
        Serial.println("PID: RELAY_PIN agora é HIGH");
        last_rl_off = 0;

    } else {
        digitalWrite(RELAY_PIN, LOW);
        Serial.println("PID: RELAY_PIN agora é LOW");
        //if(running_pid ==1){
        last_rl_off = millis();
        Serial.println("PID: Pause");
        // }
        running_pid = 2;

        while (running_pid == 2) {
            sensors.requestTemperatures();
            Input = sensors.getTempC(sensor1);
            running_lcd();

            processa_key_menu();

            Serial.print(millis());
            Serial.print(" - ");
            Serial.print(last_rl_off);
            Serial.print(" = ");
            Serial.print(millis() - last_rl_off);

            Serial.print(" => ");
            Serial.println(pause_after_off * 1000);

            lcd_key = read_LCD_buttons();

            if ((millis() - last_rl_off) > (pause_after_off * 1000)) {
                running_pid = 0;
            } else {
                delay(500);
            }
        }
    }
}


void processa_key_menu() {
    lcd_key = read_LCD_buttons();

    switch (lcd_key) {
    case btnRIGHT:
        {
            Serial.println("Key >");
            config_pid();
            break;
        }
    }

}

void running_lcd() {
    lcd.setCursor(13, 0);

    if (running_pid == 0) {
        lcd.print("Off ");
    } else if (running_pid == 2) {
        lcd.print("|| ");
    } else {
        lcd.print("On ");
    }

    lcd.setCursor(0, 1);
    lcd.print("S:");
    lcd.print(Setpoint);
    lcd.print("C R:");
    lcd.print(Input);
    lcd.print("C      ");
}

void config_pid() {
    int configure = 1;
    Serial.println("config_pid()");

    Serial.println("SetPt");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SetPt [UP DW SL]");
    lcd_key = read_LCD_buttons();
    while (configure > 0) {
        switch (lcd_key) {
        case btnUP:
            {
                Setpoint = Setpoint + 0.5;
                Serial.print("Key UP +0,5 ");
                Serial.println(Setpoint);
                break;
            }
        case btnDOWN:
            {
                Setpoint = Setpoint - 0.5;
                Serial.print("Key DW -0,5 ");
                Serial.println(Setpoint);
                break;
            }
        case btnSELECT:
            {
                configure = 0;
                Serial.println("Key SL");
                break;
            }
        }
        lcd.setCursor(0, 1);
        lcd.print("SetPt: ");
        lcd.print(Setpoint);
        lcd.print("C      ");
        delay(200);
        lcd_key = read_LCD_buttons();
    }

    configure = 1;
    Serial.println("WinSize");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("WinSize [UP DW SL]");
    lcd_key = read_LCD_buttons();
    while (configure > 0) {
        switch (lcd_key) {
        case btnUP:
            {
                WindowSize++;
                Serial.print("Key UP +1 ");
                Serial.println(WindowSize);
                break;
            }
        case btnDOWN:
            {
                WindowSize--;
                Serial.print("Key DW -1 ");
                Serial.println(WindowSize);
                break;
            }
        case btnSELECT:
            {
                configure = 0;
                Serial.println("Key SL");
                break;
            }
        }
        lcd.setCursor(0, 1);
        lcd.print("WinSize: ");
        lcd.print(WindowSize);
        lcd.print(" ");
        delay(200);
        lcd_key = read_LCD_buttons();
    }

    configure = 1;
    Serial.println("Kp");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Kp [UP DW SL]");
    lcd_key = read_LCD_buttons();
    while (configure > 0) {
        switch (lcd_key) {
        case btnUP:
            {
                Kp++;
                Serial.print("Key UP +1 ");
                Serial.println(Kp);
                break;
            }
        case btnDOWN:
            {
                Kp--;
                Serial.print("Key DW -1 ");
                Serial.println(Kp);
                break;
            }
        case btnSELECT:
            {
                configure = 0;
                Serial.println("Key SL");
                break;
            }
        }
        lcd.setCursor(0, 1);
        lcd.print("Kp: ");
        lcd.print(Kp);
        lcd.print(" ");
        delay(200);
        lcd_key = read_LCD_buttons();
    }

    configure = 1;
    Serial.println("Ki");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Ki [UP DW SL]");
    lcd_key = read_LCD_buttons();
    while (configure > 0) {
        switch (lcd_key) {
        case btnUP:
            {
                Ki++;
                Serial.print("Key UP +1 ");
                Serial.println(Ki);
                break;
            }
        case btnDOWN:
            {
                Ki--;
                Serial.print("Key DW -1 ");
                Serial.println(Ki);
                break;
            }
        case btnSELECT:
            {
                configure = 0;
                Serial.println("Key SL");
                break;
            }
        }
        lcd.setCursor(0, 1);
        lcd.print("Ki: ");
        lcd.print(Ki);
        lcd.print(" ");
        delay(200);
        lcd_key = read_LCD_buttons();
    }

    configure = 1;
    Serial.println("Kd");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Kd [UP DW SL]");
    lcd_key = read_LCD_buttons();
    while (configure > 0) {
        switch (lcd_key) {
        case btnUP:
            {
                Kd++;
                Serial.print("Key UP +1 ");
                Serial.println(Kd);
                break;
            }
        case btnDOWN:
            {
                Kd--;
                Serial.print("Key DW -1 ");
                Serial.println(Kd);
                break;
            }
        case btnSELECT:
            {
                configure = 0;
                Serial.println("Key SL");
                break;
            }
        }
        lcd.setCursor(0, 1);
        lcd.print("Kd: ");
        lcd.print(Kd);
        lcd.print(" ");
        delay(200);
        lcd_key = read_LCD_buttons();
    }

    configure = 1;
    Serial.println("CfgPause");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CfgPause [UP DW SL]");
    lcd_key = read_LCD_buttons();
    while (configure > 0) {
        switch (lcd_key) {
        case btnUP:
            {
                pause_after_off++;
                Serial.print("Key UP +1 ");
                Serial.println(pause_after_off);
                break;
            }
        case btnDOWN:
            {
                pause_after_off--;
                Serial.print("Key DW -1 ");
                Serial.println(pause_after_off);
                break;
            }
        case btnSELECT:
            {
                configure = 0;
                Serial.println("Key SL");
                break;
            }
        }
        lcd.setCursor(0, 1);
        lcd.print("CfgPause: ");
        lcd.print(pause_after_off);
        lcd.print("s ");
        delay(200);
        lcd_key = read_LCD_buttons();
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Running [>]");

    Serial.println("Exit Config");
    Serial.print("Novo Setpoint: ");
    Serial.print(Setpoint);

    myPID.SetTunings(Kp, Ki, Kd);

}
