#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define RPM_INTERRUPT_PIN       0
#define RPM_PIN                 2
#define VOLTAGE_PIN             A1
#define CURRENT_PIN             A0

#define VOLTAGE_CONSTANTA       5
#define CURRENT_SENSITIVITY     0.185
#define CURRENT_OFFSET          2.5

#define SENSOR_LOOP             10
#define SENSOR_DELAY            1000
#define DISPLAY_DELAY           2000
#define LED_DELAY               1000

typedef struct{
    float voltage;
    float current;
    float rpm;
}DataTypeDef;

void rpm_counter(void);
void baca_sensor(void);
void cetak_data(void);
void cetak_judul(void);

uint32_t rpm_count = 0;

uint32_t rpm_time;
uint32_t sensor_time;
uint32_t display_time;
uint32_t led_time;

uint8_t led_state = LOW;

DataTypeDef data_sensor;

void setup(){
    Serial.begin(9600);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(RPM_PIN, INPUT_PULLUP);

    // attachInterrupt(RPM_INTERRUPT_PIN, rpm_counter, RISING);

    lcd.init();
    lcd.init();
    lcd.backlight();
    
    cetak_judul();
}

void loop(){

    /* update sensor setiap detik */
    if((millis() - sensor_time) > SENSOR_DELAY){
        baca_sensor();

        sensor_time = millis();
    }

    /* update display setiak 2 detik */
    if((millis() - display_time) > DISPLAY_DELAY){
        cetak_data();

        display_time = millis();
    }

    /* blink LED */
    if((millis() - led_time) > LED_DELAY){
        led_state = ~led_state;

        digitalWrite(LED_BUILTIN, led_state);

        led_time = millis();
    }
}

void rpm_counter(void){
    rpm_count++;
};

void baca_sensor(void){
    float pinvolt=0;
    float pinArus=0;

    for(uint8_t n=0; n<SENSOR_LOOP; n++){
        
        /* Baca Tegangan */
        uint16_t voltRaw = analogRead(VOLTAGE_PIN);      
        pinvolt += (float) (5.0 * voltRaw) / 1024.0;  

        /* Baca Arus */
        uint16_t arusRaw = analogRead(CURRENT_PIN);
        pinArus += (float) (5.0 * arusRaw) / 1024.0;

        delay(10);  
    }

    /* Hitung Tegangan */   
    data_sensor.voltage = (float) (pinvolt / SENSOR_LOOP) * 5;

    /* Hitung Arus */
    data_sensor.current = (float) ((pinArus / SENSOR_LOOP) - CURRENT_OFFSET) / CURRENT_SENSITIVITY;

    /* Baca RPM */
    // detachInterrupt(RPM_INTERRUPT_PIN);

    // Serial.println(rpm_count);
    // Serial.println((millis() - sensor_time));
    
    // float rpm = (float) rpm_count * (60000 / (millis() - sensor_time));

    // rpm_count = 0;

    // attachInterrupt(RPM_INTERRUPT_PIN, rpm_counter, RISING);

    unsigned long rpm_pulse = pulseIn(RPM_PIN, LOW, 1000000);

    float rpm;
    if(rpm_pulse > 0){
        rpm = (float) 60000000 / rpm_pulse;
    }
    else{
        rpm = 0.0;
    }

    data_sensor.rpm = (data_sensor.rpm + rpm) / 2;

    String terminal = "V: " + String(data_sensor.voltage, 3) + " I: " + String(data_sensor.current, 3) + " rpm: " + String(data_sensor.rpm, 0);
    Serial.println(terminal);
};

void cetak_data(void){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("V:" + String(data_sensor.voltage, 1));

    lcd.setCursor(8,0);
    lcd.print("I:" + String(data_sensor.current, 1));

    lcd.setCursor(0,1);
    lcd.print("rpm:" + String(data_sensor.rpm, 0));
};

void cetak_judul(void){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Judul");
    lcd.setCursor(0,1);
    lcd.print("Mahasiswa");

    delay(3000);
};