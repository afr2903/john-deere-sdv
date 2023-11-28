const int in_1 = 11;
const int in_2 = 10;
const int enable = 9;

const int encoder_a = 2;
const int encoder_b = 3;
const bool pwm_enabled = true;

int pwm = -1;
int ticks = 0;

unsigned long last_time = 0;

int active = 0;

void setup() {
    pinMode(in_1, OUTPUT);
    pinMode(in_2, OUTPUT);
    pinMode(enable, OUTPUT);
    pinMode(encoder_a, INPUT_PULLUP);
    pinMode(encoder_b, INPUT_PULLUP);
    
    attachInterrupt(digitalPinToInterrupt(encoder_a), encoder_a_interrupt, CHANGE);

    Serial.begin(9600);
    delay(5000);
}

void loop() {
    if (millis() - last_time > 4000 && pwm_enabled) {
        if (pwm == -1)
            pwm = 80;
        else if( pwm == 80)
            pwm = 255;
        else if (pwm == 255)
            pwm = -255;
        else if (pwm == -255)
            pwm = 0;
            
        digitalWrite(in_1, pwm >= 0 ? HIGH : LOW);
        digitalWrite(in_2, pwm > 0 ? LOW : HIGH);
        analogWrite(enable, abs(pwm));

        last_time = millis();
    }

    Serial.println( String(pwm) );
    Serial.println( String(ticks) );
    Serial.flush();
    delay(100);
}

void encoder_a_interrupt() {
    if (digitalRead(encoder_a) == HIGH) {
        if (digitalRead(encoder_b) == LOW) {
            ticks++;
        } else {
            ticks--;
        }
    } else {
        if (digitalRead(encoder_b) == LOW) {
            ticks--;
        } else {
            ticks++;
        }
    }
}


