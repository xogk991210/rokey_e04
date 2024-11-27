#define PIN_ENA   8
#define PIN_DIR   9
#define PIN_PUL   10

char num = '0'; // 초기값은 '0'으로 설정하여 모터 멈춤 상태

void setup() {
    Serial.begin(115200);
    Serial.write('s');

    pinMode(PIN_ENA, OUTPUT);
    pinMode(PIN_DIR, OUTPUT);
    pinMode(PIN_PUL, OUTPUT);

    digitalWrite(PIN_ENA, LOW); // 모터 활성화
    digitalWrite(PIN_DIR, LOW); // 방향 초기화
    digitalWrite(PIN_PUL, LOW); // 초기 상태
}

void loop() {
    // 시리얼로 데이터 읽기
    if (Serial.available() > 0) {
        char incomingByte = Serial.read();
        if (incomingByte == '1' || incomingByte == '0') {
            num = incomingByte; // 유효한 값만 저장
        }
    }

    // "1"이면 모터 회전
    if (num == '1') {
        digitalWrite(PIN_PUL, HIGH);
        delayMicroseconds(1000); // 펄스 신호 유지 시간
        digitalWrite(PIN_PUL, LOW);
        delayMicroseconds(1000);
    }
    // "0"이면 모터 멈춤
    else if (num == '0') {
        digitalWrite(PIN_PUL, LOW); // LOW 상태로 유지
        delayMicroseconds(2000);
    }
}
