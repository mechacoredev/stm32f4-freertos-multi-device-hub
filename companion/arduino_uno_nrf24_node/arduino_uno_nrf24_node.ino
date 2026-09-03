#include <SPI.h>
#include <RF24.h>

static const uint8_t NRF_CE_PIN = 9;
static const uint8_t NRF_CSN_PIN = 10;
static const uint8_t LED_PIN = 4;
static const uint8_t RADIO_ADDRESS[5] = {
  0xE7, 0xE7, 0xE7, 0xE7, 0xE7
};

RF24 radio(NRF_CE_PIN, NRF_CSN_PIN);
uint32_t receivedPacketCount = 0;
uint32_t transmittedPacketCount = 0;
uint32_t transmitErrorCount = 0;
uint32_t nextTransmitMs = 0;

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  Serial.begin(115200);

  if (!radio.begin())
  {
    Serial.println(F("NRF24L01 bulunamadi"));
    while (true) {}
  }

  radio.setChannel(76);
  radio.setDataRate(RF24_1MBPS);
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(true);
  radio.setRetries(15, 15);
  radio.enableDynamicPayloads();
  radio.openWritingPipe(RADIO_ADDRESS);
  radio.openReadingPipe(1, RADIO_ADDRESS);
  radio.startListening();
  nextTransmitMs = millis() + 3000;
}

void loop()
{
  if (radio.available())
  {
    uint8_t payloadLength = radio.getDynamicPayloadSize();
    if (payloadLength == 0 || payloadLength > 32)
    {
      radio.flush_rx();
    }
    else
    {
      char payload[33] = {0};
      radio.read(payload, payloadLength);
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      receivedPacketCount++;
      Serial.print(F("F407 -> UNO: "));
      Serial.println(payload);
    }
  }

  uint32_t now = millis();
  if ((int32_t)(now - nextTransmitMs) >= 0)
  {
    nextTransmitMs = now + 3000;
    char message[32];
    uint8_t length = (uint8_t)snprintf(message, sizeof(message),
                                      "UNO MSG %lu",
                                      (unsigned long)(transmittedPacketCount + 1));

    radio.stopListening();
    bool sent = radio.write(message, length);
    radio.startListening();

    if (sent) transmittedPacketCount++;
    else transmitErrorCount++;
  }
}
