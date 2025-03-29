#include "EncoderReader.h"

EncoderReader::EncoderReader(ESP32Encoder* fl, ESP32Encoder* fr, ESP32Encoder* rl, ESP32Encoder* rr)
    : encoderFL(fl), encoderFR(fr), encoderRL(rl), encoderRR(rr)
{
}

EncoderData EncoderReader::readEncoders() {
    EncoderData data;
    data.frontLeft  = encoderFL->getCount();
    data.frontRight = encoderFR->getCount();
    data.rearLeft   = encoderRL->getCount();
    data.rearRight  = encoderRR->getCount();
    return data;
}

void EncoderReader::resetEncoders() {
    encoderFL->clearCount();
    encoderFR->clearCount();
    encoderRL->clearCount();
    encoderRR->clearCount();
}
