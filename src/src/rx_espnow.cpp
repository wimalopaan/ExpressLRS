#if defined(WMEXTENSION) && defined(WMESPNOW) && defined(TARGET_RX)

#include "rx_espnow.h"

#include <cstring>

#include "common.h"
#include "config.h"
#include "logging.h"

extern RxConfig config;

namespace
{
bool initializeDevice()
{
    return EspNowConnector::initialize();
}

int startDevice()
{
    if (!EspNowConnector::enabled())
        return DURATION_NEVER;
    espNow.start();
    return DURATION_IMMEDIATELY;
}

int eventDevice()
{
    espNow.onDeviceEvent();
    return DURATION_IGNORE;
}

int timeoutDevice()
{
    espNow.poll();
    return DURATION_IMMEDIATELY;
}

void onRadioReceived(const uint8_t *const mac, const uint8_t *const data, const size_t size)
{
    espNow.onReceive(mac, data, size);
}

void onRadioSent(const bool success)
{
    espNow.onSent(success);
}
} // namespace

device_t ESPNOW_device = {
    .initialize = initializeDevice,
    .start = startDevice,
    .event = eventDevice,
    .timeout = timeoutDevice,
    .subscribe = EVENT_ENTER_BIND_MODE | EVENT_EXIT_BIND_MODE | EVENT_CONFIG_UID_CHANGED | EVENT_CONNECTION_CHANGED,
};

void EspNowConnector::forwardMessage(const crsf_header_t *const message)
{
    if (!running() || message->type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED)
        return;
    const auto *data = reinterpret_cast<const uint8_t *>(message);
    const size_t size = size_t(message->frame_size) + 2;
    if (!EspNowProtocol::validFrame(data, size))
    {
        ERRLN("ESP-NOW invalid outgoing CRSF frame");
        return;
    }
    events.pushFrame(data, size);
}

void EspNowConnector::onReceive(const uint8_t *const source, const uint8_t *const data, const size_t size)
{
    if (running())
        events.pushReceived(source, micros(), data, size);
}

void EspNowConnector::onSent(const bool success)
{
    if (running())
        events.pushSent(micros(), success);
}

void EspNowConnector::start()
{
    if (!enabled())
        return;
    lastStartAttempt = millis();
    if (running())
        return;
    state = State::WaitingToStart;
    if (!config.GetIsBound() || InBindingMode)
        return;
    std::memcpy(uid.data(), config.GetUID(), uid.size());
    if (!radio.start(mac, onRadioReceived, onRadioSent))
        return;
    events.clear();
    knownPeers = {};
    for (uint8_t i = 0; i < EspNowProtocol::MaxSlaves; ++i)
        removeDevice(static_cast<crsf_addr_e>(EspNowProtocol::slaveAddress(i)));
    link.reset(uid, mac, EspNowRadio::newSession(), micros());
    reportedErrors = 0;
    state = State::Ready;
}

void EspNowConnector::stop()
{
    const bool wasRunning = running();
    state = State::Stopped;
    if (wasRunning)
        radio.stop();
}

void EspNowConnector::poll()
{
    switch (state)
    {
    case State::Stopped:
        return;
    case State::WaitingToStart:
        if (uint32_t(millis() - lastStartAttempt) >= 1000)
            start();
        return;
    case State::Ready:
        if (connectionState > MODE_STATES)
            return;
        if (InBindingMode || !config.GetIsBound())
        {
            waitForBinding();
            return;
        }
        if (uidChanged())
            applyUidChange();
        processQueuedEvents();
        tick();
        return;
    }
}

void EspNowConnector::onDeviceEvent()
{
    if (state == State::Stopped)
        return;
    if (connectionState > MODE_STATES)
    {
        stop();
        return;
    }
    if (InBindingMode || !config.GetIsBound())
    {
        if (running())
            waitForBinding();
        return;
    }
    if (state == State::WaitingToStart)
        start();
    else if (uidChanged())
        applyUidChange();
}

void EspNowConnector::processQueuedEvents()
{
    EspNowEvents::Event event;
    for (unsigned n = 0; n < 32 && events.pop(event); ++n)
    {
        switch (event.type)
        {
        case EspNowEvents::Type::Frame:
            link.enqueue(event.data.data(), event.size);
            break;
        case EspNowEvents::Type::Received:
            link.receive(event.source, event.data.data(), event.size, event.time);
            break;
        case EspNowEvents::Type::Sent:
            link.sent(event.time, event.success);
            break;
        }
        updateConnection(micros());
        EspNowProtocol::Frame frame;
        while (link.popFrame(frame))
            deliver(frame);
    }
}

void EspNowConnector::waitForBinding()
{
    stop();
    state = State::WaitingToStart;
    lastStartAttempt = millis();
    link.reset(uid, mac, EspNowRadio::newSession(), micros());
    updateConnection(micros());
}

bool EspNowConnector::uidChanged() const
{
    return std::memcmp(uid.data(), config.GetUID(), uid.size()) != 0;
}

void EspNowConnector::applyUidChange()
{
    if (!link.sending())
    {
        std::memcpy(uid.data(), config.GetUID(), uid.size());
        link.reset(uid, mac, EspNowRadio::newSession(), micros());
    }
}

void EspNowConnector::tick()
{
    const uint32_t now = micros();
    link.expire(now);
    updateConnection(now);
    updatePeers();
    prepareTick();
    EspNowProtocol::Packet packet;
    if (!uidChanged() && link.tick(now, packet) && !radio.send(packet))
        link.sent(micros(), false);
    reportErrors();
}

void EspNowConnector::reportErrors()
{
    const uint32_t totalErrors = events.dropped() + link.errors();
    if (totalErrors != reportedErrors && uint32_t(millis() - lastErrorReport) >= 1000)
    {
        ERRLN("ESP-NOW dropped/invalid/failed packets: %u", totalErrors);
        reportedErrors = totalErrors;
        lastErrorReport = millis();
    }
}
#endif
