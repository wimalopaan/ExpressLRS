#ifndef RX_ENDPOINT_H
#define RX_ENDPOINT_H
#include "CRSFEndpoint.h"

#if defined(WMEXTENSION) && defined(TARGET_RX)
#include "../../src/rx_wmextension.h"
#endif

class RXEndpoint final : public CRSFEndpoint {
public:
    RXEndpoint();
    bool handleRaw(const crsf_header_t *message) override;
    void handleMessage(const crsf_header_t *message) override;

    void registerParameters() override;
    void updateParameters() override;

#if defined(WMEXTENSION) && defined(TARGET_RX)
    const MultiSwitch& multiSwitch() const;
#endif
private:
    void luaparamMappingChannelOut(propertiesCommon *item, uint8_t arg);
    void luaparamSetFailsafe(propertiesCommon *item, uint8_t arg);

#if defined(WMEXTENSION) && defined(TARGET_RX)
    MultiSwitch msw;
#endif
#if defined(HAS_GYRO)
    // Commands
    void luaparamGyroQuickPreset(propertiesCommon *item, uint8_t arg);
    void luaparamGyroCalibration(propertiesCommon *item, uint8_t arg);
    void luaparamGyroOrientationCal(propertiesCommon *item, uint8_t arg);
    void luaparamGyroStickCal(propertiesCommon *item, uint8_t arg);

    // Selections
    void luaparamGyroInputCh_Select(propertiesCommon *item, uint8_t arg);
    void luaparamGyroOutputCh_Select(propertiesCommon *item, uint8_t arg);
    void luaparamGyroPIG_AxisSelect(propertiesCommon *item, uint8_t arg);
    void luaparamGyroFMode_Select(propertiesCommon *item, uint8_t arg);

#endif

};

#endif //RX_ENDPOINT_H
