#ifndef LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED
#define LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED

// Uncomment next line to use old model GPIO mapping
//#define TTGO_TBEAM_OLDV 

// UPDATE WITH YOUR TTN KEYS AND ADDR.
static PROGMEM u1_t NWKSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }; // LoRaWAN NwkSKey, network session key 
static u1_t PROGMEM APPSKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00A6, 0x00 }; // LoRaWAN AppSKey, application session key 
static const u4_t DEVADDR = 0x00000000 ; // LoRaWAN end-device address (DevAddr)


#endif //LORA_TTNMAPPER_TBEAM_CONFIG_INCLUDED
