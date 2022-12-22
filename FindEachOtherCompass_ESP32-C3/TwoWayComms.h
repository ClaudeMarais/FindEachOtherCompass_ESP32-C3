#ifndef _TWO_WAY_COMMS
#define _TWO_WAY_COMMS

// 2-way communication between ESP's
#include <ESP_Now.h>
#include <WiFi.h>

#define OTHER_DEVICE (1 - DEVICE)

// You can find the MAC address of a device using "Serial.println(WiFi.macAddress());"
#if DEVICE == 0
uint8_t OtherDeviceMACAddress[] = {0xA0, 0x76, 0x4E, 0x43, 0x87, 0x20}; // The MAC address of device 1
#else
uint8_t OtherDeviceMACAddress[] = {0xA0, 0x76, 0x4E, 0x44, 0x8F, 0xD4}; // The MAC address of device 0
#endif

namespace TwoWayComms
{
  typedef struct Data
  {
    double longitude;
    double latitude;
    bool bIsLocationRecent;   // Is this the actual current location from the GPS or a loaded previously known location?
#ifdef DEBUG_TWO_WAY_COMMS
    char deviceID;
#endif
  } Data;

  Data thisDeviceData;
  Data otherDeviceData;
  Data incomingData;

  bool bIsConnected = false;
  bool bDataReceived = false;

#ifdef DEBUG_TWO_WAY_COMMS
  char receivedDataCounter = 0;
#endif

  // Check every N seconds if still connected
  unsigned long heartBeatTimer = 0;
#ifdef SAVE_POWER
  unsigned long heartBeatDelay = 12000;
#else
  unsigned long heartBeatDelay = 6000;
#endif

  esp_now_peer_info_t peerInfo;

  // Callback when data is sent
  void OnDataSent(const uint8_t* pMacAddr, esp_now_send_status_t status)
  {
    if (status == ESP_NOW_SEND_SUCCESS)
    {
#ifdef DEBUG_TWO_WAY_COMMS
      if (!bIsConnected)
      {
        DebugPrintf("Not connected => Connected\nSuccessfully delivered to other device %d\n", OTHER_DEVICE);
      }
#endif

      bIsConnected = true;
    }
  }

  // Callback when data is received
  void OnDataReceived(const uint8_t* pMacAddr, const uint8_t* pIncomingData, int length)
  {
    memcpy(&incomingData, pIncomingData, sizeof(incomingData));

#ifdef DEBUG_TWO_WAY_COMMS
    if (!bIsConnected)
    {
      DebugPrintf("Not connected => Connected\tSuccessfully delivered to other device %d\n", OTHER_DEVICE);
    }
#endif

    bDataReceived = true;
    bIsConnected = true;
    heartBeatTimer = millis();

    otherDeviceData.longitude = incomingData.longitude;
    otherDeviceData.latitude = incomingData.latitude;
    otherDeviceData.bIsLocationRecent = incomingData.bIsLocationRecent;

#ifdef DEBUG_TWO_WAY_COMMS
    otherDeviceData.deviceID = incomingData.deviceID;
    receivedDataCounter++;
    DebugPrintf("Received data from other device %d, heartbeat counter %d\n", otherDeviceData.deviceID, receivedDataCounter);
#endif
  }

  bool Setup()
  {
    DebugPrintln("Setting up two-way communication...");

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK)
    {
      DebugPrintln("ERROR: Could not initialize ESP-NOW!");
      return false;
    }
   
    // Register peer
    memcpy(peerInfo.peer_addr, OtherDeviceMACAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK)
    {
      DebugPrintln("ERROR: Failed to add peer");
      return false;
    }

    // Register callbacks
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataReceived);

    heartBeatTimer = millis();

    DebugPrintln("Two-way communication setup succeeded.");
    DebugPrintf("I am device %d\n", DEVICE);

    return true;
  }
  
  bool SendMyLocation(const double longitude, const double latitude, const bool bIsLocationRecent)
  {
    thisDeviceData.longitude = longitude;
    thisDeviceData.latitude = latitude;
    thisDeviceData.bIsLocationRecent = bIsLocationRecent;

#ifdef DEBUG_TWO_WAY_COMMS
    thisDeviceData.deviceID = DEVICE;
#endif

    // Send message via ESP-NOW
    esp_err_t result = esp_now_send(OtherDeviceMACAddress, (uint8_t*)&thisDeviceData, sizeof(thisDeviceData));
    
    if (result != ESP_OK)
    {
#ifdef DEBUG_TWO_WAY_COMMS
      DebugPrintln("Error sending the data");
#endif
      return false;
    }

    return true;
  }

  void UpdateStatus(const unsigned long now)
  {
    if ((now - heartBeatTimer) > heartBeatDelay)
    {
      bIsConnected = false;
      heartBeatTimer = now;
      bDataReceived = false;

#ifdef DEBUG_TWO_WAY_COMMS
      DebugPrintf("No data received from other device %d for %d seconds\n", OTHER_DEVICE, int(heartBeatDelay / 1000));
#endif
    }
  }

  inline bool IsConnected() { return bIsConnected; }

}

#endif