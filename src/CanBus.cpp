#include "CanBus.h"

CanBus::CanBus(VescData *vescData) {
    this->vescData = vescData;
    this->stream = new LoopbackStream(BUFFER_SIZE);
}

void CanBus::init() {
    vesc_id = AppConfiguration::getInstance()->config.vescId;
    esp_can_id = vesc_id + 1;
    ble_proxy_can_id = vesc_id + 2;
    RECV_STATUS_1 = (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_STATUS) << 8) + this->vesc_id;
    RECV_STATUS_2 = (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_STATUS_2) << 8) + this->vesc_id;
    RECV_STATUS_3 = (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_STATUS_3) << 8) + this->vesc_id;
    RECV_STATUS_4 = (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_STATUS_4) << 8) + vesc_id;
    RECV_STATUS_5 = (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_STATUS_5) << 8) + vesc_id;

    RECV_FILL_RX_BUFFER = (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_FILL_RX_BUFFER) << 8) + esp_can_id;
    RECV_PROCESS_RX_BUFFER = (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_PROCESS_RX_BUFFER) << 8) + esp_can_id;

    RECV_PROCESS_SHORT_BUFFER_PROXY =
            (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_PROCESS_SHORT_BUFFER) << 8) + ble_proxy_can_id;
    RECV_FILL_RX_BUFFER_PROXY =
            (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_FILL_RX_BUFFER) << 8) + ble_proxy_can_id;
    RECV_FILL_RX_BUFFER_LONG_PROXY =
            (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_FILL_RX_BUFFER_LONG) << 8) + ble_proxy_can_id;
    RECV_PROCESS_RX_BUFFER_PROXY =
            (uint32_t(0x0000) << 16) + (uint16_t(CAN_PACKET_PROCESS_RX_BUFFER) << 8) + ble_proxy_can_id;
    candevice = new CanDevice();
    candevice->init();
    proxy = new BleCanProxy(candevice, stream, vesc_id, ble_proxy_can_id);
}

/*
  The VESC has to be configured to send status 1-5 regularly. It is recommenden to reduce the
  interval from 50Hz to something around 1-5Hz, which is absolutely sufficient for this application.
*/
void CanBus::loop() {
    int frameCount = 0;
    twai_message_t rx_frame;
    unsigned long now = millis();
    if (initialized) {
        if (lastRealtimeData <= now && now - lastRealtimeData > interval && !proxy->processing) {
            if(!requestRealtimeData()) {
                lastRealtimeData = (millis() + 500);
                this->vescData->connected = false;
            }
        }

        if (lastBalanceData <= now && now - lastBalanceData > interval && !proxy->processing) {
            if(!requestFloatPackageData()) {
                lastBalanceData = (millis() + 500);
                this->vescData->connected = false;
            }
        }
    } else if(initRetryCounter > 0 && lastRetry <= now && now - lastRetry > 500) {
        requestFirmwareVersion();
        initRetryCounter--;
        lastRetry = millis();
        if(initRetryCounter == 0) {
            Logger::error("CANBUS initialization failed");
            initRetryCounter = 1;
            lastRetry = (millis() + 5000);
        }
    }

    //receive next CAN frame from queue
    while (twai_receive( &rx_frame, 3 * portTICK_PERIOD_MS) == ESP_OK) {
        if(!this->vescData->connected) {
            this->vescData->connected = true;
        }
        if (!initialized) {
            Logger::notice(LOG_TAG_CANBUS, "CANBUS is now initialized");
            initialized = true;
        }
        frameCount++;
        //VESC only uses ext packages, so skip std packages
        if (rx_frame.extd) {
            if (Logger::getLogLevel() == Logger::VERBOSE) {
                printFrame(rx_frame, frameCount);
            }
            processFrame(rx_frame, frameCount);
        }
        clearFrame(rx_frame);
        if (frameCount > 1000) {
            // WORKAROUND if messages arrive too fast
            Logger::error(LOG_TAG_CANBUS, "reached 1000 frames in one loop, abort");
            buffer.clear();
            return;
        }
    }
    if (Logger::getLogLevel() == Logger::VERBOSE) {
        dumpVescValues();
    }
}

boolean CanBus::isInitialized()
{
    return initialized;
}

int CanBus::getInterval()
{
    return interval;
}

boolean CanBus::bmsVTOT(float v_tot, float v_charge) {
   twai_message_t tx_frame = {};
    int32_t send_index = 0;
    uint8_t buffer[8];  // 8 bytes for two 32-bit floats
    
    // Prepare the payload
    buffer_append_float32_auto(buffer, v_tot, &send_index); //v_tot
    buffer_append_float32_auto(buffer, v_charge, &send_index); //v_charge

    // Configure CAN frame
    tx_frame.extd = 1;
    tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_BMS_V_TOT) << 8) + vesc_id;
    tx_frame.data_length_code = 0x8;  //sending 8 bytes

    memcpy(&tx_frame.data[0], buffer, 8);

    // Send CAN frame
    return candevice->sendCanFrame(&tx_frame);
}

    /*
	 * CAN_PACKET_BMS_SOC_SOH_TEMP_STAT
	 *
	 * b[0] - b[1]: V_CELL_MIN (mV)
	 * b[2] - b[3]: V_CELL_MAX (mV)
	 * b[4]: SoC (0 - 255)
	 * b[5]: SoH (0 - 255)
	 * b[6]: T_CELL_MAX (-128 to +127 degC)
	 * b[7]: State bitfield:
	 * [B7      B6      B5      B4      B3      B2      B1      B0      ]
	 * [RSV     RSV     RSV     RSV     RSV     CHG_OK  IS_BAL  IS_CHG  ]
	 */
boolean CanBus::bmsSOCSOHTempStat(float vCellMin, float vCellMax, float SOC, float SOH, float cellMaxTemp, boolean isCharging, boolean isBalancing, boolean isChargeAllowed, boolean isChargeOk) {
    twai_message_t tx_frame = {};
    int32_t send_index = 0;
    uint8_t buffer[8];  // 8 bytes for two 32-bit floats
    
    // Prepare the payload
    buffer_append_float16(buffer, vCellMin, 1e3, &send_index);
	buffer_append_float16(buffer, vCellMax, 1e3, &send_index);
	buffer[send_index++] = (uint8_t)(SOC / 100 * 255);
	buffer[send_index++] = (uint8_t)(SOH / 100 * 255);
	buffer[send_index++] = (int8_t)cellMaxTemp;
	buffer[send_index++] =
			((isCharging ? 1 : 0) << 0) |
			((isBalancing ? 1 : 0) << 1) |
			((isChargeAllowed ? 1 : 0) << 2) |
			((isChargeOk ? 1 : 0) << 3);

    // Configure CAN frame
    tx_frame.extd = 1;
    tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_BMS_SOC_SOH_TEMP_STAT) << 8) + vesc_id;
    tx_frame.data_length_code = 0x8;  //sending 8 bytes

    memcpy(&tx_frame.data[0], buffer, 8);

    // Send CAN frame
    return candevice->sendCanFrame(&tx_frame);
}

boolean CanBus::bmsAHWHDischargeTotal(float ampHours, float wattHours) {
   twai_message_t tx_frame = {};
    int32_t send_index = 0;
    uint8_t buffer[8];  // 8 bytes for two 32-bit floats
    buffer_append_float32_auto(buffer, ampHours, &send_index);
	buffer_append_float32_auto(buffer, wattHours, &send_index);
    // Configure CAN frame
    tx_frame.extd = 1;
    tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_BMS_AH_WH_DIS_TOTAL) << 8) + vesc_id;
    tx_frame.data_length_code = 0x8;  //sending 8 bytes

    memcpy(&tx_frame.data[0], buffer, 8);
    // Send CAN frame
    return candevice->sendCanFrame(&tx_frame);
}

boolean CanBus::bmsAHWHChargeTotal(float ampHours, float wattHours) {
   twai_message_t tx_frame = {};
    int32_t send_index = 0;
    uint8_t buffer[8];  // 8 bytes for two 32-bit floats
    buffer_append_float32_auto(buffer, ampHours, &send_index);
	buffer_append_float32_auto(buffer, wattHours, &send_index);
    // Configure CAN frame
    tx_frame.extd = 1;
    tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_BMS_AH_WH_CHG_TOTAL) << 8) + vesc_id;
    tx_frame.data_length_code = 0x8;  //sending 8 bytes

    memcpy(&tx_frame.data[0], buffer, 8);
    // Send CAN frame
    return candevice->sendCanFrame(&tx_frame);
}

boolean CanBus::bmsAHWH(float ampHours, float wattHours) {
   twai_message_t tx_frame = {};
    int32_t send_index = 0;
    uint8_t buffer[8];  // 8 bytes for two 32-bit floats
    buffer_append_float32_auto(buffer, ampHours, &send_index);
	buffer_append_float32_auto(buffer, wattHours, &send_index);
    // Configure CAN frame
    tx_frame.extd = 1;
    tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_BMS_AH_WH) << 8) + vesc_id;
    tx_frame.data_length_code = 0x8;  //sending 8 bytes

    memcpy(&tx_frame.data[0], buffer, 8);
    // Send CAN frame
    return candevice->sendCanFrame(&tx_frame);
}

boolean CanBus::bmsI(float currentAmps) {
   twai_message_t tx_frame = {};
    int32_t send_index = 0;
    uint8_t buffer[8];  // 8 bytes for two 32-bit floats
    buffer_append_float32_auto(buffer, currentAmps, &send_index);
	buffer_append_float32_auto(buffer, currentAmps, &send_index);
    // Configure CAN frame
    tx_frame.extd = 1;
    tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_BMS_I) << 8) + vesc_id;
    tx_frame.data_length_code = 0x8;  //sending 8 bytes

    memcpy(&tx_frame.data[0], buffer, 8);
    // Send CAN frame
    return candevice->sendCanFrame(&tx_frame);
}

boolean CanBus::bmsBal(boolean isBalancing) {
   twai_message_t tx_frame = {};
    int32_t send_index = 0;
    uint8_t buffer[8];  // 8 bytes for two 32-bit floats
	buffer[send_index++] = 15;

	uint64_t bal_state = 0x0;
    if (isBalancing) bal_state=0xFFFFFFFFFFFFFFFF;
	buffer[send_index++] = (bal_state >> 48) & 0xFF;
	buffer[send_index++] = (bal_state >> 40) & 0xFF;
	buffer[send_index++] = (bal_state >> 32) & 0xFF;
	buffer[send_index++] = (bal_state >> 24) & 0xFF;
	buffer[send_index++] = (bal_state >> 16) & 0xFF;
	buffer[send_index++] = (bal_state >> 8) & 0xFF;
	buffer[send_index++] = (bal_state >> 0) & 0xFF;
    // Configure CAN frame
    tx_frame.extd = 1;
    tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_BMS_BAL) << 8) + vesc_id;
    tx_frame.data_length_code = 0x8;  //sending 8 bytes

    memcpy(&tx_frame.data[0], buffer, 8);
    // Send CAN frame
    return candevice->sendCanFrame(&tx_frame);
}

boolean CanBus::bmsVCell(const uint16_t* cellMillivolts, int cell_max) {
	int cell_now = 0;
	while (cell_now < cell_max) {
        twai_message_t tx_frame = {};
        // Configure CAN frame
        tx_frame.extd = 1;
        tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_BMS_V_CELL) << 8) + vesc_id;
        tx_frame.data_length_code = 0x8;  //sending 8 bytes
        uint8_t buffer[8];
		int32_t send_index = 0;
		buffer[send_index++] = cell_now;
		buffer[send_index++] = cell_max;
		if (cell_now < cell_max) {
			buffer_append_float16(buffer, cellMillivolts[cell_now++]/1000.0f, 1e3, &send_index);
		}
		if (cell_now < cell_max) {
			buffer_append_float16(buffer, cellMillivolts[cell_now++]/1000.0f, 1e3, &send_index);
		}
		if (cell_now < cell_max) {
			buffer_append_float16(buffer, cellMillivolts[cell_now++]/1000.0f, 1e3, &send_index);
		}
        memcpy(&tx_frame.data[0], buffer, 8);
        // Send CAN frame
        candevice->sendCanFrame(&tx_frame);
    }
    return true;
}

boolean CanBus::bmsTemps(const int8_t* thermTemps, int temp_max) {
    int temp_now = 0;
	while (temp_now < temp_max) {
        twai_message_t tx_frame = {};
        // Configure CAN frame
        tx_frame.extd = 1;
        tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_BMS_TEMPS) << 8) + vesc_id;
        tx_frame.data_length_code = 0x8;  //sending 8 bytes
        uint8_t buffer[8];
		int32_t send_index = 0;
		buffer[send_index++] = temp_now;
		buffer[send_index++] = temp_max;
		if (temp_now < temp_max) {
			buffer_append_float16(buffer, thermTemps[temp_now++], 1e2, &send_index);
		}
		if (temp_now < temp_max) {
			buffer_append_float16(buffer, thermTemps[temp_now++], 1e2, &send_index);
		}
		if (temp_now < temp_max) {
			buffer_append_float16(buffer, thermTemps[temp_now++], 1e2, &send_index);
		}
        memcpy(&tx_frame.data[0], buffer, 8);
        // Send CAN frame
        candevice->sendCanFrame(&tx_frame);		
    }
    twai_message_t tx_frame = {};
    // Configure CAN frame
    tx_frame.extd = 1;
    tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_BMS_HUM) << 8) + vesc_id;
    tx_frame.data_length_code = 0x8;  //sending 8 bytes
    uint8_t buffer[8];
	int32_t send_index = 0;
	buffer_append_float16(buffer, thermTemps[4], 1e2, &send_index);
	buffer_append_float16(buffer, 0.0f, 1e2, &send_index);
	buffer_append_float16(buffer, thermTemps[4], 1e2, &send_index); // Put IC temp here instead of making mew msg
    memcpy(&tx_frame.data[0], buffer, 8);
    candevice->sendCanFrame(&tx_frame);
    return true;
}

boolean CanBus::bmsState(bms_op_state op_state, bms_fault_state fault_state) {
    twai_message_t tx_frame = {};
    // Configure CAN frame
    tx_frame.extd = 1;
    tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_BMS_STATE) << 8) + vesc_id;
    tx_frame.data_length_code = 0x8;  //sending 8 bytes
    uint8_t buffer[8];
	int32_t send_index = 0;
	buffer[send_index++] = op_state;
	buffer[send_index++] = fault_state;
    return candevice->sendCanFrame(&tx_frame);
}

boolean CanBus::requestFirmwareVersion() {
    Logger::notice(LOG_TAG_CANBUS, "requestFirmwareVersion");
    twai_message_t tx_frame = {};

    tx_frame.extd = 1;
    tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_PROCESS_SHORT_BUFFER) << 8) + vesc_id;
    tx_frame.data_length_code = 0x03;
    tx_frame.data[0] = esp_can_id;
    tx_frame.data[1] = 0x00;
    tx_frame.data[2] = 0x00;  // COMM_FW_VERSION
    return candevice->sendCanFrame(&tx_frame);
}

boolean CanBus::requestRealtimeData() {
    Logger::notice(LOG_TAG_CANBUS, "requestRealtimeData");
    twai_message_t tx_frame = {};

    tx_frame.extd = 1;
    tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_PROCESS_SHORT_BUFFER) << 8) + vesc_id;
    tx_frame.data_length_code = 0x07;
    tx_frame.data[0] = esp_can_id;
    tx_frame.data[1] = 0x00;
    tx_frame.data[2] = 0x32;      // COMM_GET_VALUES_SELECTIVE
    // mask
    tx_frame.data[3] = 0x00;      // Byte1 of mask (Bits 24-31)
    tx_frame.data[4] = 0x00;      // Byte2 of mask (Bits 16-23)
    tx_frame.data[5] = B10000111; // Byte3 of mask (Bits 8-15)
    tx_frame.data[6] = B11000011; // Byte4 of mask (Bits 0-7)
    return candevice->sendCanFrame(&tx_frame);
}

boolean CanBus::requestBalanceData() {
    Logger::notice(LOG_TAG_CANBUS, "requestBalanceData");
    twai_message_t tx_frame = {};

    tx_frame.extd = 1;
    tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_PROCESS_SHORT_BUFFER) << 8) + vesc_id;
    tx_frame.data_length_code = 0x03;
    tx_frame.data[0] = esp_can_id;
    tx_frame.data[1] = 0x00;
    tx_frame.data[2] = 0x4F;  // COMM_GET_DECODED_BALANCE
    return candevice->sendCanFrame(&tx_frame);
}

boolean CanBus::requestFloatPackageData() {
    Logger::notice(LOG_TAG_CANBUS, "requestFloatPackageData");
    twai_message_t tx_frame = {};

    tx_frame.extd = 1;
    tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_PROCESS_SHORT_BUFFER) << 8) + vesc_id;
    tx_frame.data_length_code = 0x05;
    tx_frame.data[0] = esp_can_id;
    tx_frame.data[1] = 0x00;
    tx_frame.data[2] = 0x24;  // COMM_CUSTOM_APP_DATA (0x24)
    tx_frame.data[3] = 0x65;  // FLOAT PACKAGE (0x65)
    tx_frame.data[4] = 0x1;  // FLOAT_COMMAND_GET_RTDATA (0x1)
    return candevice->sendCanFrame(&tx_frame);
}

void CanBus::ping() {
    twai_message_t tx_frame = {};
    tx_frame.extd = 1;
    tx_frame.identifier = (uint32_t(0x8000) << 16) + (uint16_t(CAN_PACKET_PING) << 8) + vesc_id;
    tx_frame.data_length_code = 0x01;
    tx_frame.data[0] = esp_can_id;
    candevice->sendCanFrame(&tx_frame);
}

void CanBus::printFrame(twai_message_t rx_frame, int frameCount) {
    if (rx_frame.rtr)
        printf("#%d RTR from 0x%08x, DLC %d\r\n", frameCount, rx_frame.identifier, rx_frame.data_length_code);
    else {
        printf("#%d from 0x%08x, DLC %d, data [", frameCount, rx_frame.identifier, rx_frame.data_length_code);
        for (int i = 0; i < 8; i++) {
            printf("%d", (uint8_t) rx_frame.data[i]);
            if (i != 7) {
                printf("\t");
            }
        }
        printf("]\n");
    }
}

void CanBus::clearFrame(twai_message_t rx_frame) {
    rx_frame.identifier=0;
    rx_frame.data_length_code=0;
    rx_frame.data[0]=0;
    rx_frame.data[1]=0;
    rx_frame.data[2]=0;
    rx_frame.data[3]=0;
    rx_frame.data[4]=0;
    rx_frame.data[5]=0;
    rx_frame.data[6]=0;
    rx_frame.data[7]=0;
}

// CRC Table
const unsigned short crc16_tab[] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
		0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
		0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
		0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
		0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
		0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
		0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
		0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
		0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
		0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
		0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
		0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
		0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
		0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
		0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
		0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
		0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
		0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
		0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
		0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
		0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
		0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
		0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
		0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
		0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
		0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
		0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };

unsigned short crc16(unsigned char *buf, unsigned int len) {
	unsigned int i;
	unsigned short cksum = 0;
	for (i = 0; i < len; i++) {
		cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
	}
	return cksum;
}


void CanBus::processFrame(twai_message_t rx_frame, int frameCount) {
    String frametype = "";
    uint32_t ID = rx_frame.identifier;
    if (RECV_STATUS_1 == ID) {
        frametype = "status1";
        vescData->erpm = readInt32Value(rx_frame, 0);
        vescData->current = readInt16Value(rx_frame, 4) / 10.0;
        vescData->dutyCycle = readInt16Value(rx_frame, 6);
    }
     if (RECV_STATUS_2 == ID) {
        frametype = "status2";
        vescData->ampHours = readInt32Value(rx_frame, 0) / 10000.0;
        vescData->ampHoursCharged = readInt32Value(rx_frame, 4) / 10000.0;
    }
     if (RECV_STATUS_3 == ID) {
        frametype = "status3";
        vescData->wattHours = readInt32Value(rx_frame, 0) / 10000.0;
        vescData->wattHoursCharged = readInt32Value(rx_frame, 4) / 10000.0;
    }
     if (RECV_STATUS_4 == ID) {
        frametype = "status4";
        vescData->mosfetTemp = readInt16Value(rx_frame, 0) / 10.0;
        vescData->motorTemp = readInt16Value(rx_frame, 2) / 10.0;
        vescData->totalCurrentIn = readInt16Value(rx_frame, 4) / 10.0;
        vescData->pidPosition = readInt16Value(rx_frame, 6) / 50.0;
        vescData->motorPosition = readInt16Value(rx_frame, 6) / 50.0;
    }
     if (RECV_STATUS_5 == ID) {
        frametype = "status5";
        vescData->tachometer = readInt32Value(rx_frame, 0);
        vescData->inputVoltage = readInt16Value(rx_frame, 4) / 10.0;
        vescData->inputVoltage += AppConfiguration::getInstance()->config.batteryDrift;
    }

     if (RECV_PROCESS_SHORT_BUFFER_PROXY == ID) {
        frametype = "process short buffer for <<BLE proxy>>";
        for (int i = 1; i < rx_frame.data_length_code; i++) {
            proxybuffer.push_back(rx_frame.data[i]);
        }
        //unsigned short cksumb=crc16(proxybuffer.data(),proxybuffer.size());
        //int crc1b = (cksumb >> 8) & 0xFF;  // High byte
        //int crc2b = cksumb & 0xFF;         // Low byte
        proxybuffer.erase(proxybuffer.begin());
        unsigned short cksum=crc16(proxybuffer.data(),proxybuffer.size());
        int crc1 = (cksum >> 8) & 0xFF;  // High byte
        int crc2 = cksum & 0xFF;         // Low byte

        proxy->proxyOut(proxybuffer.data(), proxybuffer.size(), crc1, crc2);

        //printf("proxy data start %d,%d,%d,%d,%d,%d,%d,%d\n,",proxybuffer.data()[0],proxybuffer.data()[1],proxybuffer.data()[2],proxybuffer.data()[3],proxybuffer.data()[4],proxybuffer.data()[5],proxybuffer.data()[6],proxybuffer.data()[7]);
        //printf("proxy data end %d,%d,%d,%d,%d,%d,%d,%d\n,",proxybuffer.data()[proxybuffer.size()-7],proxybuffer.data()[proxybuffer.size()-6],proxybuffer.data()[proxybuffer.size()-5],proxybuffer.data()[proxybuffer.size()-4],proxybuffer.data()[proxybuffer.size()-3],proxybuffer.data()[proxybuffer.size()-2],proxybuffer.data()[proxybuffer.size()-1],proxybuffer.data()[proxybuffer.size()]);
        //printFrame(rx_frame,frameCount);
        //printf("length %d\n",proxybuffer.size());
        //printf("data4 %d\n",rx_frame.data[4]);
        //printf("data5 %d\n",rx_frame.data[5]);
        //printf("crc1 %d\n",crc1);
        //printf("crc2 %d\n",crc2);
        //printf("crc1b %d\n",crc1b);
        //printf("crc2b %d\n",crc2b);

        proxybuffer.clear();
    }

     if (RECV_FILL_RX_BUFFER == ID) {
        frametype = "fill rx buffer";
        for (int i = 1; i < rx_frame.data_length_code; i++) {
            buffer.push_back(rx_frame.data[i]);
        }
    }

     if (RECV_FILL_RX_BUFFER_PROXY == ID || RECV_FILL_RX_BUFFER_LONG_PROXY == ID) {
        boolean longBuffer = RECV_FILL_RX_BUFFER_LONG_PROXY == ID;
        frametype = longBuffer ? "fill rx long buffer" : "fill rx buffer";
        for (int i = (longBuffer ? 2 : 1); i < rx_frame.data_length_code; i++) {
            proxybuffer.push_back(rx_frame.data[i]);
        }
        //printFrame(rx_frame,frameCount);
        //printf("proxy fill data start %d,%d,%d,%d,%d,%d,%d,%d\n,",proxybuffer.data()[0],proxybuffer.data()[1],proxybuffer.data()[2],proxybuffer.data()[3],proxybuffer.data()[4],proxybuffer.data()[5],proxybuffer.data()[6],proxybuffer.data()[7]);
        //printf("proxy fill data end %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n,",proxybuffer.data()[proxybuffer.size()-12],proxybuffer.data()[proxybuffer.size()-11],proxybuffer.data()[proxybuffer.size()-10],proxybuffer.data()[proxybuffer.size()-9],proxybuffer.data()[proxybuffer.size()-8],proxybuffer.data()[proxybuffer.size()-7],proxybuffer.data()[proxybuffer.size()-6],proxybuffer.data()[proxybuffer.size()-5],proxybuffer.data()[proxybuffer.size()-4],proxybuffer.data()[proxybuffer.size()-3],proxybuffer.data()[proxybuffer.size()-2],proxybuffer.data()[proxybuffer.size()-1]);
        //printf("long buffer? %d\n",longBuffer);

    }

    if (RECV_PROCESS_RX_BUFFER == ID || RECV_PROCESS_RX_BUFFER_PROXY == ID) {
        boolean isProxyRequest = false;
        frametype = "process rx buffer for ";
        if (RECV_PROCESS_RX_BUFFER_PROXY == ID) {
            frametype += " <<BLE proxy>> ";
            isProxyRequest = true;
        }
        //Serial.printf("bytes %d\n", buffer.size());

        if ((!isProxyRequest && buffer.empty()) || (isProxyRequest && proxybuffer.empty())) {
            Serial.printf("buffer empty, abort");
            return;
        }
        uint8_t command = (isProxyRequest ? proxybuffer : buffer).at(0);
        if (command == 0x00) {
            frametype += "COMM_FW_VERSION";
            int offset = 1;
            vescData->majorVersion = readInt8ValueFromBuffer(0, isProxyRequest);
            vescData->minorVersion = readInt8ValueFromBuffer(1, isProxyRequest);
            vescData->name = readStringValueFromBuffer(2 + offset, 12, isProxyRequest);
        } else if (command == 0x4F) {  //0x4F = 79 DEC
            frametype += "COMM_GET_DECODED_BALANCE";
            int offset = 1;
            vescData->pidOutput = readInt32ValueFromBuffer(0 + offset, isProxyRequest) / 1000000.0;
            vescData->pitch = readInt32ValueFromBuffer(4 + offset, isProxyRequest) / 1000000.0;
            vescData->roll = readInt32ValueFromBuffer(8 + offset, isProxyRequest) / 1000000.0;
            vescData->loopTime = readInt32ValueFromBuffer(12 + offset, isProxyRequest);
            vescData->motorCurrent = readInt32ValueFromBuffer(16 + offset, isProxyRequest) / 1000000.0;
            vescData->motorPosition = readInt32ValueFromBuffer(20 + offset, isProxyRequest) / 1000000.0;
            vescData->balanceState = readInt16ValueFromBuffer(24 + offset, isProxyRequest);
            vescData->switchState = readInt16ValueFromBuffer(26 + offset, isProxyRequest);
            vescData->adc1 = readInt32ValueFromBuffer(28 + offset, isProxyRequest) / 1000000.0;
            vescData->adc2 = readInt32ValueFromBuffer(32 + offset, isProxyRequest) / 1000000.0;
            lastBalanceData = millis();
        } else if (command == 0x24) {  //0x24 = 36 DEC
            frametype += "COMM_CUSTOM_APP_DATA";
            if(readInt8ValueFromBuffer(1,isProxyRequest) == 101) //magic number
            {
                if(readInt8ValueFromBuffer(2,isProxyRequest) == 1 ) //FLOAT_COMMAND_GET_RTDATA (0x1)
                {
                    int offset = 3;
                    //FLOAT PACKAGE (0x65)
                    //FLOAT_COMMAND_GET_RTDATA (0x1)
                    //printFrame(rx_frame,frameCount);
                    //dumpVescValues();
                    // Reading floats
                    vescData->pidOutput = readFloatValueFromBuffer(0 + offset, isProxyRequest);
                    vescData->pitch = readFloatValueFromBuffer(4 + offset, isProxyRequest);
                    vescData->roll = readFloatValueFromBuffer(8 + offset, isProxyRequest);
                    //vescData->loopTime = readInt32ValueFromBuffer(12 + offset, isProxyRequest); No functional equivilent
                    //vescData->motorCurrent = readInt32ValueFromBuffer(16 + offset, isProxyRequest) / 1000000.0; Done in COMM_GET_VALUES 0x4
                    //vescData->motorPosition = readInt32ValueFromBuffer(20 + offset, isProxyRequest) / 1000000.0; Done in COMM_GET_VALUES 0x4
                    // Reading state (1 byte)
                    vescData->balanceState = readInt8ValueFromBuffer(12 + offset, isProxyRequest);
                    // Reading switch_state (1 byte)
                    uint16_t switchState = readInt8ValueFromBuffer(13 + offset, isProxyRequest);
                    // Reading adc1 and adc2 (floats)
                    vescData->adc1 = readFloatValueFromBuffer(14 + offset, isProxyRequest);
                    vescData->adc2 = readFloatValueFromBuffer(18 + offset, isProxyRequest);

                    switch(switchState)
                    {
                        case 0:
                            vescData->switchState=0;
                        break;
                        case 1:
                            vescData->switchState = (vescData->adc1 > vescData->adc2) ? 1 : 2;
                        break;
                        case 2:
                            vescData->switchState=3;
                        break;
                    }
                    lastBalanceData = millis();
                }
            }
        } else if (command == 0x32) { //0x32 = 50 DEC
            frametype += "COMM_GET_VALUES_SELECTIVE";
            int offset = 1;
            vescData->mosfetTemp = readInt16ValueFromBuffer(4 + offset, isProxyRequest) / 10.0;
            vescData->motorTemp = readInt16ValueFromBuffer(6 + offset, isProxyRequest) / 10.0;
            vescData->dutyCycle = readInt16ValueFromBuffer(8 + offset, isProxyRequest) / 1000.0;
            vescData->erpm = readInt32ValueFromBuffer(10 + offset, isProxyRequest);
            vescData->inputVoltage = readInt16ValueFromBuffer(14 + offset, isProxyRequest) / 10.0;
            vescData->inputVoltage += AppConfiguration::getInstance()->config.batteryDrift;
            vescData->tachometer = readInt32ValueFromBuffer(16 + offset, isProxyRequest);
            vescData->tachometerAbsolut = readInt32ValueFromBuffer(20 + offset, isProxyRequest);
            vescData->fault = readInt8ValueFromBuffer(24 + offset, isProxyRequest);
            lastRealtimeData = millis();
        }  else if (command == 0x33) { //0x33 = 51 DEC
            frametype += "COMM_GET_VALUES_SETUP_SELECTIVE";
            int offset = 1;
            int startbyte = 0;
            uint32_t bitmask = readInt32ValueFromBuffer(0 + offset, isProxyRequest);
            startbyte += 4;
            if(bitmask & ((uint32_t) 1 << 0)) {
                vescData->mosfetTemp = readInt16ValueFromBuffer(startbyte + offset, isProxyRequest) / 10.0;
                startbyte += 2;
            }
            if(bitmask & ((uint32_t) 1 << 1)) {
                vescData->motorTemp = readInt16ValueFromBuffer(startbyte + offset, isProxyRequest) / 10.0;
                startbyte += 2;
            }
            if(bitmask & ((uint32_t) 1 << 2)) {
                // current in
                startbyte += 4;
            }
            if(bitmask & ((uint32_t) 1 << 3)) {
                // current in_total
                startbyte += 4;
            }
            if(bitmask & ((uint32_t) 1 << 4)) {
                vescData->dutyCycle = readInt16ValueFromBuffer(startbyte + offset, isProxyRequest) / 1000.0;
                startbyte += 2;
            }
            if(bitmask & ((uint32_t) 1 << 5)) {
                vescData->erpm = readInt32ValueFromBuffer(startbyte + offset, isProxyRequest);
                startbyte += 4;
            }
            if(bitmask & ((uint32_t) 1 << 6)) {
                // speed
                startbyte += 4;
            }
            if(bitmask & ((uint32_t) 1 << 7)) {
                vescData->inputVoltage = readInt16ValueFromBuffer(startbyte + offset, isProxyRequest) / 10.0;
                vescData->inputVoltage += AppConfiguration::getInstance()->config.batteryDrift;
                startbyte += 2;
            }
            if(bitmask & ((uint32_t) 1 << 8)) {
                // battery level
                startbyte += 2;
            }
            if(bitmask & ((uint32_t) 1 << 9)) {
                // amphours consumed
                startbyte += 4;
            }
            if(bitmask & ((uint32_t) 1 << 10)) {
                // amphours charged
                startbyte += 4;
            }
            if(bitmask & ((uint32_t) 1 << 11)) {
                // watthours consumed
                startbyte += 4;
            }
            if(bitmask & ((uint32_t) 1 << 12)) {
                // watthours charged
                startbyte += 4;
            }
            if(bitmask & ((uint32_t) 1 << 13)) {
                vescData->tachometer = readInt32ValueFromBuffer(16 + offset, isProxyRequest);
                startbyte += 4;
            }
            if(bitmask & ((uint32_t) 1 << 14)) {
                vescData->tachometerAbsolut = readInt32ValueFromBuffer(20 + offset, isProxyRequest);
                startbyte += 4;
            }
            if(bitmask & ((uint32_t) 1 << 16)) {
                vescData->fault = readInt8ValueFromBuffer(24 + offset, isProxyRequest);
                startbyte += 1;
            }
            lastRealtimeData = millis();
        } else if (command == 0x04) {
            frametype += "COMM_GET_VALUES";
            int offset = 1;
            vescData->mosfetTemp = readInt16ValueFromBuffer(0 + offset, isProxyRequest) / 10.0;
            vescData->motorTemp = readInt16ValueFromBuffer(2 + offset, isProxyRequest) / 10.0;
            vescData->motorCurrent = readInt32ValueFromBuffer(4 + offset, isProxyRequest) / 100.0;
            vescData->current = readInt32ValueFromBuffer(8 + offset, isProxyRequest) / 100.0;
            // id = vescData->readInt32ValueFromBuffer(12 + offset, isProxyRequest) / 100.0;
            // iq = vescData->readInt32ValueFromBuffer(16 + offset, isProxyRequest) / 100.0;
            vescData->dutyCycle = readInt16ValueFromBuffer(20 + offset, isProxyRequest) / 1000.0;
            vescData->erpm = readInt32ValueFromBuffer(22 + offset, isProxyRequest);
            vescData->inputVoltage = readInt16ValueFromBuffer(26 + offset, isProxyRequest) / 10.0;
            vescData->inputVoltage += AppConfiguration::getInstance()->config.batteryDrift;
            vescData->ampHours =  readInt32ValueFromBuffer(28 + offset, isProxyRequest) / 10000.0;
            vescData->ampHoursCharged = readInt32ValueFromBuffer(32 + offset, isProxyRequest) / 10000.0;
            vescData->wattHours =  readInt32ValueFromBuffer(46 + offset, isProxyRequest) / 10000.0;
            vescData->wattHoursCharged = readInt32ValueFromBuffer(40 + offset, isProxyRequest) / 10000.0;
            vescData->tachometer = readInt32ValueFromBuffer(44 + offset, isProxyRequest);
            vescData->tachometerAbsolut = readInt32ValueFromBuffer(58 + offset, isProxyRequest);
            vescData->fault = readInt8ValueFromBuffer(52 + offset, isProxyRequest);
            lastRealtimeData = millis();
        } else if (command == 0x0E) {  //0x0E = 14 DEC
            frametype += "COMM_GET_MCCONF";
        } else if (command == 0x11) {  //0x11 = 17 DEC
            frametype += "COMM_GET_APPCONF";
        } else if (command == 0x2F) { //0x2F = 47 DEC
            frametype += "COMM_GET_VALUES_SETUP";
        } else if (command == 0x33) { //0x33 = 51 DEC
            frametype += "COMM_GET_VALUES_SETUP_SELECTIVE";
        } else if (command == 0x41) { //0x41 = 65 DEC
            frametype += "COMM_GET_IMU_DATA";
        } else if (command == 0x3E) { //0x3E = 62 DEC
            frametype += "COMM_PING_CAN";
        } else {
            frametype += command;
        }
        if (isProxyRequest) {
            //printFrame(rx_frame,frameCount);
                        //if (proxybuffer.data()[1]==36&&proxybuffer.data()[2]==101&&proxybuffer.data()[3]==0)
            //{
            //printf("in command %d\n",command);

            //printf("length %d\n",proxybuffer.size());
            //printFrame(rx_frame,frameCount);
            //printf("proxy data process start %d,%d,%d,%d,%d,%d,%d,%d\n,",proxybuffer.data()[0],proxybuffer.data()[1],proxybuffer.data()[2],proxybuffer.data()[3],proxybuffer.data()[4],proxybuffer.data()[5],proxybuffer.data()[6],proxybuffer.data()[7]);
            //printf("proxy data process end %d,%d,%d,%d,%d,%d,%d,%d\n,",proxybuffer.data()[proxybuffer.size()-8],proxybuffer.data()[proxybuffer.size()-7],proxybuffer.data()[proxybuffer.size()-6],proxybuffer.data()[proxybuffer.size()-5],proxybuffer.data()[proxybuffer.size()-4],proxybuffer.data()[proxybuffer.size()-3],proxybuffer.data()[proxybuffer.size()-2],proxybuffer.data()[proxybuffer.size()-1]);
            //printf("length %d\n",proxybuffer.size());
            //if()
            //printf("%s",frametype);
            //if(proxybuffer.size()>>8 == 1)
            //{
                //((uint8_t) in.at(1) << 8) + (uint8_t) in.at(2);
            //printf("in command %d\n",command);

      //      printFrame(rx_frame,frameCount);
        //    if (!proxybuffer.empty()) {
          //      proxybuffer.erase(proxybuffer.begin());
            //}
           // }
            //if (!proxybuffer.empty()) {
            //    proxybuffer.erase(proxybuffer.begin());
            //}
             //}
            //proxybuffer.erase(proxybuffer.begin());
        
            unsigned short cksum=crc16(proxybuffer.data(),proxybuffer.size());
            int crc1 = (cksum >> 8) & 0xFF;  // High byte
            int crc2 = cksum & 0xFF;         // Low byte
            proxy->proxyOut(proxybuffer.data(), proxybuffer.size(), crc1, crc2);
            proxybuffer.clear();    
        } else {
            buffer.clear();
        }
    }

    if (Logger::getLogLevel() <= Logger::NOTICE) {
        snprintf(buf, bufSize, "processed frame #%d, type %s", frameCount, frametype.c_str());
        Logger::verbose(LOG_TAG_CANBUS, buf);
    }
}

void CanBus::dumpVescValues() {
    if (Logger::getLogLevel() != Logger::VERBOSE || millis() - lastDump < 1000) {
        return;
    }
    std::string bufferString;
    bufferString += "name=";
    snprintf(buf, bufSize, "%s, ", vescData->name.c_str());
    bufferString += buf;
    bufferString += "dutycycle=";
    snprintf(buf, bufSize, "%f", vescData->dutyCycle);
    bufferString += buf;
    bufferString += ", erpm=";
    snprintf(buf, bufSize, "%f", vescData->erpm);
    bufferString += buf;
    bufferString += ", current=";
    snprintf(buf, bufSize, "%f", vescData->current);
    bufferString += buf;
    bufferString += ", ampHours=";
    snprintf(buf, bufSize, "%f", vescData->ampHours);
    bufferString += buf;
    bufferString += ", ampHoursCharged=";
    snprintf(buf, bufSize, "%f", vescData->ampHoursCharged);
    bufferString += buf;
    bufferString += ", wattHours=";
    snprintf(buf, bufSize, "%f", vescData->wattHours);
    bufferString += buf;
    bufferString += ", wattHoursCharged=";
    snprintf(buf, bufSize, "%f", vescData->wattHoursCharged);
    bufferString += buf;
    bufferString += ", mosfetTemp=";
    snprintf(buf, bufSize, "%f", vescData->mosfetTemp);
    bufferString += buf;
    bufferString += ", motorTemp=";
    snprintf(buf, bufSize, "%f", vescData->motorTemp);
    bufferString += buf;
    bufferString += ", inputVoltage=";
    snprintf(buf, bufSize, "%f", vescData->inputVoltage);
    bufferString += buf;
    bufferString += ", tachometer=";
    snprintf(buf, bufSize, "%f", vescData->tachometer);
    bufferString += buf;
    bufferString += ", pidOutput=";
    snprintf(buf, bufSize, "%f", vescData->pidOutput);
    bufferString += buf;
    bufferString += ", pitch=";
    snprintf(buf, bufSize, "%f", vescData->pitch);
    bufferString += buf;
    bufferString += ", roll=";
    snprintf(buf, bufSize, "%f", vescData->roll);
    bufferString += buf;
    bufferString += ", loopTime=";
    snprintf(buf, bufSize, "%d", vescData->loopTime);
    bufferString += buf;
    bufferString += ", motorCurrent=";
    snprintf(buf, bufSize, "%f", vescData->motorCurrent);
    bufferString += buf;
    bufferString += ", motorPosition=";
    snprintf(buf, bufSize, "%f", vescData->motorPosition);
    bufferString += buf;
    bufferString += ", balanceState=";
    snprintf(buf, bufSize, "%d", vescData->balanceState);
    bufferString += buf;
    bufferString += ", switchState=";
    snprintf(buf, bufSize, "%d", vescData->switchState);
    bufferString += buf;
    bufferString += ", adc1=";
    snprintf(buf, bufSize, "%f", vescData->adc1);
    bufferString += buf;
    bufferString += ", adc2=";
    snprintf(buf, bufSize, "%f", vescData->adc2);
    bufferString += buf;
    bufferString += ", fault=";
    snprintf(buf, bufSize, "%d", vescData->fault);
    bufferString += buf;
    Logger::verbose(LOG_TAG_CANBUS, bufferString.c_str());
    lastDump = millis();
}

float CanBus::readFloatValueFromBuffer(int startbyte, boolean isProxyRequest) {
    int32_t index = startbyte;
    const uint8_t *currBuffer = isProxyRequest ? proxybuffer.data() : buffer.data();

    return buffer_get_float32_auto(currBuffer, &index);
}

int32_t CanBus::readInt32Value(twai_message_t rx_frame, int startbyte) {
    int32_t intVal = (
            ((int32_t) rx_frame.data[startbyte] << 24) +
            ((int32_t) rx_frame.data[startbyte + 1] << 16) +
            ((int32_t) rx_frame.data[startbyte + 2] << 8) +
            ((int32_t) rx_frame.data[startbyte + 3]));
    return intVal;
}

int16_t CanBus::readInt16Value(twai_message_t rx_frame, int startbyte) {
    int16_t intVal = (
            ((int16_t) rx_frame.data[startbyte] << 8) +
            ((int16_t) rx_frame.data[startbyte + 1]));
    return intVal;
}

int32_t CanBus::readInt32ValueFromBuffer(int startbyte, boolean isProxyRequest) {
    int32_t intVal = (
            ((int32_t) (isProxyRequest ? proxybuffer : buffer).at(startbyte) << 24) +
            ((int32_t) (isProxyRequest ? proxybuffer : buffer).at(startbyte + 1) << 16) +
            ((int32_t) (isProxyRequest ? proxybuffer : buffer).at(startbyte + 2) << 8) +
            ((int32_t) (isProxyRequest ? proxybuffer : buffer).at(startbyte + 3)));
    return intVal;
}

int16_t CanBus::readInt16ValueFromBuffer(int startbyte, boolean isProxyRequest) {
    int16_t intVal = (
            ((int16_t) (isProxyRequest ? proxybuffer : buffer).at(startbyte) << 8) +
            ((int16_t) (isProxyRequest ? proxybuffer : buffer).at(startbyte + 1)));
    return intVal;
}

int8_t CanBus::readInt8ValueFromBuffer(int startbyte, boolean isProxyRequest) {
    return (isProxyRequest ? proxybuffer : buffer).at(startbyte);
}

std::string CanBus::readStringValueFromBuffer(int startbyte, int length, boolean isProxyRequest) {
    std::string name;
    for (int i = startbyte; i < startbyte + length; i++) {
        name += (char) (isProxyRequest ? proxybuffer : buffer).at(i);
    }
    return name;
}