#include "esphome/core/log.h"
#include "tigotap.h"

namespace esphome {
    namespace tigo_tap {

    static const char *TAG = "TigoTap";

    /********** Sensor **********/
    void Sensor::setup() {
    }
    
    void Sensor::loop() {
    }

    void Sensor::update() {
        this->publish_state( this->getValue() );
    }

    void Sensor::dump_config() {
        ESP_LOGD(TAG, "%s Sensor - Barcode: %s\n   Timeout is: %i", this->getTypeString().c_str(), this->getBarcode().c_str(), this->getSensorTimeout());
    }

    void Sensor::setType(std::string type) {
        if ( type.compare("Vin") == 0 )
            this->_type = SensorType::Vin;
        else if ( type.compare("Vout") == 0 )
            this->_type = SensorType::Vout;
        else if ( type.compare("Iin") == 0 )
            this->_type = SensorType::Iin;
        else if ( type.compare("Temp") == 0 )
            this->_type = SensorType::Temp;
        else if ( type.compare("DcDcDuty") == 0)
            this->_type = SensorType::DcDcDuty;
        else if ( type.compare("RSSI") == 0)
            this->_type = SensorType::RSSI;
        else {
            this->_type = SensorType::None;
            ESP_LOGE(TAG, "Sensortype %s is not supported!", type.c_str());
        }
    }

    std::string Sensor::getTypeString() {
        std::string typeString;
        switch(this->_type) {
            case SensorType::None:      typeString = "None";      break;
            case SensorType::Vin:       typeString = "Vin";       break;
            case SensorType::Vout:      typeString = "Vout";      break;
            case SensorType::Iin:       typeString = "Iin";       break;
            case SensorType::Temp:      typeString = "Temp";      break;
            case SensorType::DcDcDuty:  typeString = "DcDcDuty";  break;
            case SensorType::RSSI:      typeString = "RSSI";      break;
        }

        return typeString;
    }

    void Sensor::setValue(uint16_t value) {
        this->_value = value;
        this->_refreshTimer = App.get_loop_component_start_time();
    }

    uint16_t Sensor::getRawValue() {
        // Reset Value if not set in the timeout Window
        uint32_t now = App.get_loop_component_start_time();
        if ( this->_timeout != 0 ) {
            if (now - this->_refreshTimer > this->_timeout) {
                this->_value = 0;
            }
        }

        return this->_value;
    }

    float Sensor::getValue() {
        float value = 0.0;

        switch( this->getType() ) {
            case SensorType::Vin:       value = ((float)this->getRawValue() * 0.050);     break;
            case SensorType::Vout:      value = ((float)this->getRawValue() * 0.100);     break;
            case SensorType::Iin:       value = ((float)this->getRawValue() * 0.005);     break;
            case SensorType::Temp:      value = ((float)this->getRawValue() * 0.100);     break;
            case SensorType::DcDcDuty:  value = (((float)(this->getRawValue())/255)*100); break;
            case SensorType::RSSI:
                value = (float)(this->getRawValue());
                // Roughly aproximate db Values based on the chart found in:
                // https://www.nxp.com/docs/en/data-sheet/JN516X.pdf - Page 29
                if (value > 10.0)
                    value = (-98) - (value/255) * (-98-(-9));
                else
                    value = (-102) - (value/10) * (-102-(-94));
                break;
        }

        return value;
    }



    /********** FrameBuffer **********/
    FrameBuffer::FrameBuffer() {
        this->_index = 0;
    }

    bool FrameBuffer::append(uint8_t value) {
        if (this->_index < MAX_FRAME_SIZE) {
            this->_frameBuffer[this->_index] = value;
            this->_index++;

            return true;
        }
        else {
            return false;
        }
    }

    int FrameBuffer::remove(uint16_t numOfBytes) {
        if (this->_index >= numOfBytes) {
            this->_index -= numOfBytes;
        }
        else {
            this->_index = 0;
        }

        return this->_index;
    }

    uint16_t FrameBuffer::endWord() {
        uint8_t b1 = 0;
        uint8_t b2 = 0;
        if (this->_index > 0) b1 = this->_frameBuffer[this->_index-1];
        if (this->_index > 1) b2 = this->_frameBuffer[this->_index-2];

        uint16_t endWord = (b2<<8) | (b1 & 0xFF);

        return endWord;
    }

    int FrameBuffer::length() {
        return this->_index;
    }

    void FrameBuffer::reset() {
        this->_index = 0;
    }

    uint8_t FrameBuffer::operator[](uint16_t index) {
        if ( index < (this->_index-1) )
            return this->_frameBuffer[index];
        else
            return this->_frameBuffer[this->_index-1];
    }


    /********** TigoTap **********/
    TigoTap::TigoTap() {
        esp_err_t err = nvs_flash_init();
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            // NVS partition was truncated and needs to be erased
            // Retry nvs_flash_init
            nvs_flash_erase();
            nvs_flash_init();
        }

        if ( nvs_open("barcodeTable", NVS_READWRITE, &this->nvsBarcodeTable) != ESP_OK )
            ESP_LOGE(TAG, "Error opening Barcode Table from NVS!");

        //std::string test = "4-B48C82H";
        //this->updateBarcodeTable(&test, 2);
    }

    void TigoTap::setup() {
    }

    void TigoTap::loop() {
        FrameBuffer frame = FrameBuffer();
        if ( !this->readFrame(&frame) )
            ESP_LOGD(TAG, "Error Reading Frame!");
        else {
            uint16_t frameType = (frame[2]<<8) | (frame[3] & 0xFF);

            if ( ( frameType == 0x0149) || (frameType == 0x0B10) || (frameType == 0x0B0F) ) {
                // only process frames we are intressed in. CRC is expensive!
                if ( this->checkFrameCRC(&frame) )
                    processFrame(&frame);
            }
        }

    }

    void TigoTap::dump_config() {
        for (auto it = this->_sensors.begin(); it != this->_sensors.end(); ++it) {
            (*it)->dump_config();
        }
    }

    void TigoTap::update() {
        for (auto it = this->_sensors.begin(); it != this->_sensors.end(); ++it) {
            (*it)->update();
        }
    }

    void TigoTap::register_sensor(Sensor *obj, std::string barcode) {
        obj->setBarcode(barcode);

        uint16_t pvId;
        if ( nvs_get_u16(this->nvsBarcodeTable, barcode.c_str(), &pvId) == ESP_OK ) {
            obj->setPvNodeId(pvId);
        }

        this->_sensors.push_back(obj);
    }

    bool TigoTap::readByte(uint8_t* byte) {
        // wait a for a byte to come available on the uart
        int cnt = 0;
        while( (this->available()==0) && (cnt < READ_MAX_RETRIES) ) {
            cnt++;
            vTaskDelay(1);
        }

        // read byte if there is some
        if (cnt < READ_MAX_RETRIES) {
            this->read_byte(byte);
            return true;
        }
        else
            return false;
    }

    bool TigoTap::readFrame(FrameBuffer* buffer) {
        uint8_t currentByte;
        bool frameStarted = false;
        bool frameFinished = false;

        uint16_t loopErrorCnt = 0;
        while ( (frameFinished == false) && (buffer->length() < MAX_FRAME_SIZE)) {
            if (this->readByte(&currentByte)) {
                buffer->append(currentByte);
                uint16_t endWord = buffer->endWord();

                if ( (frameStarted == false) && (endWord == 0x7E07) ) {
                    // Frame Start detected
                    frameStarted = true;
                    buffer->reset();
                }
                else if ( (frameStarted == true) && (endWord == 0x7E08)) {
                    // Frame End detected
                    frameFinished = true;
                    buffer->remove(2); // remove EOF from Buffer
                }
                else if ( (frameStarted == true) && ((endWord >> 8) == 0x7E) ) {
                    // Escape Character detected
                    switch(endWord) {
                        case 0x7E00:
                            buffer->remove(2);
                            buffer->append(0x7E);
                            break;
                        case 0x7E01:
                            buffer->remove(2);
                            buffer->append(0x24);
                            break;
                        case 0x7E02:
                            buffer->remove(2);
                            buffer->append(0x23);
                            break;
                        case 0x7E03:
                            buffer->remove(2);
                            buffer->append(25);
                            break;
                        case 0x7E04:
                            buffer->remove(2);
                            buffer->append(0xA4);
                            break;
                        case 0x7E05:
                            buffer->remove(2);
                            buffer->append(0xA3);
                            break;
                        case 0x7E06:
                            buffer->remove(2);
                            buffer->append(0xA5);
                            break;
                        //default: maybe discard Frame as this shoudn't happen?
                    }
                }
            }
            else {
                break;
            }
        }

        return (frameStarted && frameFinished);
    }

    void TigoTap::processFrame(FrameBuffer* frame) {
        uint16_t frameType = ((*frame)[2]<<8) | ((*frame)[3] & 0xFF);


        if (frameType == 0x0149) {
            // We are (for now) not intressted in the Headers Content so just skip it
            uint8_t headerLength = this->calcFrameHeaderLength(frame);
            // Set the index to the start of the first Packet
            uint16_t index = 4 + headerLength;

            while ( index < frame->length()-2 ) {
                uint8_t packetType = (*frame)[index];
                uint8_t packetLength = (*frame)[index+6];

                if (packetType == 0x31) {
                    // Power Report Packet
                    if ( !this->processPowerPacket(frame, index) ) 
                        break;
                }
                else if (packetType == 0x09) {
                    ESP_LOGD(TAG, "Packet 09 deteced!");
                    this->logFrame(frame);
                    if ( !this->processTopologyReport(frame, index) )
                        break;
                }
                index = index + packetLength + 7;
            }
        }
        else if ( (frameType == 0x0B10) || (frameType == 0x0B0F) ) {
            uint8_t packetType = (*frame)[7];

            if (packetType == 0x27) {
                ESP_LOGD(TAG, "Packet 27 deteced!");
                this->logFrame(frame);
                this->processNodeTable(frame, 7);
            }
        }
    }

    bool TigoTap::processPowerPacket(FrameBuffer* frame, uint16_t position) {
        if ( (frame->length()>position+19) && ((*frame)[position+6]==13) ) {
            uint16_t pvNodeId = ((*frame)[position+1]<<8) | ((*frame)[position+2] & 0xFF);

            uint16_t vin = ((*frame)[position+7]<<4) | (((*frame)[position+8]&0xF0)>>4); // Scale 0.05V
            uint16_t vout = (((*frame)[position+8]&0x0F)<< 8) | ((*frame)[position+9]&0xFF); // Scale 0.10V
            uint8_t duty = (*frame)[position+10]; // 255 = 100%
            uint16_t iin = ((*frame)[position+11]<<4) | (((*frame)[position+12]&0xF0)>>4); // Scale 0.005A
            uint16_t temp = (((*frame)[position+12]&0x0F)<<8) | ((*frame)[position+13]&0xFF); // Scale 0.1°C
            uint8_t rssi = (*frame)[position+19];

            for (auto it = this->_sensors.begin(); it != this->_sensors.end(); ++it){
                if ((*it)->getPvNodeId() == pvNodeId) {
                    switch ((*it)->getType()) {
                        case SensorType::Vin:       (*it)->setValue(vin);      break;
                        case SensorType::Vout:      (*it)->setValue(vout);     break;
                        case SensorType::Iin:       (*it)->setValue(iin);      break;
                        case SensorType::DcDcDuty:  (*it)->setValue(duty);     break;
                        case SensorType::Temp:      (*it)->setValue(temp);     break;
                        case SensorType::RSSI:      (*it)->setValue(rssi);     break;
                    }
                }
            }
            
            //ESP_LOGD("Power", "ID: %x (RSSI: %i) - Vin: %f, Vout: %f, Duty: %i, I: %f, T: %f", pvNodeId, rssi, (vin*0.05), (vout*0.10), duty, (iin*0.005), (t*0.1));
        
            return true;
        }
        else {
            return false;
        }
    }

    bool TigoTap::processTopologyReport(FrameBuffer* frame, uint16_t position) {
        uint8_t length = (*frame)[position+6];
        uint16_t pvNodeId = ((*frame)[position+9]<<8) | ((*frame)[position+10]&0xFF);

        uint8_t pvNodeAddr[8];
        for (uint16_t i=0; i<8; i++) {
            pvNodeAddr[i] = (*frame)[position+15+i];
        }
        uint8_t rssi = (*frame)[position+23];

        //ESP_LOGD(TAG, "ID: %i (RSSI: %i) - %x %x %x %x %x %x %x %x - len: %i", pvNodeId, rssi, pvNodeAddr[0], pvNodeAddr[1], pvNodeAddr[2], pvNodeAddr[3], pvNodeAddr[4], pvNodeAddr[5], pvNodeAddr[6], pvNodeAddr[7], length);
        std::string barcode;
        this->generateBarcodeFromAddress(&barcode, pvNodeAddr);
        this->updateBarcodeTable(&barcode, pvNodeId);

        return true;
    }

    bool TigoTap::processNodeTable(FrameBuffer* frame, uint16_t position) {
        uint16_t startIndex = ((*frame)[position+2]<<8) | ((*frame)[position+3] & 0xFF);
        uint16_t entrys = ((*frame)[position+4]<<8) | ((*frame)[position+5] & 0xFF);

        ESP_LOGD(TAG, "start-idx: %i, entrys: %i, frameLength:", startIndex, entrys, frame->length());
        for(uint16_t entry=0; entry<entrys; entry++) {
            uint8_t pvNodeAddr[8];
            uint16_t pvNodeId;
            for (uint8_t i=0; i<8; i++) {
                pvNodeAddr[i] = (*frame)[position+6+i];
            }
            pvNodeId = ((*frame)[position+14]<<8) | ((*frame)[position+15] & 0xFF);

            //ESP_LOGD(TAG, "ID: %i (entrys: %i) - %x %x %x %x %x %x %x %x", pvNodeId, entrys, pvNodeAddr[0], pvNodeAddr[1], pvNodeAddr[2], pvNodeAddr[3], pvNodeAddr[4], pvNodeAddr[5], pvNodeAddr[6], pvNodeAddr[7]);
            std::string barcode;
            this->generateBarcodeFromAddress(&barcode, pvNodeAddr);
            this->updateBarcodeTable(&barcode, pvNodeId);

            position += 10;
        }

        return true;
    }

    bool TigoTap::checkFrameCRC(FrameBuffer* frame) {
        uint16_t crc = 0x8408;  // Initial value

        for (uint16_t i = 0; i < frame->length()-2; i++) {
            uint8_t index = (crc ^ (*frame)[i]) & 0xFF;
            crc = (crc >> 8) ^ crc_table[index];
        }
        crc = (crc >> 8) | (crc << 8);
        
        return (frame->endWord() == crc);
    }

    uint8_t TigoTap::calcFrameHeaderLength(FrameBuffer* frame) {
        uint8_t headerLength = 5; // Paket # Low 1b & Slot-Cnt 2b
        uint8_t header = (*frame)[5];

        if ( (header & 0x01) == 0 ) // RX Buffer 1b
            headerLength += 1;
        if ( (header & 0x02) == 0 ) // TX Buffer 1b
            headerLength += 1;
        if ( (header & 0x04) == 0 ) // A Buffer 2b
            headerLength += 2;
        if ( (header & 0x08) == 0 ) // B Buffer 2b
            headerLength += 2;
        if ( (header & 0x10) == 0 ) // Packet # High 1b
            headerLength += 1;

        return headerLength;
    }

    void TigoTap::generateBarcodeFromAddress(std::string* barcode, uint8_t* address) {
        // Generate CRC4
        uint8_t crc = 0x2;
        for (uint8_t i=0; i<8; i++) {
            crc = crc4_table[ address[i] ^ (crc<<4) ];
        }

        // create String
        char buffer[10];
        snprintf(buffer, 10, "%X-%02X%02X%02X%c", (address[3]>>4), address[5], address[6], address[7], barcode_crc_map[crc]);

        barcode->append(buffer);
    }

    void TigoTap::updateBarcodeTable(std::string* barcode, uint16_t pvID) {
        // Writing barcode & pvID to NVS
        uint16_t oldPvId = 0;
        if ( nvs_get_u16(this->nvsBarcodeTable, barcode->c_str(), &oldPvId) != ESP_OK ) {
            ESP_LOGD(TAG, "Barcode %s not found in NVS", barcode->c_str());
            ESP_LOGD (TAG, "  ...setting pvID to %i", pvID);
            nvs_set_u16(this->nvsBarcodeTable, barcode->c_str(), pvID);
        }
        else {
            ESP_LOGD(TAG, "Barcode %s was found with pvID %i", barcode->c_str(), oldPvId);
            if (oldPvId != pvID) {
                ESP_LOGD (TAG, "  ...updating pvID to %i", pvID);
                nvs_set_u16(this->nvsBarcodeTable, barcode->c_str(), pvID);
            }
        }

        // Updating Sensors
        for (auto it = this->_sensors.begin(); it != this->_sensors.end(); ++it){
            if ((*it)->getBarcode().compare( (*barcode) ) == 0 ) {
                (*it)->setPvNodeId(pvID);
            }
        }
    }

    void TigoTap::logFrame(FrameBuffer* frame) {
        const int max = MAX_FRAME_SIZE*3;

        char buffer[max];
        int pos = 0;
        for (int i=0; i<frame->length(); i++) {
            pos += snprintf(buffer + pos, max, "%02X ", (*frame)[i]);
        }

        ESP_LOGD(TAG, "%i: %S", frame->length(), buffer);
    }

    }  // namespace empty_uart_component
}  // namespace esphome