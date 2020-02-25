# WemosLora_OTAA_ABP_EEPROM
Lora TTN with OTAA, and Ultrasonic Distance Measurement: saves Network data to EEPROM to prevent long wait after Join


Many times I have had issues with failed Join Request in the past to TTN Network.
After long search I found this repository² and adpated it to my needs:
This sketch does uses a homebuild Addon-Board for WEMOS D1 Mini. This aDDon Board has the option to solder an RFM95 Lora Board on it
With this Board you may connect to TTN Network. Purpose of this Board is Measuring the distance with a Ultrasonic Module 
Please Replace the "Fillmein" with your Credentials from TTN

How it works:
1. If there was no sucessful OTAA Join with the network ever before the Module will TRy to do an OTAA Activtaion
2. Once OTAA Join was sucessful all Keys are stored to the EEPROM
3. Next a distance with a SR04 Ultrasonic Moudle is measured
4. After that the module is going to sleep to minimize power consumption
5. When the module is waking up again after 3min it will read the stored KEYs from the EEPROM and try to send data with these Keys to TTN.
6. If transmission was fired(no confirmation) the module will go to sleep again an start at step 5 after 3 min.

Source:²https://github.com/Edzelf/LoRa/blob/master/ESP_lora_tracker/ESP_LoRa_tracker.ino
