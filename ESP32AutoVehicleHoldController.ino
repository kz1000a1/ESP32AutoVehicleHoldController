//
// AVH(Auto Vehicle Hold) auto introduce and remove system firmware for SUBARU Levorg VN5
//
#include "driver/twai.h"
#include "subaru_levorg_vnx.h"

// Pins used to connect to CAN bus transceiver:
// #define RX_PIN GPIO_NUM_21
// #define TX_PIN GPIO_NUM_20
// #define RX_PIN GPIO_NUM_19
// #define TX_PIN GPIO_NUM_22
#define RX_PIN GPIO_NUM_32
#define TX_PIN GPIO_NUM_26

#define POLLING_RATE_MS 1000
static bool driver_installed = false;

enum debug_mode DebugMode = NORMAL;


void print_frame(twai_message_t* twai_frame) {
  uint32_t CurrentTime;

  CurrentTime = micros();

  // Output all received message(s) to CDC port as candump -L
  if (twai_frame->rtr == 0) {  // Data Frame
    Serial.printf("(%d.%06d) can0 %03X#", CurrentTime / 1000000,
                  CurrentTime % 1000000,
                  twai_frame->identifier);
    for (uint8_t i = 0; i < twai_frame->data_length_code; i++) {
      Serial.printf("%02X", twai_frame->data[i]);
    }
    Serial.printf("\n");
  } else {  // Remote Frame
    Serial.printf("(%d.%06d) can0 %03X#R%d\n", CurrentTime / 1000000,
                  CurrentTime % 1000000,
                  twai_frame->identifier,
                  twai_frame->data_length_code);
  }
}

void transmit_can_frame(twai_message_t* rx_frame, uint8_t Remove) {
  // Storage for transmit message buffer
  twai_message_t tx_frame;
  tx_frame.identifier = CAN_ID_CCU;
  tx_frame.data_length_code = 8;
  tx_frame.rtr = 0;
  tx_frame.extd = 0;
  tx_frame.ss = 1;
  tx_frame.self = 0;
  tx_frame.dlc_non_comp = 0;

  if ((rx_frame->data[1] & 0x0f) == 0x0f) {
    tx_frame.data[1] = rx_frame->data[1] &= 0xf0;
  } else {
    tx_frame.data[1] = rx_frame->data[1] += 0x01;
  }
  
  if (Remove) {
    tx_frame.data[2] = rx_frame->data[2] | 0x01;  // Remove auto behicle hold bit on
  } else {
    tx_frame.data[2] = rx_frame->data[2] | 0x02;  // Introduce auto behicle hold bit on
  }
  
  tx_frame.data[2] = rx_frame->data[2] | 0x01;  // Disable auto behicle hold bit on
  tx_frame.data[3] = rx_frame->data[3];
  tx_frame.data[4] = rx_frame->data[4];
  tx_frame.data[5] = rx_frame->data[5];
  tx_frame.data[6] = rx_frame->data[6];
  tx_frame.data[7] = rx_frame->data[7];
  // Calculate checksum
  tx_frame.data[0] = (tx_frame.data[1] + tx_frame.data[2] + tx_frame.data[3] + tx_frame.data[4] + tx_frame.data[5] + tx_frame.data[6] + tx_frame.data[7]) + SUM_CHECK_ADDER;
  if (twai_transmit(&tx_frame, pdMS_TO_TICKS(1000)) != ESP_OK) {
    if (DebugMode == DEBUG) {
      Serial.printf("# Error: Failed to queue message for transmission\n");
    }
  }
  if (DebugMode == DEBUG) {
    Serial.printf("# ");
    print_frame(&tx_frame);
  }
}

bool if_can_message_receive_is_pendig() {

  uint32_t alerts_triggered;
  twai_status_info_t twaistatus;

  // Check if alert happened
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));

  if (DebugMode == DEBUG) {
  　twai_get_status_info(&twaistatus);

  　// Handle alerts
  　if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
      Serial.println("# Alert: TWAI controller has become error passive.");
    }
    
    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      Serial.println("# Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
      Serial.printf("# Bus error count: %d\n", twaistatus.bus_error_count);
    }
    
    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      Serial.println("# Alert: The RX queue is full causing a received frame to be lost.");
      Serial.printf("# RX buffered: %d\t", twaistatus.msgs_to_rx);
      Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
      Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
    }
  }
  
  // If CAN message receive is pending, process the message
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    return true;
  } else {
    return false;
  }
}


void setup() {
  if (DebugMode != NORMAL) {
    Serial.begin(115200);
    while (!Serial)
      ;
  }

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  // twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  twai_filter_config_t f_config = { .acceptance_code = CAN_ID_CCU << 21, .acceptance_mask = (0x7f3 << 21) | 0x1fffff, .single_filter = true };

  if (DebugMode == CANDUMP) {
    f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  }
  
  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    if (DebugMode == DEBUG) {
      Serial.println("# Error: Failed to install driver");
    }
    return;
  }

  // Start TWAI driver
  if (twai_start() != ESP_OK) {
    if (DebugMode == DEBUG) {
      Serial.println("# Error: Failed to start driver");
    }
    return;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) != ESP_OK) {
    if (DebugMode == DEBUG) {
      Serial.println("# Error: Failed to reconfigure alerts");
    }
    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;
}


void loop() {
  twai_message_t rx_frame;

  static enum cu_status TcuStatus = ENGINE_STOP;
  static enum cu_status ScuStatus = ENGINE_STOP;
  static enum cu_status CcuStatus = ENGINE_STOP;
  static enum status Status = PROCESSING;
  static uint16_t PreviousCanId = CAN_ID_CCU;
  static uint8_t Retry = 0;
  static uint8_t R_Gear = 0;
  static float Speed = 0;

  if (!driver_installed) {
    // Driver not installed
    delay(1000);
    return;
  }

  // If CAN message receive is pending, process the message
  if (if_can_message_receive_is_pendig()) {
    // One or more messages received. Handle all.
    while (twai_receive(&rx_frame, 0) == ESP_OK) {
      if (DebugMode == CANDUMP || (DebugMode == DEBUG && (rx_frame.identifier == CAN_ID_CCU || rx_frame.identifier == CAN_ID_TCU || rx_frame.identifier == CAN_ID_MCU || rx_frame.identifier == CAN_ID_SCU))) {
        print_frame(&rx_frame);
      }

      if (rx_frame.rtr != 0 || rx_frame.data_length_code != 8) {
        continue;
      }

      if (DebugMode != CANDUMP) {
        switch (rx_frame.identifier) {
          case CAN_ID_MCU:
            Speed = (rx_frame.data[2] + ((rx_frame.data[3] & 0x1f) << 8)) * 0.05625;
            break;

          case CAN_ID_TCU:
            if (R_Gear != (rx_frame.data[3] == 0x03)) {
              if (Status == SUCCEEDED) {
                Status = PROCESSING;
              }
              if (DebugMode == DEBUG) {
                if (R_Gear) {
                  // Output Information message
                  Serial.printf("# Information: Change Reverse to Another Gear.\n");
                } else {
                  // Output Information message
                  Serial.printf("# Information: Change Another to Reverse Gear.\n");
                }
              }
            }
            TcuStatus = READY;
            R_Gear = (rx_frame.data[3] == 0x03);
            PreviousCanId = rx_frame.identifier;
            break;

          case CAN_ID_SCU:
            if (rx_frame.data[5] & 0x20) {
              ScuStatus = AVH_ON;
              if (DebugMode == DEBUG) {
                // Output Information message
                Serial.printf("# Information: Auto vehicle hold On.\n");
              }
              if (Retry != 0 && Status == PROCESSING && (!R_Gear)) {
                if (DebugMode == DEBUG) {
                  // Output Information message
                  Serial.printf("# Information: Enable auto vehicle hold succeeded.\n");
                }
                Retry = 0;
                Status = SUCCEEDED;
                CcuStatus = READY;
              }
            } else {
              ScuStatus = AVH_OFF;
              if (DebugMode == DEBUG) {
                // Output Information message
                Serial.printf("# Information: Auto vehicle hold Off.\n");
              }
              if (Retry != 0 && Status == PROCESSING && R_Gear) {
                if (DebugMode == DEBUG) {
                  // Output Information message
                  Serial.printf("# Information: Disble auto vehicle hold succeeded.\n");
                }
                Retry = 0;
                Status = SUCCEEDED;
                CcuStatus = READY;
              }
            }

            // PreviousCanId = rx_frame.identifier;
            break;

          case CAN_ID_CCU:
            if (PreviousCanId != CAN_ID_TCU) {  // TCU don't transmit message
              TcuStatus = ENGINE_STOP;
              ScuStatus = ENGINE_STOP;
              CcuStatus = ENGINE_STOP;
              Status = PROCESSING;
              Retry = 0;
              R_Gear = 0;
              Speed = 0;
              if (DebugMode == DEBUG) {
                // Output Information message
                Serial.printf("# Information: ENGINE STOP.\n");
              }
            } else if (rx_frame.data[2] & 0x03) {
              if (DebugMode == DEBUG) {
                // Output Information message
                Serial.printf("# Information: Eliminate engine auto stop cancelled.\n");
              }
              Status = CANCELLED;
            } else if (Status == PROCESSING) {
              if (CcuStatus == NOT_READY || CcuStatus == ENGINE_STOP || (ScuStatus == AVH_OFF && R_Gear) || (ScuStatus == AVH_ON && (!R_Gear))) {
                Serial.printf("# Information: Status (CCU=%d SCU=%d TCU=%d R=%d).\n", CcuStatus, ScuStatus, TcuStatus, R_Gear);
                CcuStatus = READY;
                if (DebugMode == DEBUG) {
                  // Output Information message
                  Serial.printf("# Information: READY.\n");
                  Serial.printf("# Information: Status (CCU=%d SCU=%d TCU=%d R=%d).\n", CcuStatus, ScuStatus, TcuStatus, R_Gear);
                }
              // } else if (ScuStatus == AVH_OFF && (!R_Gear) || (ScuStatus == AVH_ON && R_Gear)) {  // Transmit message for Enable or disable auto vehicle hold
              } else if ((ScuStatus == AVH_OFF && (!R_Gear) && 15 < Speed) || (ScuStatus == AVH_ON && R_Gear)) {  // Transmit message for Enable or disable auto vehicle hold
                if (DebugMode == DEBUG) {
                  // Output Information message
                  Serial.printf("# Information: Send Frame Speed=%d R=%d.\n", (int)Speed, R_Gear);
                }
                if (MAX_RETRY <= Retry) {  // Previous enable or disable auto vehicle hold message failed
                  if (DebugMode == DEBUG) {
                    // Output Warning message
                    Serial.printf("# Warning: Enable or disable auto vehicle hold failed\n");
                  }
                  Status = FAILED;
                } else {
                  Retry++;
                  for (int i = 0; i < 5; i++) {
                    delay(50);
                    transmit_can_frame(&rx_frame, R_Gear);  // Transmit message
                  }
                  // Discard message(s) that received during HAL_delay()
                  twai_clear_receive_queue();
                  CcuStatus = NOT_READY;
                  ScuStatus = NOT_READY;
                }
              } else {  // Unexpected case
                if (DebugMode == DEBUG) {
                  // Output Warning message
                  Serial.printf("# Warning: Unexpected case (CCU=%d SCU=%d TCU=%d Speed=%d).\n", CcuStatus, ScuStatus, TcuStatus, (int)Speed);
                }
              }
            }
            PreviousCanId = rx_frame.identifier;
            break;

            // default:  // Unexpected can id
            // if (DebugMode == DEBUG) {
            // Output Warning message
            // Serial.printf("# Warning: Unexpected can id (0x%03x).\n", rx_frame.identifier);
            // }
            // break;
        }
      }
    }
  }
}
