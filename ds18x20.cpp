#include "ds18x20.h"

static dma_tube_config tube_config_rx;
static dma_tube_config tube_config_tx;
static dma_channel USART_RX_DMA_TUBE;
static dma_channel USART_TX_DMA_TUBE;
static dma_dev *DEV_DMA;
static dma_request_src  DMA_REQ_RX_SRC;
static dma_request_src  DMA_REQ_TX_SRC;
static HardwareSerial *serial_ds;

DS18x20Serial::DS18x20Serial(uint8_t NumUSART) {
  DEV_DMA = DMA1;
  switch (NumUSART) {
    case 1:
      serial_ds = &Serial1;
      USART_RX_DMA_TUBE = DMA_CH5;
      USART_TX_DMA_TUBE = DMA_CH4;
      DMA_REQ_RX_SRC = DMA_REQ_SRC_USART1_RX;
      DMA_REQ_TX_SRC = DMA_REQ_SRC_USART1_TX;
      break;
    case 2:
      serial_ds = &Serial2;
      USART_RX_DMA_TUBE = DMA_CH6;
      USART_TX_DMA_TUBE = DMA_CH7;
      DMA_REQ_RX_SRC = DMA_REQ_SRC_USART2_RX;
      DMA_REQ_TX_SRC = DMA_REQ_SRC_USART2_TX;
      break;
    case 3:
      serial_ds = &Serial3;
      USART_RX_DMA_TUBE = DMA_CH3;
      USART_TX_DMA_TUBE = DMA_CH2;
      DMA_REQ_RX_SRC = DMA_REQ_SRC_USART3_RX;
      DMA_REQ_TX_SRC = DMA_REQ_SRC_USART3_TX;
      break;
    default:
      break;
  }

}

static uint8_t rx_buf[36] = {0};

static uint8_t convert_T[] = {
  OW_0, OW_0, OW_1, OW_1, OW_0, OW_0, OW_1, OW_1, // 0xcc SKIP ROM
  OW_0, OW_0, OW_1, OW_0, OW_0, OW_0, OW_1, OW_0  // 0x44 CONVERT
};

static uint8_t read_scratch[] = {
  OW_0, OW_0, OW_1, OW_1, OW_0, OW_0, OW_1, OW_1, // 0xcc SKIP ROM
  OW_0, OW_1, OW_1, OW_1, OW_1, OW_1, OW_0, OW_1, // 0xbe READ SCRATCH
  OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R,
  OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R, OW_R
};

static volatile  uint8_t scratch[sizeof(read_scratch)];
static volatile uint32 irq_fired_rx = 0;
static volatile uint32 irq_fired_tx = 0;
static uint8_t  delay_800_ms = 0;
static volatile usart_dev *serial_dev;

static void rx_dma_irq(void) {
  irq_fired_rx = 1;
  dma_disable(DEV_DMA, USART_RX_DMA_TUBE);
}

static void tx_dma_irq(void) {
  irq_fired_tx = 1;
  dma_disable(DEV_DMA, USART_TX_DMA_TUBE);
}

static void irq_100_Hz(void) {
  delay_800_ms++;
}

void DS18x20Serial::setup_usart(void) {
  serial_ds->begin(BAUD);
  serial_dev = serial_ds->c_dev();
  serial_dev->regs->CR2 &= ~USART_CR2_CLKEN;
  serial_dev->regs->CR2 &= ~USART_CR2_LINEN;
  serial_dev->regs->CR3 &= ~USART_CR3_SCEN;
  serial_dev->regs->CR3 &= ~USART_CR3_IREN;
  serial_dev->regs->CR3 = USART_CR3_DMAR | USART_CR3_DMAT | USART_CR3_HDSEL; //H-D Enable DMA Rx & Tx
}

// Set up our dma_tube_config structure.
void DS18x20Serial::setup_tube_config_rx(void) {
  tube_config_rx.tube_src = &serial_ds->c_dev()->regs->DR;
  tube_config_rx.tube_src_size = DMA_SIZE_8BITS;
  tube_config_rx.tube_dst = &rx_buf[0];
  tube_config_rx.tube_dst_size = DMA_SIZE_8BITS;
  tube_config_rx.tube_nr_xfers = 1;
  tube_config_rx.tube_flags = DMA_CFG_DST_INC | DMA_CFG_CMPLT_IE | DMA_CFG_ERR_IE;
  tube_config_rx.target_data = NULL;
  tube_config_rx.tube_req_src = DMA_REQ_RX_SRC;
}
void DS18x20Serial::setup_tube_config_tx(void) {
  tube_config_tx.tube_src = &convert_T[0];
  tube_config_tx.tube_src_size = DMA_SIZE_8BITS;
  tube_config_tx.tube_dst = &serial_ds->c_dev()->regs->DR;
  tube_config_tx.tube_dst_size = DMA_SIZE_8BITS;
  tube_config_tx.tube_nr_xfers = 1;
  tube_config_tx.tube_flags = DMA_CFG_SRC_INC | DMA_CFG_CMPLT_IE | DMA_CFG_ERR_IE;
  tube_config_tx.target_data = NULL;
  tube_config_tx.tube_req_src = DMA_REQ_TX_SRC;
}
// Configure the DMA controller to serve DMA requests from the USART.
void DS18x20Serial::setup_dma_xfer(void) {
  dma_init(DEV_DMA);
  //-----------------RX-------------------
  int status = dma_tube_cfg(DEV_DMA, USART_RX_DMA_TUBE, &tube_config_rx);
  ASSERT(status == DMA_TUBE_CFG_SUCCESS);
  dma_set_priority(DEV_DMA, USART_RX_DMA_TUBE, DMA_PRIORITY_MEDIUM);
  dma_attach_interrupt(DMA1, USART_RX_DMA_TUBE, rx_dma_irq);
  //-----------------TX-------------------
  status = dma_tube_cfg(DEV_DMA, USART_TX_DMA_TUBE, &tube_config_tx);
  ASSERT(status == DMA_TUBE_CFG_SUCCESS);
  dma_set_priority(DEV_DMA, USART_RX_DMA_TUBE, DMA_PRIORITY_MEDIUM);
  dma_attach_interrupt(DEV_DMA, USART_TX_DMA_TUBE, tx_dma_irq);
}

void DS18x20Serial::DS18x20SerialInit(void) {
  delay_800_ms = 0;
  setup_tube_config_rx();
  setup_tube_config_tx();
  setup_dma_xfer();
  setup_usart();
  //------- Setup Timer -------------
  Timer2.setChannel1Mode(TIMER_OUTPUTCOMPARE);
  Timer2.setPeriod(10000); // in microseconds
  Timer2.setCompare1(1); // overflow might be small
  Timer2.attachCompare1Interrupt(irq_100_Hz);
  //---------------------------------

}

inline bool wait_OW_Reset () {
  return serial_dev->regs->SR & USART_SR_TC;
}


void DS18x20Serial::OW_Reset(void) {
  dma_disable(DEV_DMA, USART_TX_DMA_TUBE);
  serial_dev->regs->CR1 &= ~USART_CR1_UE;
  while (serial_dev->regs->CR1 & USART_CR1_UE) {};
  //Change Baudrate to 9600
  serial_dev->regs->BRR = 3750;
  serial_dev->regs->CR1 |= USART_CR1_UE;

  serial_ds->write(0xf0);
}
inline void OW_ResetStop() {
  serial_dev->regs->CR1 &= ~USART_CR1_UE;
  while (serial_dev->regs->CR1 & USART_CR1_UE) {};
  //Change usart baudrate to 115200
  serial_dev->regs->BRR = 312;
  serial_dev->regs->CR1 |= USART_CR1_UE;

}



inline bool wait_OW_SendCommand () {
  return irq_fired_tx;
}
inline bool wait_OW_ReadData () {
  return irq_fired_rx;
}

void DS18x20Serial::OW_SendCommand(void) {
  irq_fired_tx = 0;
  dma_set_mem_addr(DEV_DMA, USART_TX_DMA_TUBE, &convert_T[0]);
  dma_set_num_transfers(DEV_DMA, USART_TX_DMA_TUBE, sizeof(convert_T));
  dma_enable(DEV_DMA, USART_TX_DMA_TUBE);
}

void DS18x20Serial::OW_SendCommandReadData(void) {
  irq_fired_rx = 0;
  dma_set_mem_addr(DEV_DMA, USART_RX_DMA_TUBE, &rx_buf[0]);
  dma_set_num_transfers(DEV_DMA, USART_RX_DMA_TUBE, 33);
  dma_enable(DEV_DMA, USART_RX_DMA_TUBE);

  irq_fired_tx = 0;
  dma_set_mem_addr(DEV_DMA, USART_TX_DMA_TUBE, &read_scratch[0]);
  dma_set_num_transfers(DEV_DMA, USART_TX_DMA_TUBE, sizeof(read_scratch));
  dma_enable(DEV_DMA, USART_TX_DMA_TUBE);
}

bool DS18x20Serial::OW_ConvertDataTT(void) {
  tt = 0;
  for (int i = 17; i < 33; i++) {
    if (rx_buf[i] == 0xff) {
      tt = (tt >> 1) | 0x8000;
    } else {
      tt = tt >> 1;
    }
  }
  if (rx_buf[0] == 0xF0) return false;
  return true;
}
void DS18x20Serial::ds18x20_state_machine(void) {
  switch (ds18x20_state) {
    case 1:
      OW_Reset();
      ds18x20_state = 2;
      ds18x20_state_stop = false;
      break;
    case 2:
      if (wait_OW_Reset ())ds18x20_state = 3;
      break;
    case 3:
      OW_ResetStop(); ds18x20_state = 4;
      break;
    case 4:
      OW_SendCommand(); //SKIP ROM CONVERT T start conversion
      ds18x20_state = 5;
      break;
    case 5:
      if (wait_OW_SendCommand ()) {
        ds18x20_state = 6;
        delay_800_ms = 0;
      }
      break;
    case 6:
      if (delay_800_ms > 80)ds18x20_state = 7;
      break;
    case 7:
      OW_Reset(); ds18x20_state = 8;
      break;
    case 8:
      if (wait_OW_Reset ())ds18x20_state = 9;
      break;
    case 9:
      OW_ResetStop(); ds18x20_state = 10;
      break;
    case 10:
      OW_SendCommandReadData();// SKIP ROM Read Scratchpad
      ds18x20_state = 11;
      break;
    case 11:
      if (wait_OW_ReadData ())ds18x20_state = 12;
      break;
    case 12:
      ds18x20_present = OW_ConvertDataTT();
      ds18x20_state = DS18x20_STATE_FINISH;
      ds18x20_state_stop = true;
      break;
    default:
      ds18x20_state_stop = false;
      break;
  }
}

