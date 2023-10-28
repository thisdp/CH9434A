#include "CH9434.h"

CH9434::CH9434(SPIClass &spiClass, uint32_t xtFreqArg, bool pllEnabledArg, uint8_t clkDivNumArg){
  spi = &spiClass;
  gpioValue = 0;
  lowPower = 0;
  sysFreq = 32000000;
  xtFreq = xtFreqArg;
  pllEnabled = pllEnabledArg;
  clkDivNum = clkDivNumArg;
}

bool CH9434::begin(uint8_t ss, int8_t sck, int8_t miso, int8_t mosi){
  cs = ss;
  ::pinMode(cs,OUTPUT);
  ::digitalWrite(cs,HIGH);
  spi->begin(sck,miso,mosi);
  SPISettings settings(1000000,MSBFIRST,SPI_MODE0);
  spi->beginTransaction(settings);
  uint8_t clk_ctrl_reg = 0;
  if(pllEnabled) clk_ctrl_reg |= (1<<7);
  //if(xtEnabled) clk_ctrl_reg |= (1<<6);
  clk_ctrl_reg |= (clkDivNum&0x1f);
  /* 计算当前的串口基准时钟 */
  uint32_t targetSysFreq = 32000000;
  switch(clk_ctrl_reg&0xc0)	{
    case 0x00:
    case 0x40: //外部晶振提供时钟
      if((xtFreq > 36000000)||(xtFreq < 24000000)) return false; //时钟错误
      targetSysFreq = xtFreq;
      break;
    case 0x80:
    case 0xC0: //使用外部晶振，并开启倍频
      if((xtFreq> 36000000)||(xtFreq < 24000000)) return false; //时钟错误
      targetSysFreq = xtFreq*15/(clkDivNum&0x1f);
      if(targetSysFreq > 40000000) return false;//时钟错误
      break;
  }
  sysFreq = targetSysFreq;
  writeRegister(CH9434_CLK_CTRL_CFG_ADD, clk_ctrl_reg);
  delay(50);
  return true;
}

void CH9434::sleep(){
  writeRegister(CH9434_SLEEP_MOD_CFG_ADD, CH9434_LOWPOWER_SLEEP);
}

void CH9434::wakeUp(){
  ::digitalWrite(cs,HIGH);
  delayMicroseconds(2);
  ::digitalWrite(cs,LOW);
}

void CH9434::pinMode(uint8_t gpio_idx,int8_t mode){
  switch(mode){
    case -1:  //禁用
      setGPIOEnabled(gpio_idx,false);  //失能
      break;
    case OUTPUT:
      setGPIOPullUpEnabled(gpio_idx,false); //关闭上拉
      setGPIOPullDownEnabled(gpio_idx,false); //关闭下拉
      setGPIODirection(gpio_idx,true);  //输出
      setGPIOEnabled(gpio_idx,true);  //使能
      break;
    case INPUT_PULLUP:
      setGPIODirection(gpio_idx,false);  //输入
      setGPIOPullUpEnabled(gpio_idx,true); //开启上拉
      setGPIOPullDownEnabled(gpio_idx,false); //关闭下拉
      setGPIOEnabled(gpio_idx,true);  //使能
      break;
    case INPUT_PULLDOWN:
      setGPIODirection(gpio_idx,false);  //输入
      setGPIOPullUpEnabled(gpio_idx,false); //关闭上拉
      setGPIOPullDownEnabled(gpio_idx,true); //开启下拉
      setGPIOEnabled(gpio_idx,true);  //使能
      break;
    default:  //如果没有的，则默认设置为输入
      setGPIOPullUpEnabled(gpio_idx,false); //关闭上拉
      setGPIOPullDownEnabled(gpio_idx,false); //关闭下拉
      setGPIODirection(gpio_idx,false);  //输入
      setGPIOEnabled(gpio_idx,true);  //使能
      break;
  }
}

void CH9434::digitalWrite(uint8_t gpio_idx,uint8_t out_val){
  switch(out_val){
    case HIGH:
      gpioValue |= (1<<gpio_idx);
    break;
    case LOW:
      gpioValue &= ~(1<<gpio_idx);
    break;
    default:
    break;
  }
  uint8_t pin_val_reg = (uint8_t)(gpioValue>>((gpio_idx/8)*8));
  writeRegister(CH9434_GPIO_PIN_VAL+(gpio_idx/8), pin_val_reg);
}

uint8_t CH9434::digitalRead(uint8_t gpio_idx){
  uint8_t pin_val_reg = readRegister(CH9434_GPIO_PIN_VAL+(gpio_idx/8));
  return (pin_val_reg>>(gpio_idx%8))&1;
}

uint32_t CH9434::getSysFreq(){
  return sysFreq;
}

/*************************Private***************************/
void CH9434::setGPIOEnabled(uint8_t gpio_idx, uint8_t enabled){ //GPIO使能
  uint8_t gpio_func_reg = readRegister(CH9434_GPIO_FUNC_EN+(gpio_idx/8));
  if(enabled) gpio_func_reg |= (1<<(gpio_idx%8));
  else        gpio_func_reg &=~(1<<(gpio_idx%8));
  writeRegister(CH9434_GPIO_FUNC_EN+(gpio_idx/8), gpio_func_reg);
}

void CH9434::setGPIODirection(uint8_t gpio_idx, uint8_t dir){ //GPIO方向设置
  uint8_t gpio_dir_reg = readRegister(CH9434_GPIO_DIR_MOD+(gpio_idx/8));
  if(dir) gpio_dir_reg |= (1<<(gpio_idx%8));
  else    gpio_dir_reg &=~(1<<(gpio_idx%8));
  writeRegister(CH9434_GPIO_DIR_MOD+(gpio_idx/8), gpio_dir_reg);
}

void CH9434::setGPIOPullUpEnabled(uint8_t gpio_idx, uint8_t pu){ //GPIO上拉设置
  uint8_t gpio_pu_reg = readRegister(CH9434_GPIO_PU_MOD+(gpio_idx/8));
  if(pu) gpio_pu_reg |= (1<<(gpio_idx%8));
  else   gpio_pu_reg &=~(1<<(gpio_idx%8));
  writeRegister(CH9434_GPIO_PU_MOD+(gpio_idx/8), gpio_pu_reg);
}

void CH9434::setGPIOPullDownEnabled(uint8_t gpio_idx, uint8_t pd){ //GPIO下拉设置
  uint8_t gpio_pd_reg = readRegister(CH9434_GPIO_PD_MOD+(gpio_idx/8));
  if(pd) gpio_pd_reg |= (1<<(gpio_idx%8));
  else   gpio_pd_reg &=~(1<<(gpio_idx%8));
  writeRegister(CH9434_GPIO_PD_MOD+(gpio_idx/8), gpio_pd_reg);
}

bool CH9434::getGPIOEnabled(uint8_t gpio_idx){ //GPIO使能
  uint8_t gpio_func_reg = readRegister(CH9434_GPIO_FUNC_EN+(gpio_idx/8));
  return (gpio_func_reg>>(gpio_idx%8))&0x01;
}

bool CH9434::getGPIODirection(uint8_t gpio_idx){ //GPIO方向设置
  uint8_t gpio_dir_reg = readRegister(CH9434_GPIO_DIR_MOD+(gpio_idx/8));
  return (gpio_dir_reg>>(gpio_idx%8))&0x01;
}

bool CH9434::getGPIOPullUpEnabled(uint8_t gpio_idx){ //GPIO上拉设置
  uint8_t gpio_pu_reg = readRegister(CH9434_GPIO_PU_MOD+(gpio_idx/8));
  return (gpio_pu_reg>>(gpio_idx%8))&0x01;
}

bool CH9434::getGPIOPullDownEnabled(uint8_t gpio_idx){ //GPIO下拉设置
  uint8_t gpio_pd_reg = readRegister(CH9434_GPIO_PD_MOD+(gpio_idx/8));
  return (gpio_pd_reg>>(gpio_idx%8))&0x01;
}

uint8_t CH9434::readRegister(uint8_t reg){
  ::digitalWrite(cs,LOW);
  spi->transfer(CH9434_REG_OP_READ|reg);
  delayMicroseconds(3);
  uint8_t data = spi->transfer(0xff);
  delayMicroseconds(1);
  ::digitalWrite(cs,HIGH);
  return data;
}

void CH9434::writeRegister(uint8_t reg, uint8_t data){
  ::digitalWrite(cs,LOW);
  spi->transfer(CH9434_REG_OP_WRITE|reg);
  delayMicroseconds(1);
  spi->transfer(data);
  delayMicroseconds(3);
  ::digitalWrite(cs,HIGH);
}


/*********************************************************************串口********************************************************************/
SPISerial::SPISerial(CH9434 *masterControllerArg, uint8_t uartIndexArg){
  master = masterControllerArg;
  uartIndex = uartIndexArg;
  peekedDataAvailable = 0;
}

SPISerial::SPISerial(CH9434 &masterControllerArg, uint8_t uartIndexArg){
  master = &masterControllerArg;
  uartIndex = uartIndexArg;
}

size_t SPISerial::write(uint8_t data){
  setTxFIFOData(data);
  return 0;
}

size_t SPISerial::write(uint8_t *data, uint16_t length){
  setTxFIFOData(data,length);
  return 0;
}

int SPISerial::available(){
  return getRxFIFOLength();
}

int SPISerial::read(){
  if(peekedDataAvailable){
    peekedDataAvailable = false;
    return peekedData;
  }
  return getRxFIFOData();
}

int SPISerial::peek(){
  if(!peekedDataAvailable){
    peekedData = read();
    peekedDataAvailable = true;
  }
  return peekedData;
}

bool SPISerial::begin(unsigned long baud, uint32_t config){
  uint8_t dataBits = 8;
  uint8_t verifyBits = CH9434_UART_NO_PARITY;
  uint8_t stopBits = 1;
  switch(config){
    case SERIAL_5N1: dataBits = 5, verifyBits = CH9434_UART_NO_PARITY,   stopBits = 1; break;
    case SERIAL_6N1: dataBits = 6, verifyBits = CH9434_UART_NO_PARITY,   stopBits = 1; break;
    case SERIAL_7N1: dataBits = 7, verifyBits = CH9434_UART_NO_PARITY,   stopBits = 1; break;
    case SERIAL_8N1: dataBits = 8, verifyBits = CH9434_UART_NO_PARITY,   stopBits = 1; break;
    case SERIAL_5N2: dataBits = 5, verifyBits = CH9434_UART_NO_PARITY,   stopBits = 2; break;
    case SERIAL_6N2: dataBits = 6, verifyBits = CH9434_UART_NO_PARITY,   stopBits = 2; break;
    case SERIAL_7N2: dataBits = 7, verifyBits = CH9434_UART_NO_PARITY,   stopBits = 2; break;
    case SERIAL_8N2: dataBits = 8, verifyBits = CH9434_UART_NO_PARITY,   stopBits = 2; break;
    case SERIAL_5E1: dataBits = 5, verifyBits = CH9434_UART_EVEN_PARITY, stopBits = 1; break;
    case SERIAL_6E1: dataBits = 6, verifyBits = CH9434_UART_EVEN_PARITY, stopBits = 1; break;
    case SERIAL_7E1: dataBits = 7, verifyBits = CH9434_UART_EVEN_PARITY, stopBits = 1; break;
    case SERIAL_8E1: dataBits = 8, verifyBits = CH9434_UART_EVEN_PARITY, stopBits = 1; break;
    case SERIAL_5E2: dataBits = 5, verifyBits = CH9434_UART_EVEN_PARITY, stopBits = 2; break;
    case SERIAL_6E2: dataBits = 6, verifyBits = CH9434_UART_EVEN_PARITY, stopBits = 2; break;
    case SERIAL_7E2: dataBits = 7, verifyBits = CH9434_UART_EVEN_PARITY, stopBits = 2; break;
    case SERIAL_8E2: dataBits = 8, verifyBits = CH9434_UART_EVEN_PARITY, stopBits = 2; break;
    case SERIAL_5O1: dataBits = 5, verifyBits = CH9434_UART_ODD_PARITY,  stopBits = 1; break;
    case SERIAL_6O1: dataBits = 6, verifyBits = CH9434_UART_ODD_PARITY,  stopBits = 1; break;
    case SERIAL_7O1: dataBits = 7, verifyBits = CH9434_UART_ODD_PARITY,  stopBits = 1; break;
    case SERIAL_8O1: dataBits = 8, verifyBits = CH9434_UART_ODD_PARITY,  stopBits = 1; break;
    case SERIAL_5O2: dataBits = 5, verifyBits = CH9434_UART_ODD_PARITY,  stopBits = 2; break;
    case SERIAL_6O2: dataBits = 6, verifyBits = CH9434_UART_ODD_PARITY,  stopBits = 2; break;
    case SERIAL_7O2: dataBits = 7, verifyBits = CH9434_UART_ODD_PARITY,  stopBits = 2; break;
    case SERIAL_8O2: dataBits = 8, verifyBits = CH9434_UART_ODD_PARITY,  stopBits = 2; break;
  }
  setParameters(baud,dataBits,verifyBits,stopBits);
  setFIFO(true,CH9434_UART_FIFO_MODE_1280);
  return true;
}

/*************************************************
* 描述: 串口参数设置
* 输入: 
        bps：串口的波特率
        data_bits：数据位
        veri_bits：校验位
        stop_bits：停止位
* 输出: 无
* 返回: 无
*************************************************/
void SPISerial::setParameters(uint32_t bps,uint8_t data_bits,uint8_t veri_bits,uint8_t stop_bits){
  uint32_t x = ( ( 10*master->getSysFreq()/8/bps ) + 5 ) / 10;	
  uint8_t uart_reg_dll = x&0xff;
  uint8_t uart_reg_dlm = (x>>8)&0xff;
  
  //DLAB置位 设置LCR寄存器
  uint8_t uart_reg_lcr = master->readRegister(CH9434_UARTx_LCR_ADD+0x10*uartIndex);
  
  uart_reg_lcr |= CH9434_UARTx_BIT_DLAB;
  //数据位
  uart_reg_lcr &= ~0x03;
  switch(data_bits)    {
    case CH9434_UART_5_BITS_PER_CHAR:
      break;
    case CH9434_UART_6_BITS_PER_CHAR:
      uart_reg_lcr |= 0x01;
      break;
    case CH9434_UART_7_BITS_PER_CHAR:
      uart_reg_lcr |= 0x02;
      break;
    case CH9434_UART_8_BITS_PER_CHAR:
      uart_reg_lcr |= 0x03;
      break;
    default:
      uart_reg_lcr |= 0x03;
      break;	
  }
  //停止位
  uart_reg_lcr &= ~(1<<2);
  if(stop_bits == CH9434_UART_TWO_STOP_BITS) uart_reg_lcr |= (1<<2);

  //校验位
  uart_reg_lcr &= ~(1<<3);
  uart_reg_lcr &= ~(3<<4);
  switch(veri_bits)    {
    case CH9434_UART_NO_PARITY:
      break;
    case CH9434_UART_ODD_PARITY:
      uart_reg_lcr |= (1<<3);
      break;
    case CH9434_UART_EVEN_PARITY:
      uart_reg_lcr |= (1<<3);
      uart_reg_lcr |= (1<<4);
      break;
    case CH9434_UART_MARK_PARITY:
      uart_reg_lcr |= (1<<3);
      uart_reg_lcr |= (2<<4);
      break;
    case CH9434_UART_SPACE_PARITY:
      uart_reg_lcr |= (1<<3);
      uart_reg_lcr |= (3<<4);
      break;
    default:
      break;	
  }
  master->writeRegister(CH9434_UARTx_LCR_ADD+0x10*uartIndex, uart_reg_lcr);
  //设置DLL
  master->writeRegister(CH9434_UARTx_DLL_ADD+0x10*uartIndex, uart_reg_dll);
  //设置DLM
  master->writeRegister(CH9434_UARTx_DLM_ADD+0x10*uartIndex, uart_reg_dlm);
  //DLAB清0
  uart_reg_lcr &= ~CH9434_UARTx_BIT_DLAB;
  master->writeRegister(CH9434_UARTx_LCR_ADD+0x10*uartIndex, uart_reg_lcr);
}

/*************************************************
* 描述: 串口FIFO设置
* 输入: 
        fifo_en：FIFO功能使能
        fifo_level：FIFO触发等级
* 输出: 无
* 返回: 无
*************************************************/
void SPISerial::setFIFO(uint8_t fifo_en,uint8_t fifo_level){
  uint8_t uart_reg_fcr = 0;
  if(fifo_en){
    uart_reg_fcr |= 0x01;
    uart_reg_fcr |= fifo_level<<6;
  }
  master->writeRegister(CH9434_UARTx_FCR_ADD+0x10*uartIndex, uart_reg_fcr);
}

/*************************************************
* 描述: 串口中断设置
* 输入: 
        modem：modem信号中断
        line：线路状态中断
        tx：发送中断
        rx：接收中断
* 输出: 无
* 返回: 无
*************************************************/
void SPISerial::setIRQ(uint8_t modem,uint8_t line,uint8_t tx,uint8_t rx){
  uint8_t  uart_reg_ier = 0;
  if(modem) uart_reg_ier |= (1<<3);
  if(line)	uart_reg_ier |= (1<<2);
  if(tx)	  uart_reg_ier |= (1<<1);
  if(rx)	  uart_reg_ier |= (1<<0);
  master->writeRegister(CH9434_UARTx_IER_ADD+0x10*uartIndex, uart_reg_ier);
}

/* 流控功能和引脚设置 */
/*************************************************
* 描述: 流控设置
* 输入:
        flow_en：流控使能
* 输出: 无
* 返回: 无
*************************************************/
void SPISerial::setFlowControlEnabled(uint8_t flow_en){
  uint8_t uart_reg_mcr = master->readRegister(CH9434_UARTx_MCR_ADD+0x10*uartIndex);
  uart_reg_mcr &=~(1<<5);
  if(flow_en) uart_reg_mcr |= (1<<5);
  master->writeRegister(CH9434_UARTx_MCR_ADD+0x10*uartIndex, uart_reg_mcr);
}

/*************************************************
* 描述: 开启中断串口请求
* 输入: 无
* 输出: 无
* 返回: 无
*************************************************/
void SPISerial::enableIRQ(){
  uint8_t uart_reg_mcr = master->readRegister(CH9434_UARTx_MCR_ADD+0x10*uartIndex);
  uart_reg_mcr |= (1<<3);
  master->writeRegister(CH9434_UARTx_MCR_ADD+0x10*uartIndex, uart_reg_mcr);
}

/*************************************************
* 描述: 设置串口RTS、DTR引脚
* 输入:
        rts_val：RTS引脚电平状态
        dtr_val：DTR引脚电平状态
* 输出: 无
* 返回: 无
*************************************************/
void SPISerial::setRTSDTR(uint8_t rts_val,uint8_t dtr_val){
  uint8_t uart_reg_mcr = master->readRegister(CH9434_UARTx_MCR_ADD+0x10*uartIndex);
  if(rts_val) uart_reg_mcr |= (1<<1);
  else        uart_reg_mcr &= ~(1<<1);
  if(dtr_val) uart_reg_mcr |= (1<<0);
  else        uart_reg_mcr &= ~(1<<0);
  master->writeRegister(CH9434_UARTx_MCR_ADD+0x10*uartIndex, uart_reg_mcr);
}

/*************************************************
* 描述: SRC寄存器写操作
* 输入:
      src_val：SRC寄存器值
* 输出: 无
* 返回: 无
*************************************************/
void SPISerial::writeSRC(uint8_t src_val){
  master->writeRegister(CH9434_UARTx_SCR_ADD+0x10*uartIndex, src_val);
}

/*************************************************
* 描述: SRC寄存器读操作
* 输入: 无
* 输出: 无
* 返回: SRC寄存器值
*************************************************/
uint8_t SPISerial::readSRC(){
  uint8_t uart_reg_src = master->readRegister(CH9434_UARTx_SCR_ADD+0x10*uartIndex);
  return uart_reg_src;
}

/*************************************************
* 描述: 串口中断码查询
* 输入: 无
* 输出: 无
* 返回: IIR寄存器值
*************************************************/
uint8_t SPISerial::readIIR(){
  uint8_t uart_reg_iir = master->readRegister(CH9434_UARTx_IIR_ADD+0x10*uartIndex);
  return uart_reg_iir;
}

/*************************************************
* 描述: 串口LSR寄存器读取
* 输入: 无
* 输出: 无
* 返回: LSR寄存器值
*************************************************/
uint8_t SPISerial::readLSR(){
  uint8_t uart_reg_lsr = master->readRegister(CH9434_UARTx_LSR_ADD+0x10*uartIndex);
  return uart_reg_lsr;
}

/*************************************************
* 描述: 串口MSR寄存器读取
* 输入: 无
* 输出: 无
* 返回: MSR寄存器值
*************************************************/
uint8_t SPISerial::readMSR(){
  uint8_t uart_reg_msr = master->readRegister(CH9434_UARTx_MSR_ADD+0x10*uartIndex);
  return uart_reg_msr;
}

/*************************************************
* 描述: 获取串口接收数据长度
* 输入: 无
* 输出: 无
* 返回: 串口接收FIFO的大小
*************************************************/
uint16_t SPISerial::getRxFIFOLength(){
  master->writeRegister(CH9434_FIFO_CTRL_ADD, uartIndex);
  uint8_t uart_fifo_cnt_l = master->readRegister(CH9434_FIFO_CTRL_L_ADD);
  uint8_t uart_fifo_cnt_h = master->readRegister(CH9434_FIFO_CTRL_H_ADD);
  return (uart_fifo_cnt_h<<8) | uart_fifo_cnt_l;
}

/*************************************************
* 描述: 读取串口接收数据
* 输入:
        p_data：数据存储指针
        read_len：读取的数据长度
* 输出: 无
* 返回: 无
*************************************************/
void SPISerial::getRxFIFOData(uint8_t *p_data,uint16_t read_len){
  uint8_t *p_sv_data = p_data;
  for(uint16_t i=0; i<read_len; i++)
    *p_sv_data++ = master->readRegister(CH9434_UARTx_RBR_ADD+0x10*uartIndex);
}

uint8_t SPISerial::getRxFIFOData(){
  return master->readRegister(CH9434_UARTx_RBR_ADD+0x10*uartIndex);;
}

/*************************************************
* 描述: 获取串口发送FIFO长度
* 输入: 无
* 输出: 无
* 返回: 当前串口的接收数据长度
*************************************************/
uint16_t SPISerial::getTxFIFOLength(){
  master->writeRegister(CH9434_FIFO_CTRL_ADD, CH9434_FIFO_CTRL_TR|uartIndex);
  uint8_t uart_fifo_cnt_l = master->readRegister(CH9434_FIFO_CTRL_L_ADD);
  uint8_t uart_fifo_cnt_h = master->readRegister(CH9434_FIFO_CTRL_H_ADD);
  return (uart_fifo_cnt_h<<8) | uart_fifo_cnt_l;
}

/*************************************************
* 描述: 串口填入发送数据
* 输入:
        p_data：发送数据指针
        send_len：发送的数据长度
* 输出: 无
* 返回: 无
*************************************************/
void SPISerial::setTxFIFOData(uint8_t *p_data,uint16_t send_len){
  uint8_t *p_sv_data = p_data;
  for(uint16_t i=0; i<send_len; i++)
    master->writeRegister(CH9434_UARTx_RBR_ADD+0x10*uartIndex, *p_sv_data++);
}

void SPISerial::setTxFIFOData(uint8_t p_data){
  master->writeRegister(CH9434_UARTx_RBR_ADD+0x10*uartIndex, p_data);
}

/*************************************************
* 描述: 串口485切换引脚设置
* 输入:
        tnow_en：串口tnow使能状态
        polar：极性反向设置
* 输出: 无
* 返回: 无
*************************************************/
void SPISerial::setTNOW(uint8_t tnow_en,uint8_t polar){
  uint8_t tnow_ctrl_reg = master->readRegister(CH9434_TNOW_CTRL_CFG_ADD);
  if(tnow_en) tnow_ctrl_reg |= (1<<uartIndex);
  else        tnow_ctrl_reg &=~(1<<uartIndex);
  if(polar) tnow_ctrl_reg |= (1<<(uartIndex+4));
  else      tnow_ctrl_reg &=~(1<<(uartIndex+4));
  master->writeRegister(CH9434_TNOW_CTRL_CFG_ADD,tnow_ctrl_reg);
}