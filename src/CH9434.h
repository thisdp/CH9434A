#pragma once
#include <Arduino.h>
#include <SPI.h>

/* 寄存器操作 */
#define CH9434_REG_OP_WRITE             0x80
#define CH9434_REG_OP_READ              0x00

/* 寄存器 */
#define CH9434_UART0_REG_OFFSET_ADD     0x00
#define CH9434_UART1_REG_OFFSET_ADD     0x10
#define CH9434_UART2_REG_OFFSET_ADD     0x20
#define CH9434_UART3_REG_OFFSET_ADD     0x30
	#define CH9434_UARTx_RBR_ADD          0
	#define CH9434_UARTx_THR_ADD          0
	#define CH9434_UARTx_IER_ADD          1
	#define CH9434_UARTx_IIR_ADD          2
	#define CH9434_UARTx_FCR_ADD          2
	#define CH9434_UARTx_LCR_ADD          3
		#define CH9434_UARTx_BIT_DLAB       (1<<7)
	#define CH9434_UARTx_MCR_ADD          4
	#define CH9434_UARTx_LSR_ADD          5
	#define CH9434_UARTx_MSR_ADD          6
	#define CH9434_UARTx_SCR_ADD          7
	#define CH9434_UARTx_DLL_ADD          0
	#define CH9434_UARTx_DLM_ADD          1

//TNOW UART相关
#define CH9434_TNOW_CTRL_CFG_ADD        0x41
	#define CH9434_BIT_TNOW_OUT_POLAR     0xF0
	#define CH9434_BIT_TNOW_OUT_EN        0x0F
#define CH9434_FIFO_CTRL_ADD            0x42
	#define CH9434_FIFO_CTRL_TR           (1<<4)
	#define CH9434_FIFO_UART_IDX          0x0F
#define CH9434_FIFO_CTRL_L_ADD	        0x43
#define CH9434_FIFO_CTRL_H_ADD	        0x44

//电源时钟设置
#define CH9434_CLK_CTRL_CFG_ADD         0x48
	#define CH9434_CLK_CTRL_MOD           (3<<6)
	#define CH9434_XT_POWER_EN            (1<<5)
	#define CH9434_CLK_DIV_MASK           0x1F
#define CH9434_SLEEP_MOD_CFG_ADD        0x4A

//GPIO相关设置 //GPIO功能使能
#define CH9434_GPIO_FUNC_EN           0x50
//GPIO方向选择
#define CH9434_GPIO_DIR_MOD           0x54
//GPIO上拉设置
#define CH9434_GPIO_PU_MOD            0x58
//GPIO下拉设置
#define CH9434_GPIO_PD_MOD            0x5C
//GPIO引脚电平
#define CH9434_GPIO_PIN_VAL           0x60

/* 串口参数定义 */
/* FIFO Size */
#define CH9434_UART_FIFO_MODE_256       0         //256
#define CH9434_UART_FIFO_MODE_512       1         //512
#define CH9434_UART_FIFO_MODE_1024      2         //1024
#define CH9434_UART_FIFO_MODE_1280      3         //1280

/* TNOW序号 */
#define CH9434_TNOW_POLAR_NORMAL        0                                 //正常输出
#define CH9434_TNOW_POLAR_OPPO          1                                 //反向输出
               
#define CH9434_TNOW_0                   0
#define CH9434_TNOW_1                   1
#define CH9434_TNOW_2                   2
#define CH9434_TNOW_3                   3

/* 低功耗模式 */
#define CH9434_LOWPOWER_INVALID         0
#define CH9434_LOWPOWER_SLEEP           1

/*
一、芯片时钟配置相关说明：
1.外部晶振：32M
2.开启倍频系数：15（固定）
3.芯片设置最高时钟频率不超过40M
4.串口基准时钟为：不开启倍频：32MHz  开启倍频：32MHz*15/分频系数

二、推荐常见波特率的计算方式
1.32M -> 串口基准时钟计算波特率：32M/8/波特率
2.32*15/13=36.923M -> 串口基准时钟计算波特率：36.923M/8/波特率 （如波特率：921600）
*/

class SPISerial;
class CH9434{
public:
  CH9434(SPIClass &spiClass, uint32_t xtFreqArg = 32000000, bool pllEnabledArg = false, uint8_t clkDivNumArg = 15);
  bool begin(uint8_t ss, int8_t sck = -1, int8_t miso = -1, int8_t mosi = -1);
  void sleep();
  void wakeUp();
  /********************************GPIO***********************************************/
  void pinMode(uint8_t gpio_idx,int8_t mode);
  void digitalWrite(uint8_t gpio_idx,uint8_t out_val);
  uint8_t digitalRead(uint8_t gpio_idx);
  uint32_t getSysFreq();

private:
  void setGPIOEnabled(uint8_t gpio_idx, uint8_t enabled); //GPIO使能
  void setGPIODirection(uint8_t gpio_idx, uint8_t dir); //GPIO方向设置
  void setGPIOPullUpEnabled(uint8_t gpio_idx, uint8_t pu); //GPIO上拉设置
  void setGPIOPullDownEnabled(uint8_t gpio_idx, uint8_t pd); //GPIO下拉设置
  bool getGPIOEnabled(uint8_t gpio_idx); //GPIO使能
  bool getGPIODirection(uint8_t gpio_idx); //GPIO方向设置
  bool getGPIOPullUpEnabled(uint8_t gpio_idx); //GPIO上拉设置
  bool getGPIOPullDownEnabled(uint8_t gpio_idx); //GPIO下拉设置
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t data);

  SPIClass *spi;
  uint8_t cs;
  //定义外部晶振频率，记录外部晶振频率，当使用外部晶振时记录，调用CH9434OscXtFreqSet修改
  uint32_t xtFreq;
  //定义当前串口基准时钟频率，根据配置的时钟模式，得出的CH9434串口基准时钟，用于后面计算波特率
  uint32_t sysFreq;  //芯片默认为内部32M
  //睡眠模式，记录CH9434低功耗状态
  uint8_t  lowPower;
  //GPIO的输出电平值 共24个GPIO，CH9434通用GPIO的输出电平值，按位定义，控制引脚电平函数：CH9434GPIOPinOut
  uint32_t gpioValue;
  bool pllEnabled; //倍频功能使能，倍频系数固定15
  uint8_t clkDivNum; //分频系数

  friend class SPISerial;
};

class SPISerial: public Stream{
public:
  SPISerial(CH9434 *masterControllerArg, uint8_t uartIndexArg);
  SPISerial(CH9434 &masterControllerArg, uint8_t uartIndexArg);
  size_t write(uint8_t data);
  size_t write(uint8_t *data, uint16_t length);
  int available();
  int read();
  int peek();
  bool begin(unsigned long baud, uint32_t config = SERIAL_8N1);

  void setParameters(uint32_t bps,uint8_t data_bits,uint8_t veri_bits,uint8_t stop_bits); //串口参数设置
  void setFIFO(uint8_t fifo_en,uint8_t fifo_level); //串口FIFO设置
  void setIRQ(uint8_t modem,uint8_t line,uint8_t tx,uint8_t rx);  //串口中断设置
  void setFlowControlEnabled(uint8_t flow_en);  //流控设置
  void enableIRQ();  //开启中断串口请求
  void setRTSDTR(uint8_t rts_val,uint8_t dtr_val);  //设置串口RTS、DTR引脚
  void writeSRC(uint8_t src_val); //SRC寄存器写操作
  uint8_t readSRC();  //SRC寄存器读操作
  uint8_t readIIR();  //串口中断码查询
  uint8_t readLSR();  //串口LSR寄存器读取
  uint8_t readMSR();  //串口MSR寄存器读取
  uint16_t getRxFIFOLength(); //获取串口接收数据长度
  void getRxFIFOData(uint8_t *p_data,uint16_t read_len);  //读取串口接收数据
  uint8_t getRxFIFOData();  //读取串口接收数据
  uint16_t getTxFIFOLength(); //获取串口发送FIFO长度
  void setTxFIFOData(uint8_t *p_data,uint16_t send_len);  //串口填入发送数据
  void setTxFIFOData(uint8_t p_data);  //串口填入发送数据
  void setTNOW(uint8_t tnow_en,uint8_t polar);  //串口485切换引脚设置
private:
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t data);
  //数据
  uint8_t uartIndex;
  CH9434 *master;
  uint8_t peekedData;
  uint8_t peekedDataAvailable;
};

