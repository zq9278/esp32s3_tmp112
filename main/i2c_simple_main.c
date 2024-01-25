
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

#include "driver/ledc.h"

#include "driver/uart.h"
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (4) // 示例GPIO
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TEST_DUTY         (8192) // 占空比
#define LEDC_TEST_FADE_TIME    (3000)
//void pwm_control_task(void *pvParameter);
void set_pwm_duty(uint8_t duty_value);
void ntc_pwm_init(); 

#define BUF_SIZE (1024) 

typedef struct 
{
    double Kp;        // 比例系数
    double Ki;        // 积分系数
    double Kd;        // 微分系数
    double integral;  // 积分项
    double pre_error; // 上一次误差
    float maxIntegral;
    float maxOutput;
} PID_Controller;

uint8_t pwm_duty;

 PID_Controller pid;
void PID_Init(PID_Controller *pid, double Kp, double Ki, double Kd);
void pid_control_task(void *pvParameter);

#define Limit(x,min,max) (x)=(((x)<=(min))?(min):(((x)>=(max))?(max):(x)))




void ntc_pwm_init()
{
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // 分辨率
        .freq_hz = 5000,                      // 频率
        .speed_mode = LEDC_HS_MODE,           // 速度模式
        .timer_num = LEDC_HS_TIMER            // 定时器
    };
    // 设置定时器配置
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_HS_CH0_CHANNEL,
        .duty = 0,
        .gpio_num = LEDC_HS_CH0_GPIO,
        .speed_mode = LEDC_HS_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_HS_TIMER};
    // 设置通道配置
    ledc_channel_config(&ledc_channel);
}
// 设置PWM占空比的函数
void set_pwm_duty(uint8_t duty_value)
{
    // 将百分比转换为占空比
    // LEDC_TEST_DUTY通常是根据PWM分辨率来定义的最大值
    uint32_t duty = (LEDC_TEST_DUTY * duty_value) / 100;

    // 设置PWM占空比
    // LEDC_HS_MODE表示高速模式，LEDC_HS_CH0_CHANNEL是要控制的通道
    ledc_set_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL, duty);

    // 更新PWM占空比以应用新的设置
    ledc_update_duty(LEDC_HS_MODE, LEDC_HS_CH0_CHANNEL);
}
void PID_Init(PID_Controller *pid, double Kp, double Ki, double Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0;
    pid->pre_error = 0.0;
    pid->maxIntegral = 100.0;
    pid->maxOutput = 100.0;
}

// 计算PID控制器的输出
double PID_Compute(PID_Controller *pid, double setpoint, double measured)
{
    double error = setpoint - measured;                                               // 计算误差
    pid->integral += error;                                                           // 积分项
    Limit(pid->integral,-pid->maxIntegral,pid->maxIntegral);
    double derivative = error - pid->pre_error;                                       // 微分项
    pid->pre_error = error;                                                           // 更新误差
    double output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative; // 计算输出
    Limit(output,0,pid->maxOutput);
    return output;
}

static const char *TAG = "i2c-simple-example";

#define I2C_MASTER_SCL_IO           0      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           1      /*!< GPIO number used for I2C master data  */

#define I2C_MASTER_SCL_IO2           2      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO2           3      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_NUM1              1                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_FREQ_HZ2          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000


#define TMP114_SENSOR_ADDR   0x49        // TMP114的I2C地址（已更正）


static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, TMP114_SENSOR_ADDR, &reg_addr, 1, data, len,100);
}
static esp_err_t tmp114_read_temperature(uint8_t reg_addr, uint8_t *data, size_t len)
{
    //return i2c_master_write_read_device(I2C_MASTER_NUM1, TMP114_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return i2c_master_write_read_device(I2C_MASTER_NUM1, TMP114_SENSOR_ADDR, &reg_addr, 1, data, len, 100);
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

static esp_err_t i2c_master2_init(void)
{
    int i2c_master_port2 = I2C_MASTER_NUM1;

    i2c_config_t conf2 = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO2,
        .scl_io_num = I2C_MASTER_SCL_IO2,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ2,
    };

    i2c_param_config(i2c_master_port2, &conf2);
    return i2c_driver_install(i2c_master_port2, conf2.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void uart_init()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(UART_NUM_0, &uart_config);                             // 初始化UART参数配置
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0); // 安装UART驱动
    uart_set_pin(
        UART_NUM_0, // UART 编号
        10,        // TX GPIO
        11,        // RX GPIO
        -1,       // RTS GPIO
        -1        // CTS GPIO
    );
}

void app_main(void)
{uart_init(); // 初始化UART
    PID_Init(&pid, 100.0, 0.05, 0.0); // 初始化PID参数
    ntc_pwm_init();
     uint8_t data[2];
     uint8_t data2[2];
     int16_t TmpData;
     int16_t TmpData4;
     double TmpData2;
     double TmpData_4;
     double val1, val2, val3,val4;
      char data_rx[200]; // 定义数据缓冲区
     ESP_ERROR_CHECK(i2c_master_init());
     ESP_ERROR_CHECK(i2c_master2_init());
     ESP_LOGI(TAG, "I2C initialized successfully");
     while(1){
ESP_ERROR_CHECK(mpu9250_register_read(0x00, data, 2));
ESP_ERROR_CHECK(tmp114_read_temperature(0x00, data2, 2));
    
     
	TmpData=(data[0]<<8) | data[1];
	TmpData = TmpData >> 4;
	TmpData2 = TmpData*0.0625;

    TmpData4=(data2[0]<<8) | data2[1];
	TmpData4 = TmpData4 >> 4;
	TmpData_4 = TmpData4*0.0625;



 int len = uart_read_bytes(UART_NUM_0, data_rx, 200, 100 / portTICK_PERIOD_MS);
        if (len > 0) {
            data_rx[len] = '\0'; // 确保字符串正确终止
            // 尝试从接收到的数据中提取三个浮点数
            int result = sscanf((char *)data_rx,"%lf,%lf,%lf,%lf", &val1, &val2, &val3,&val4);
            if (result == 4) {
                PID_Init(&pid, val2, val3, val4); // 初始化 PID 参数
            }
        }





pwm_duty = PID_Compute(&pid, val1, TmpData2); // 计算 PID
     set_pwm_duty(pwm_duty);
     printf("温度:%f,%f,%f,%f\n", TmpData2,TmpData_4,val1,(double)pwm_duty);

     }
     
    //  ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    //  ESP_LOGI(TAG, "I2C de-initialized successfully");
}
