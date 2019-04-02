#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "wm8978.h"
#include "board.h"

#define bit0  0x001
#define bit1  0x002
#define bit2  0x004
#define bit3  0x008
#define bit4  0x010
#define bit5  0x020
#define bit6  0x040
#define bit7  0x080
#define bit8  0x100

static const char *WM_TAG = "WM8978_DRIVER";

#define WM_ASSERT(a, format, b, ...) \
    if ((a) != 0) { \
        ESP_LOGE(WM_TAG, format, ##__VA_ARGS__); \
        return b;\
    }

static const i2c_config_t wm_i2c_cfg = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = IIC_DATA,
    .scl_io_num = IIC_CLK,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 100000
};

static int i2c_init()
{
    int res;
    res = i2c_param_config(I2C_NUM_1, &wm_i2c_cfg);
    res |= i2c_driver_install(I2C_NUM_1, wm_i2c_cfg.mode, 0, 0, 0);
    WM_ASSERT(res, "i2c_init error", -1);
    return res;
}


static uint16_t WM8978_REGVAL_TAL[58]=
{
	0X0000,0X0000,0X0000,0X0000,0X0050,0X0000,0X0140,0X0000,
	0X0000,0X0000,0X0000,0X00FF,0X00FF,0X0000,0X0100,0X00FF,
	0X00FF,0X0000,0X012C,0X002C,0X002C,0X002C,0X002C,0X0000,
	0X0032,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,0X0000,
	0X0038,0X000B,0X0032,0X0000,0X0008,0X000C,0X0093,0X00E9,
	0X0000,0X0000,0X0000,0X0000,0X0003,0X0010,0X0010,0X0100,
	0X0100,0X0002,0X0001,0X0001,0X0039,0X0039,0X0039,0X0039,
	0X0001,0X0001
};

static void wm8979_interface()
{
	//WM8978_REGVAL_TAL[WM8978_AUDIO_INTERFACE]|=bit0;//mono left phase
	WM8978_REGVAL_TAL[WM8978_AUDIO_INTERFACE] &=~(bit6|bit5);//16bit
	//WM8978_REGVAL_TAL[WM8978_AUDIO_INTERFACE]|=(bit6|bit5);//32bit
	wm8978_write_reg(WM8978_AUDIO_INTERFACE,WM8978_REGVAL_TAL[WM8978_AUDIO_INTERFACE]);
	WM8978_REGVAL_TAL[WM8978_CLOCKING]|=bit0; //the codec ic is master mode
	WM8978_REGVAL_TAL[WM8978_CLOCKING]|=bit3|bit2;// 100 256/16=16; 16 bit
	WM8978_REGVAL_TAL[WM8978_CLOCKING]&=~(bit7|bit6|bit5);
//	WM8978_REGVAL_TAL[WM8978_CLOCKING]|=bit6;// 010
//	WM8978_REGVAL_TAL[WM8978_CLOCKING]|=bit8;
	WM8978_REGVAL_TAL[WM8978_CLOCKING]&=~bit8;//mclk is the clk source
	wm8978_write_reg(WM8978_CLOCKING,WM8978_REGVAL_TAL[WM8978_CLOCKING]);
	WM8978_REGVAL_TAL[WM8978_CLOCKING]&=~bit8;//mclk is the clk source
}
static void wm8979_pll(uint32_t k,uint8_t n)

{
	WM8978_REGVAL_TAL[WM8978_POWER_MANAGEMENT_1]|=bit5;//enable pll
	wm8978_write_reg(WM8978_POWER_MANAGEMENT_1,WM8978_REGVAL_TAL[WM8978_POWER_MANAGEMENT_1]);
	WM8978_REGVAL_TAL[WM8978_PLL_N]|=bit4;//mclk/2 =20m
	WM8978_REGVAL_TAL[WM8978_PLL_N]&=0x1f0;
	WM8978_REGVAL_TAL[WM8978_PLL_N]|=n;//7
	wm8978_write_reg(WM8978_PLL_N,WM8978_REGVAL_TAL[WM8978_PLL_N]);
	//k=EE009F
	WM8978_REGVAL_TAL[WM8978_PLL_K1]=(k>>18);
	wm8978_write_reg(WM8978_PLL_K1,WM8978_REGVAL_TAL[WM8978_PLL_K1]);
	WM8978_REGVAL_TAL[WM8978_PLL_K2]=(k>>9);
	wm8978_write_reg(WM8978_PLL_K2,WM8978_REGVAL_TAL[WM8978_PLL_K2]);
	WM8978_REGVAL_TAL[WM8978_PLL_K3]=k;
	wm8978_write_reg(WM8978_PLL_K3,WM8978_REGVAL_TAL[WM8978_PLL_K3]);
}
static void wm8979_loopback()
{

	WM8978_REGVAL_TAL[WM8978_COMPANDING_CONTROL]|=bit0; //start loopback
	wm8978_write_reg(WM8978_COMPANDING_CONTROL,WM8978_REGVAL_TAL[WM8978_COMPANDING_CONTROL]);

}

esp_err_t wm8978_deinit(void)
{
    int ret = 0;
	ret = wm8978_write_reg(WM8978_RESET,0x00);
    return ret;
}

/**
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
esp_err_t wm8978_init(audio_hal_codec_config_t *cfg)
{
	int ret = 0;
	i2c_init();
// #ifdef CONFIG_ESP_LYRAT_V4_3_BOARD
//     #include "headphone_detect.h"
//     headphone_detect_init();
// #endif
	ret=wm8978_write_reg(0,0);
	if(ret != 0)
		WM_ASSERT(ret, "wm8978 softreset error", -1);
	// wm8979_pll(0x86c227,0x07);
	wm8979_interface();
	ret |= wm8978_write_reg(1,0X3B);	//R1,MICEN = 1(MIC使能),BIASEN = 1,VMIDSEL[1:0] = 11(5K)
	ret |= wm8978_write_reg(2,0X1B0);	//R2,ROUT1,LOUT1输出使能(耳机可以工作),BOOSTENR,BOOSTENL使能
	ret |= wm8978_write_reg(3,0X6C);	//R3,LOUT2,ROUT2输出使能(喇叭工作),RMIX,LMIX使能
	// ret |= wm8978_write_reg(6,0);		//R6,MCLK由外部提供
	ret |= wm8978_write_reg(43,1<<4);	//R43,INVROUT2反向,驱动喇叭
	ret |= wm8978_write_reg(47,1<<8);	//R47设置,PGABOOSTL,左通道MIC获得20倍增益
	ret |= wm8978_write_reg(48,1<<8);	//R48设置,PGABOOSTR,右通道MIC获得20倍增益
	ret |= wm8978_write_reg(49,1<<1);	//R49,TSDEN,开启过热保护
	ret |= wm8978_write_reg(10,1<<3);	//R10,SOFTMUTE关闭,128x采样,最佳SNR
	ret |= wm8978_write_reg(14,1<<3);	//R14,ADC 128x采样率

	if(ret != 0)
		WM_ASSERT(ret, "i2c_init error", -1);

    WM8978_ADDA_Cfg(1,1);
    WM8978_Input_Cfg(1,0,0);
    WM8978_Output_Cfg(1,0);
    WM8978_MIC_Gain(25);
    WM8978_AUX_Gain(0);
    WM8978_LINEIN_Gain(0);
    WM8978_SPKvol_Set(60);
    WM8978_HPvol_Set(15,15);
    WM8978_EQ_3D_Dir(0);
    WM8978_EQ1_Set(0,24);
    WM8978_EQ2_Set(0,24);
    WM8978_EQ3_Set(0,24);
    WM8978_EQ4_Set(0,24);
    WM8978_EQ5_Set(0,24);

	return ret;
}

int wm8978_config_i2s(audio_hal_codec_mode_t mode, audio_hal_codec_i2s_iface_t *iface)
{
	return 0;
}

/**
 * @param volume: 0 ~ 63
 *
 * @return
 *     - (-1)  Error
 *     - (0)   Success
 */
int wm8978_set_voice_volume(uint8_t volume)
{
	int ret = 0;
	// volume&=0X3F;
	// if(volume==0)volume|=1<<6;
	// ret = wm8978_write_reg(WM8978_LOUT1_HP_CONTROL, volume);//for earphone
	// ret |= wm8978_write_reg(WM8978_ROUT1_HP_CONTROL, volume|(1<<8));//for earphone
	// ret |= wm8978_write_reg(WM8978_LOUT2_SPK_CONTROL, volume);
	// ret |= wm8978_write_reg(WM8978_ROUT2_SPK_CONTROL, volume|(1<<8));
	return ret;
}

/**
 *
 * @return
 *           volume
 */
int wm8978_get_voice_volume(int *volume)
{
    int ret = 0;
    // uint8_t _vol = 0;
	// _vol = WM8978_REGVAL_TAL[WM8978_LOUT1_HP_CONTROL]&0x3F;
	// *volume = _vol;
    return ret;
}

int wm8978_ctrl_state(audio_hal_codec_mode_t mode, audio_hal_ctrl_t ctrl_state)
{
    int res = 0;
    return res;
}

// static int wm_write_reg(uint8_t slave_add, uint8_t reg_add, uint8_t data)
// {
//     int res = 0;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     res |= i2c_master_start(cmd);
//     res |= i2c_master_write_byte(cmd, slave_add, 1 /*ACK_CHECK_EN*/);
//     res |= i2c_master_write_byte(cmd, reg_add, 1 /*ACK_CHECK_EN*/);
//     res |= i2c_master_write_byte(cmd, data, 1 /*ACK_CHECK_EN*/);
//     res |= i2c_master_stop(cmd);
//     res |= i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);
//     ES_ASSERT(res, "es_write_reg error", -1);
//     return res;
// }

// static int wm_read_reg(uint8_t reg_add, uint8_t *pData)
// {
//     uint8_t data;
//     int res = 0;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();

//     res |= i2c_master_start(cmd);
//     res |= i2c_master_write_byte(cmd, ES8388_ADDR, 1 /*ACK_CHECK_EN*/);
//     res |= i2c_master_write_byte(cmd, reg_add, 1 /*ACK_CHECK_EN*/);
//     res |= i2c_master_stop(cmd);
//     res |= i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);

//     cmd = i2c_cmd_link_create();
//     res |= i2c_master_start(cmd);
//     res |= i2c_master_write_byte(cmd, ES8388_ADDR | 0x01, 1 /*ACK_CHECK_EN*/);
//     res |= i2c_master_read_byte(cmd, &data, 0x01/*NACK_VAL*/);
//     res |= i2c_master_stop(cmd);
//     res |= i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);

//     ES_ASSERT(res, "es_read_reg error", -1);
//     *pData = data;
//     return res;
// }

/**
 * @brief write data to register of wm8978
 *
 * @param reg:   register address
 * @param val:   value to write
 *
 * @return
 *     - (-1) Parameter error
 *     - (0)   Success
 */
int wm8978_write_reg(uint8_t reg_addr,uint16_t data)
{
	int res = 0;
	uint8_t buf[2];
	buf[0]=((data&0x0100)>>8)|(reg_addr<<1);
	buf[1]=(uint8_t)(data&0xff);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    res |= i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (WM8978_ADDR<<1) | I2C_MASTER_WRITE, 1);
    i2c_master_write_byte(cmd, buf[0], 1);
    i2c_master_write_byte(cmd, buf[1], 1);
    i2c_master_stop(cmd);
    res |= i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
	WM8978_REGVAL_TAL[reg_addr]=data;
    WM_ASSERT(res, "wm_write_reg error", -1);
	return res;
}

// WM8978 read register
// Reads the value  of the local register buffer zone
// reg: Register Address
// Return Value: Register value
uint16_t wm8978_read_reg(uint8_t reg_addr)
// uint16_t wm8978_read_reg(uint8_t reg_addr, uint8_t* pData)

{
	// int ret = 0;
	// // uint16_t data;
	// uint8_t data;


	// // i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    // // ret = i2c_master_start(cmd);
    // // ret |= i2c_master_write_byte(cmd, (WM8978_ADDR<<1) | I2C_MASTER_READ, 0);
    // // ret |= i2c_master_write_byte(cmd, reg_addr, 0);
    // // ret |= i2c_master_read_byte(cmd, &data, 0);
    // // ret |= i2c_master_stop(cmd);
    // // res |= i2c_master_cmd_begin(I2C_NUM_1, cmd, 1000 / portTICK_RATE_MS);
    // // i2c_cmd_link_delete(cmd);

	// // WM_ASSERT(ret, "wm8978_read_reg error", -1);
	// data = WM8978_REGVAL_TAL[reg_addr]&0x3F;

	// *pData = data;


	// return ret;

	return WM8978_REGVAL_TAL[reg_addr];
}

// static int wm_read_reg(uint8_t reg_add, uint8_t *pData)
// {
//     uint8_t data;
//     int res = 0;
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();

//     res |= i2c_master_start(cmd);
//     res |= i2c_master_write_byte(cmd, ES8388_ADDR, 1 /*ACK_CHECK_EN*/);
//     res |= i2c_master_write_byte(cmd, reg_add, 1 /*ACK_CHECK_EN*/);
//     res |= i2c_master_stop(cmd);
//     res |= i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);

//     cmd = i2c_cmd_link_create();
//     res |= i2c_master_start(cmd);
//     res |= i2c_master_write_byte(cmd, ES8388_ADDR | 0x01, 1 /*ACK_CHECK_EN*/);
//     res |= i2c_master_read_byte(cmd, &data, 0x01/*NACK_VAL*/);
//     res |= i2c_master_stop(cmd);
//     res |= i2c_master_cmd_begin(0, cmd, 1000 / portTICK_RATE_MS);
//     i2c_cmd_link_delete(cmd);

//     ES_ASSERT(res, "es_read_reg error", -1);
//     *pData = data;
//     return res;
// }



//WM8978 DAC/ADC配置
//adcen:adc使能(1)/关闭(0)
//dacen:dac使能(1)/关闭(0)

void WM8978_ADDA_Cfg(uint8_t dacen,uint8_t adcen)
{
	uint16_t regval;
	regval=wm8978_read_reg(3);	//??ȡR3
	if(dacen)regval|=3<<0;		//R3?????λ??Ϊ1,???DACR&DACL
	else regval&=~(3<<0);		//R3?????λ????ر?ACR&DACL.
	wm8978_write_reg(3,regval);	//??R3
	regval=wm8978_read_reg(2);	//??ȡR2
	if(adcen)regval|=3<<0;		//R2?????λ??Ϊ1,???ADCR&ADCL
	else regval&=~(3<<0);		//R2?????λ????ر?DCR&ADCL.
	wm8978_write_reg(2,regval);	//??R2
}
//WM8978 输入通道配置
//micen:MIC开启(1)/关闭(0)
//lineinen:Line In开启(1)/关闭(0)
//auxen:aux开启(1)/关闭(0)
void WM8978_Input_Cfg(uint8_t micen,uint8_t lineinen,uint8_t auxen)
{
	uint16_t regval;
	regval=wm8978_read_reg(2);	//??ȡR2
	if(micen)regval|=3<<2;		//???INPPGAENR,INPPGAENL(MIC??GA?Ŵ?
	else regval&=~(3<<2);		//?ر?NPPGAENR,INPPGAENL.
 	wm8978_write_reg(2,regval);	//??R2

	regval=wm8978_read_reg(44);	//??ȡR44
	if(micen)regval|=3<<4|3<<0;	//???LIN2INPPGA,LIP2INPGA,RIN2INPPGA,RIP2INPGA.
	else regval&=~(3<<4|3<<0);	//?ر?IN2INPPGA,LIP2INPGA,RIN2INPPGA,RIP2INPGA.
	wm8978_write_reg(44,regval);//??R44

	if(lineinen)WM8978_LINEIN_Gain(5);//LINE IN 0dB??
	else WM8978_LINEIN_Gain(0);	//?ر?INE IN
	if(auxen)WM8978_AUX_Gain(7);//AUX 6dB??
	else WM8978_AUX_Gain(0);	//?ر?UX??
}
//WM8978 输出配置
//dacen:DAC输出(放音)开启(1)/关闭(0)
//bpsen:Bypass输出(录音,包括MIC,LINE IN,AUX等)开启(1)/关闭(0)
void WM8978_Output_Cfg(uint8_t dacen,uint8_t bpsen)
{
	uint16_t regval=0;
	if(dacen)regval|=1<<0;	//DAC???ʹ?
	if(bpsen)
	{
		regval|=1<<1;		//BYPASSʹ?
		regval|=5<<2;		//0dB??
	}
	wm8978_write_reg(50,regval);//R50??
	wm8978_write_reg(51,regval);//R51??
}
//WM8978 MIC增益设置(不包括BOOST的20dB,MIC-->ADC输入部分的增益)
//gain:0~63,对应-12dB~35.25dB,0.75dB/Step
void WM8978_MIC_Gain(uint8_t gain)
{
	gain&=0X3F;
	wm8978_write_reg(45,gain);		//R45,?ͨ??PGA??
	wm8978_write_reg(46,gain|1<<8);	//R46,?ͨ??PGA??
}
//WM8978 L2/R2(也就是Line In)增益设置(L2/R2-->ADC输入部分的增益)
//gain:0~7,0表示通道禁止,1~7,对应-12dB~6dB,3dB/Step
void WM8978_LINEIN_Gain(uint8_t gain)
{
	uint16_t regval;
	gain&=0X07;
	regval=wm8978_read_reg(47);	//??ȡR47
	regval&=~(7<<4);			//???ԭ??????
 	wm8978_write_reg(47,regval|gain<<4);//??R47
	regval=wm8978_read_reg(48);	//??ȡR48
	regval&=~(7<<4);			//???ԭ??????
 	wm8978_write_reg(48,regval|gain<<4);//??R48
}

//WM8978 AUXR,AUXL(PWM音频部分)增益设置(AUXR/L-->ADC输入部分的增益)
//gain:0~7,0表示通道禁止,1~7,对应-12dB~6dB,3dB/Step
void WM8978_AUX_Gain(uint8_t gain)
{
	uint16_t regval;
	gain&=0X07;
	regval=wm8978_read_reg(47);	//??ȡR47
	regval&=~(7<<0);			//???ԭ??????
 	wm8978_write_reg(47,regval|gain<<0);//??R47
	regval=wm8978_read_reg(48);	//??ȡR48
	regval&=~(7<<0);			//???ԭ??????
 	wm8978_write_reg(48,regval|gain<<0);//??R48
}
//设置I2S工作模式
//fmt:0,LSB(右对齐);1,MSB(左对齐);2,飞利浦标准I2S;3,PCM/DSP;
//len:0,16位;1,20位;2,24位;3,32位;
void WM8978_I2S_Cfg(uint8_t fmt,uint8_t len)
{
	fmt&=0X03;
	len&=0X03;//?????Χ
	wm8978_write_reg(4,(fmt<<3)|(len<<5));	//R4,WM8978???ģʽ??
}


//设置耳机左右声道音量
//voll:左声道音量(0~63)
//volr:右声道音量(0~63)
void WM8978_HPvol_Set(uint8_t voll,uint8_t volr)
{
	voll&=0X3F;
	volr&=0X3F;//?????Χ
	if(voll==0)voll|=1<<6;//???Ϊ0ʱ,ֱ??ute
	if(volr==0)volr|=1<<6;//???Ϊ0ʱ,ֱ??ute
	wm8978_write_reg(52,voll);			//R52,?????????????
	wm8978_write_reg(53,volr|(1<<8));	//R53,?????????????,ͬ?????(HPVU=1)
}
//设置喇叭音量
//voll:左声道音量(0~63)
void WM8978_SPKvol_Set(uint8_t volx)
{
	volx&=0X3F;//?????Χ
	if(volx==0)volx|=1<<6;//???Ϊ0ʱ,ֱ??ute
 	wm8978_write_reg(54,volx);			//R54,?????????????
	wm8978_write_reg(55,volx|(1<<8));	//R55,?????????????,ͬ?????(SPKVU=1)
}
//设置3D环绕声
//depth:0~15(3D强度,0最弱,15最强)
void WM8978_3D_Set(uint8_t depth)
{
	depth&=0XF;//?????Χ
 	wm8978_write_reg(41,depth);	//R41,3D?????
}
//设置EQ/3D作用方向
//dir:0,在ADC起作用
//    1,在DAC起作用(默认)
void WM8978_EQ_3D_Dir(uint8_t dir)
{
	uint16_t regval;
	regval=wm8978_read_reg(0X12);
	if(dir)regval|=1<<8;
	else regval&=~(1<<8);
 	wm8978_write_reg(18,regval);//R18,EQ1?ĵ?λ???Q/3D???
}

//设置EQ1
//cfreq:截止频率,0~3,分别对应:80/105/135/175Hz
//gain:增益,0~24,对应-12~+12dB
void WM8978_EQ1_Set(uint8_t cfreq,uint8_t gain)
{
	uint16_t regval;
	cfreq&=0X3;//?????Χ
	if(gain>24)gain=24;
	gain=24-gain;
	regval=wm8978_read_reg(18);
	regval&=0X100;
	regval|=cfreq<<5;	//?????Ƶ?
	regval|=gain;		//????
 	wm8978_write_reg(18,regval);//R18,EQ1??
}
//设置EQ2
//cfreq:中心频率,0~3,分别对应:230/300/385/500Hz
//gain:增益,0~24,对应-12~+12dB
void WM8978_EQ2_Set(uint8_t cfreq,uint8_t gain)
{
	uint16_t regval=0;
	cfreq&=0X3;//?????Χ
	if(gain>24)gain=24;
	gain=24-gain;
	regval|=cfreq<<5;	//?????Ƶ?
	regval|=gain;		//????
 	wm8978_write_reg(19,regval);//R19,EQ2??
}
//设置EQ3
//cfreq:中心频率,0~3,分别对应:650/850/1100/1400Hz
//gain:增益,0~24,对应-12~+12dB
void WM8978_EQ3_Set(uint8_t cfreq,uint8_t gain)
{
	uint16_t regval=0;
	cfreq&=0X3;//?????Χ
	if(gain>24)gain=24;
	gain=24-gain;
	regval|=cfreq<<5;	//?????Ƶ?
	regval|=gain;		//????
 	wm8978_write_reg(20,regval);//R20,EQ3??
}
//设置EQ4
//cfreq:中心频率,0~3,分别对应:1800/2400/3200/4100Hz
//gain:增益,0~24,对应-12~+12dB
void WM8978_EQ4_Set(uint8_t cfreq,uint8_t gain)
{
	uint16_t regval=0;
	cfreq&=0X3;//?????Χ
	if(gain>24)gain=24;
	gain=24-gain;
	regval|=cfreq<<5;	//?????Ƶ?
	regval|=gain;		//????
 	wm8978_write_reg(21,regval);//R21,EQ4??
}
//设置EQ5
//cfreq:中心频率,0~3,分别对应:5300/6900/9000/11700Hz
//gain:增益,0~24,对应-12~+12dB
void WM8978_EQ5_Set(uint8_t cfreq,uint8_t gain)
{
	uint16_t regval=0;
	cfreq&=0X3;//?????Χ
	if(gain>24)gain=24;
	gain=24-gain;
	regval|=cfreq<<5;	//?????Ƶ?
	regval|=gain;		//????
 	wm8978_write_reg(22,regval);//R22,EQ5??
}
