#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/vmalloc.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/major.h>
#include <linux/kdev_t.h>
#include <linux/of_gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/usb.h>
#include <linux/power_supply.h>
#include "lt9211c.h"

#define	 MIPIRX_INPUT_SEL		   MIPI_DSI			//MIPI_DSI/MIPI_CSI
#define	 MIPIRX_PORT_SEL			PORTA			   //PORTA/PORTB/DOU_PORT
#define	 MIPIRX_LANE_NUM			MIPIRX_4LANE		//MIPIRX_4LANE/MIPIRX_3LANE/MIPIRX_2LANE/MIPIRX_1LANE
#define	 MIPIRX_PORT_SWAP		   DISABLED			 //ENABLED/DISABLED
#define	 LVDSTX_PORT_SEL		 DOU_PORT		//PORTA/PORTB/DOU_PORT
#define	 LVDSTX_PORT_SWAP		DISABLED		 //ENABLED/DISABLED
#define	 LVDSTX_PORT_COPY		DISABLED		//ENABLED/DISABLED
#define	 LVDSTX_LANENUM		  FOUR_LANE	   //FOUR_LANE/FIVE_LANE
#define	 LVDSTX_MODE			 SYNC_MODE	   //SYNC_MODE/DE_MODE
#define	 LVDSTX_DATAFORMAT	   JEIDA			//VESA/JEIDA
#define	 LVDSTX_COLORSPACE	   RGB			 //RGB/YUV422
#define	 LVDSTX_COLORDEPTH	   DEPTH_8BIT	  //DEPTH_6BIT/DEPTH_8BIT/DEPTH_10BIT
#define	 LVDSTX_SYNC_INTER_MODE  DISABLED		 //ENABLED/DISABLED
#define	 LVDSTX_VIDEO_FORMAT	 P_FORMAT		//P_FORMAT/I_FORMAT
#define	 LVDSTX_SYNC_CODE_SEND   NON_REPECTIVE	   ///NON_REPECTIVE/REPECTIVE
#define	FAIL 0
#define	SUCCESS 1
#define lt9211_ADDR_LENGTH	  1
#define I2C_MAX_TRANSFER_SIZE   255
#define RETRY_MAX_TIMES		 3
#define lt9211_I2C_NAME	  "lt9211"
#define lt9211_DRIVER_VERSION  "1.0.0"
#define LT9211_READ_ID 1

static struct lt9211_data *pdata = NULL;
StructPcrPara g_stPcrPara;
static int DEBUG = 0;
#define PRINT_DEG(args...) \
	do { \
		if (DEBUG) { \
			printk(args); \
		} \
	} while (0)

/*******************************************************
	Function:
	Write data to the i2c slave device.
	Input:
	client: i2c device.
	buf[0]: write start address.
	buf[1~len-1]: data buffer
	len: lt9211_ADDR_LENGTH + write bytes count
	Output:
	numbers of i2c_msgs to transfer:
		0: succeed, otherwise: failed
 *********************************************************/
int lt9211_i2c_write(struct i2c_client *client, unsigned char *buf, int len)
{
	unsigned int pos = 0, transfer_length = 0;
	unsigned char address = buf[0];
	unsigned char put_buf[64];
	int retry, ret = 0;
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags = !I2C_M_RD,
	};
	if(likely(len < sizeof(put_buf))) {
		/* code optimize,use stack memory*/
		msg.buf = &put_buf[0];
	} else {
		msg.buf = kmalloc(len > I2C_MAX_TRANSFER_SIZE
						  ? I2C_MAX_TRANSFER_SIZE : len, GFP_KERNEL);
		if(!msg.buf)
			return -ENOMEM;
	}
	len -= lt9211_ADDR_LENGTH;
	while(pos != len) {
		if(unlikely(len - pos > I2C_MAX_TRANSFER_SIZE - lt9211_ADDR_LENGTH))
			transfer_length = I2C_MAX_TRANSFER_SIZE - lt9211_ADDR_LENGTH;
		else
			transfer_length = len - pos;
		msg.buf[0] = address;
		msg.len = transfer_length + lt9211_ADDR_LENGTH;
		memcpy(&msg.buf[lt9211_ADDR_LENGTH], &buf[lt9211_ADDR_LENGTH + pos], transfer_length);
		for(retry = 0; retry < RETRY_MAX_TIMES; retry++) {
			if(likely(i2c_transfer(client->adapter, &msg, 1) == 1)) {
				pos += transfer_length;
				address += transfer_length;
				break;
			}
			dev_info(&client->dev, "I2C write retry[%d]\n", retry + 1);
			usleep_range(1000,2000);
		}
		if(unlikely(retry == RETRY_MAX_TIMES)) {
			dev_err(&client->dev,
					"I2c write failed,dev:%02x,reg:%02x,size:%u\n",
					client->addr, address, len);
			ret = -EAGAIN;
			goto write_exit;
		}
	}
write_exit:
	if(len + lt9211_ADDR_LENGTH >= sizeof(put_buf))
		kfree(msg.buf);
	return ret;
}

/*******************************************************
	Function:
	Read data from the i2c slave device.
	Input:
	client: i2c device.
	buf[0]: read start address.
	buf[1~len-1]: data buffer
	len: lt9211_ADDR_LENGTH + read bytes count
	Output:
	numbers of i2c_msgs to transfer:
		0: succeed, otherwise: failed
 *********************************************************/
int lt9211_i2c_read(struct i2c_client *client, unsigned char *buf, int len)
{
	unsigned int transfer_length = 0;
	unsigned int pos = 0;
	unsigned char address = buf[0];
	unsigned char get_buf[64], addr_buf[2];
	int retry, ret = 0;
	struct i2c_msg msgs[] = {
		{
			.addr = client->addr,
			.flags = !I2C_M_RD,
			.buf = &addr_buf[0],
			.len = lt9211_ADDR_LENGTH,
		}, {
			.addr = client->addr,
			.flags = I2C_M_RD,
		}
	};
	len -= lt9211_ADDR_LENGTH;
	if(likely(len < sizeof(get_buf))) {
		/* code optimize, use stack memory */
		msgs[1].buf = &get_buf[0];
	} else {
		msgs[1].buf = kzalloc(len > I2C_MAX_TRANSFER_SIZE
							  ? I2C_MAX_TRANSFER_SIZE : len, GFP_KERNEL);
		if(!msgs[1].buf)
			return -ENOMEM;
	}
	while(pos != len) {
		if(unlikely(len - pos > I2C_MAX_TRANSFER_SIZE))
			transfer_length = I2C_MAX_TRANSFER_SIZE;
		else
			transfer_length = len - pos;
		msgs[0].buf[0] = address;
		msgs[1].len = transfer_length;
		for(retry = 0; retry < RETRY_MAX_TIMES; retry++) {
			if(likely(i2c_transfer(client->adapter, msgs, 2) == 2)) {
				memcpy(&buf[lt9211_ADDR_LENGTH + pos], msgs[1].buf, transfer_length);
				pos += transfer_length;
				address += transfer_length;
				break;
			}
			dev_info(&client->dev, "I2c read retry[%d]:0x%x\n",
					 retry + 1, address);
			usleep_range(1000,2000);
		}
		if(unlikely(retry == RETRY_MAX_TIMES)) {
			dev_err(&client->dev,
					"I2c read failed,dev:%02x,reg:%02x,size:%u\n",
					client->addr, address, len);
			ret = -EAGAIN;
			goto read_exit;
		}
	}
read_exit:
	if(len >= sizeof(get_buf))
		kfree(msgs[1].buf);
	return ret;
}

int lt9211_write(struct i2c_client *client, unsigned char addr, unsigned char data)
{
	unsigned char buf[2] = {addr, data};
	int ret = -1;
	ret = lt9211_i2c_write(client, buf, 2);
	if(ret < 0) {
		printk("lt9211_i2c_write error\n");
	}
	return ret;
}

unsigned char lt9211_read(struct i2c_client *client, unsigned char addr)
{
	unsigned char buf[2] = {addr};
	int ret = -1;
	ret = lt9211_i2c_read(client, buf, 2);
	if(ret == 0) {
		return buf[1];
	} else {
		return 0;
	}
}

//0xff 0x81 register bank
//0x00 0x01 0x02 ID寄存器
#ifdef LT9211_READ_ID
static int lt9211_read_ID(void )
{
	unsigned char ID[3];

	lt9211_write(pdata->client, 0xff, 0x81);
	ID[0] =lt9211_read(pdata->client, 0x00);
	ID[1] =lt9211_read(pdata->client, 0x01);
	ID[2] =lt9211_read(pdata->client, 0x02);
	PRINT_DEG("lt9211 read ID[0]=0x%2x ID[1]=0x%2x ID[2]=0x%2x\n", ID[0], ID[1], ID[2]);
	return 0;
}
#endif

//关闭lvds
void Drv_LVDSTxPhy_PowerOff(void)
{
	lt9211_write(pdata->client,0xff,0x82);
	lt9211_write(pdata->client,0x36,0x00); //lvds enable
	lt9211_write(pdata->client,0x37,0x00);
}

//打开lvds
void Drv_LvdsTxPhy_Poweron(void)
{
#if LVDSTX_PORT_SEL  == PORTA
	lt9211_write(pdata->client,0xff,0x82);
	lt9211_write(pdata->client,0x36,0x01); //lvds enable
	lt9211_write(pdata->client,0x37,0x40);
	PRINT_DEG("LVDS Output PortA\n");
#if LVDSTX_LANENUM == FIVE_LANE
	lt9211_write(pdata->client,0x36,0x03);
#endif

#elif LVDSTX_PORT_SEL  == PORTB
	lt9211_write(pdata->client,0xff,0x82);
	lt9211_write(pdata->client,0x36,0x02); //lvds enable
	lt9211_write(pdata->client,0x37,0x04);
	PRINT_DEG("LVDS Output PortB\n");
#elif LVDSTX_PORT_SEL  == DOU_PORT
	lt9211_write(pdata->client,0xff,0x82);
	lt9211_write(pdata->client,0x36,0x03); //lvds enable
	lt9211_write(pdata->client,0x37,0x44); //port rterm enable
	PRINT_DEG("LVDS Output PortA&B\n");
#endif

	lt9211_write(pdata->client,0x38,0x14);
	lt9211_write(pdata->client,0x39,0x31);
	lt9211_write(pdata->client,0x3a,0xc8);
	lt9211_write(pdata->client,0x3b,0x00);
	lt9211_write(pdata->client,0x3c,0x0f);
	lt9211_write(pdata->client,0x46,0x40);
	lt9211_write(pdata->client,0x47,0x40);
	lt9211_write(pdata->client,0x48,0x40);
	lt9211_write(pdata->client,0x49,0x40);
	lt9211_write(pdata->client,0x4a,0x40);
	lt9211_write(pdata->client,0x4b,0x40);
	lt9211_write(pdata->client,0x4c,0x40);
	lt9211_write(pdata->client,0x4d,0x40);
	lt9211_write(pdata->client,0x4e,0x40);
	lt9211_write(pdata->client,0x4f,0x40);
	lt9211_write(pdata->client,0x50,0x40);
	lt9211_write(pdata->client,0x51,0x40);
	lt9211_write(pdata->client,0xff,0x81);
	lt9211_write(pdata->client,0x03,0xbf); //mltx reset
	lt9211_write(pdata->client,0x03,0xff); //mltx release
}

void Drv_SystemTxSram_Sel(void)
{
	//[7:6]2'b00: TX Sram sel MIPITX; others sel LVDSTX
	lt9211_write(pdata->client,0xff,0x85);
	lt9211_write(pdata->client,0x30,(lt9211_read(pdata->client,0x30) & 0x3f));
	lt9211_write(pdata->client,0x30,(lt9211_read(pdata->client,0x30) | BIT6_1));
}

unsigned int Drv_System_FmClkGet(IN unsigned char ucSrc)
{
	unsigned int ulRtn = 0;

	lt9211_write(pdata->client,0xff,0x86);
	lt9211_write(pdata->client,0X90,ucSrc);
	usleep_range(500, 5000);//2ms
	lt9211_write(pdata->client,0x90,(ucSrc | BIT7_1));
	ulRtn = (lt9211_read(pdata->client,0x98) & 0x0f);
	ulRtn = (ulRtn << 8) + lt9211_read(pdata->client,0x99);
	ulRtn = (ulRtn << 8) + lt9211_read(pdata->client,0x9a);
	lt9211_write(pdata->client,0x90,(lt9211_read(pdata->client,0x90) & BIT7_0));
	return ulRtn;
}

void Mod_LvdsTxPll_RefPixClk_Get(void)
{
	/*mipi to lvds use desscpll pix clk as txpll ref clk*/
	g_stRxVidTiming.ulPclk_Khz = Drv_System_FmClkGet(AD_DESSCPLL_PIX_CLK);
}

void Drv_LvdsTxPll_RefPixClk_Set(void)
{
	lt9211_write(pdata->client,0xff,0x82);
	/*mipi to lvds use desscpll pix clk as txpll ref clk*/
	lt9211_write(pdata->client,0x30,0x00); //[7]0:txpll normal work; txpll ref clk sel ad desscpll fast pix clk
}

void Drv_LvdsTxPll_Config(void)
{
	unsigned char ucPreDiv = 0;
	unsigned char ucSericlkDiv = 0;
	unsigned char ucDivSet = 0;
	unsigned int ucPixClkDiv = 0;
	unsigned int ulLvdsTXPhyClk = 0;

	/* txphyclk = vco clk * ucSericlkDiv */
	//#if (LVDSTX_PORT_SEL == DOU_PORT)
	ulLvdsTXPhyClk = (unsigned int)((g_stRxVidTiming.ulPclk_Khz >> 1) * 7); //2 port: byte clk = pix clk / 2;
	lt9211_write(pdata->client,0xff,0x85);
	lt9211_write(pdata->client,0x6f,(lt9211_read(pdata->client,0x6f) | BIT0_1)); //htotal -> 2n

	/*txpll prediv sel*/
	lt9211_write(pdata->client,0xff,0x82);
	if (g_stRxVidTiming.ulPclk_Khz < 20000) {
		lt9211_write(pdata->client,0x31,0x28); //[2:0]3'b000: pre div set div1
		ucPreDiv = 1;
	} else if (g_stRxVidTiming.ulPclk_Khz >= 20000 && g_stRxVidTiming.ulPclk_Khz < 40000) {
		lt9211_write(pdata->client,0x31,0x28); //[2:0]3'b000: pre div set div1
		ucPreDiv = 1;
	} else if (g_stRxVidTiming.ulPclk_Khz >= 40000 && g_stRxVidTiming.ulPclk_Khz < 80000) {
		lt9211_write(pdata->client,0x31,0x29); //[2:0]3'b001: pre div set div2
		ucPreDiv = 2;
	} else if (g_stRxVidTiming.ulPclk_Khz >= 80000 && g_stRxVidTiming.ulPclk_Khz < 160000) {
		lt9211_write(pdata->client,0x31,0x2a); //[2:0]3'b010: pre div set div4
		ucPreDiv = 4;
	} else if (g_stRxVidTiming.ulPclk_Khz >= 160000 && g_stRxVidTiming.ulPclk_Khz < 320000) {
		lt9211_write(pdata->client,0x31,0x2b); //[2:0]3'b011: pre div set div8
		ucPreDiv = 8;
//		ulLvdsTXPhyClk = ulDessc_Pix_Clk * 3.5;
	} else if (g_stRxVidTiming.ulPclk_Khz >= 320000) {
		lt9211_write(pdata->client,0x31,0x2f); //[2:0]3'b111: pre div set div16
		ucPreDiv = 16;
//		ulLvdsTXPhyClk = ulDessc_Pix_Clk * 3.5;
	}
	/*txpll serick_divsel*/
	lt9211_write(pdata->client,0xff,0x82);
	if (ulLvdsTXPhyClk >= 640000 ) { //640M~1.28G
		lt9211_write(pdata->client,0x32,0x42);
		ucSericlkDiv = 1; //sericlk div1 [6:4]:0x40
	} else if (ulLvdsTXPhyClk >= 320000 && ulLvdsTXPhyClk < 640000) {
		lt9211_write(pdata->client,0x32,0x02);
		ucSericlkDiv = 2; //sericlk div2 [6:4]:0x00
	} else if (ulLvdsTXPhyClk >= 160000 && ulLvdsTXPhyClk < 320000) {
		lt9211_write(pdata->client,0x32,0x12);
		ucSericlkDiv = 4; //sericlk div4 [6:4]:0x10
	} else if (ulLvdsTXPhyClk >= 80000 && ulLvdsTXPhyClk < 160000) {
		lt9211_write(pdata->client,0x32,0x22);
		ucSericlkDiv = 8; //sericlk div8 [6:4]:0x20
	} else { //40M~80M
		lt9211_write(pdata->client,0x32,0x32);
		ucSericlkDiv = 16; //sericlk div16 [6:4]:0x30
	}
	/* txpll_pix_mux_sel & txpll_pixdiv_sel*/
	lt9211_write(pdata->client,0xff,0x82);
	if (g_stRxVidTiming.ulPclk_Khz > 150000) {
		lt9211_write(pdata->client,0x33,0x04); //pixclk > 150000, pixclk mux sel (vco clk / 3.5)
		ucPixClkDiv = 3;//3.5*2 避免浮点数运算
	} else {
		ucPixClkDiv = (unsigned char)((ulLvdsTXPhyClk * ucSericlkDiv ) / (g_stRxVidTiming.ulPclk_Khz * 7));
		if (ucPixClkDiv <= 1) {
			lt9211_write(pdata->client,0x33,0x00); //pixclk div sel /7
		} else if ((ucPixClkDiv > 1) && (ucPixClkDiv <= 2)) {
			lt9211_write(pdata->client,0x33,0x01); //pixclk div sel /14
		} else if ((ucPixClkDiv > 2) && (ucPixClkDiv <= 4)) {
			lt9211_write(pdata->client,0x33,0x02); //pixclk div sel /28
		} else if ((ucPixClkDiv > 4) && (ucPixClkDiv <= 8)) {
			lt9211_write(pdata->client,0x33,0x03); //pixclk div sel /56
		} else {
			lt9211_write(pdata->client,0x33,0x03); //pixclk div sel /56
		}
	}

	ucDivSet = (unsigned char)((ulLvdsTXPhyClk * ucSericlkDiv) / (g_stRxVidTiming.ulPclk_Khz / ucPreDiv));

	lt9211_write(pdata->client,0x34,0x01); //txpll div set software output enable
	lt9211_write(pdata->client,0x35,ucDivSet);
	//PRINT_DEG("ulPclk_Khz: %ld, ucPreDiv: %d, ucSericlkDiv: %d, ucPixClkDiv: %d(/10), ucDivSet: %d, ulLvdsTXPhyClk: %d\n",
	//				g_stRxVidTiming.ulPclk_Khz, ucPreDiv, ucSericlkDiv, ucPixClkDiv, ucDivSet, ulLvdsTXPhyClk);
}

unsigned short Drv_VidChkSingle_Get(unsigned char ucPara)
{
	unsigned short usRtn = 0;

	lt9211_write(pdata->client,0xff,0x81);
	lt9211_write(pdata->client,0x0b,0x7f);
	lt9211_write(pdata->client,0x0b,0xff);
	usleep_range(1000, 8000); //80ms
	lt9211_write(pdata->client,0xff,0x86);
	switch(ucPara) {
	case HTOTAL_POS:
		usRtn = (lt9211_read(pdata->client,0x60) << 8) + lt9211_read(pdata->client,0x61);
		break;
	case HACTIVE_POS:
		usRtn = (lt9211_read(pdata->client,0x5c) << 8) + lt9211_read(pdata->client,0x5d);
		break;
	case HFP_POS:
		usRtn = (lt9211_read(pdata->client,0x58) << 8) + lt9211_read(pdata->client,0x59);
		break;
	case HSW_POS:
		usRtn = (lt9211_read(pdata->client,0x50) << 8) + lt9211_read(pdata->client,0x51);
		break;
	case HBP_POS:
		usRtn = (lt9211_read(pdata->client,0x54) << 8) + lt9211_read(pdata->client,0x55);
		break;
	case VTOTAL_POS:
		usRtn = (lt9211_read(pdata->client,0x62) << 8) + lt9211_read(pdata->client,0x63);
		break;
	case VACTIVE_POS:
		usRtn = (lt9211_read(pdata->client,0x5e) << 8) + lt9211_read(pdata->client,0x5f);
		break;
	case VFP_POS:
		usRtn = (lt9211_read(pdata->client,0x5a) << 8) + lt9211_read(pdata->client,0x5b);
		break;
	case VSW_POS:
		usRtn = (lt9211_read(pdata->client,0x52) << 8) + lt9211_read(pdata->client,0x53);
		break;
	case VBP_POS:
		usRtn = (lt9211_read(pdata->client,0x56) << 8) + lt9211_read(pdata->client,0x57);
		break;
	case HSPOL_POS:
		usRtn = (lt9211_read(pdata->client,0x4f) & 0x01);
		break;
	case VSPOL_POS:
		usRtn = (lt9211_read(pdata->client,0x4f) & 0x02);
		break;
	default:
		break;
	}
	return usRtn;
}

unsigned char Drv_VidChk_FrmRt_Get(void)
{
	unsigned char ucframerate = 0;
	unsigned int ulframetime = 0;

	lt9211_write(pdata->client,0xff,0x86);
	ulframetime = lt9211_read(pdata->client,0x43);
	ulframetime = (ulframetime << 8) + lt9211_read(pdata->client,0x44);
	ulframetime = (ulframetime << 8) + lt9211_read(pdata->client,0x45);
	ucframerate = (unsigned char)(((unsigned int)25000000 / (unsigned int)(ulframetime))); //2500000/ulframetime
	return ucframerate;
}

void Drv_VidChkAll_Get(OUT StructVidChkTiming *video_time)
{
	video_time->usHtotal	=	 Drv_VidChkSingle_Get(HTOTAL_POS);
	video_time->usHact	  =	 Drv_VidChkSingle_Get(HACTIVE_POS);
	video_time->usHfp	   =	 Drv_VidChkSingle_Get(HFP_POS);
	video_time->usHs		=	 Drv_VidChkSingle_Get(HSW_POS);
	video_time->usHbp	   =	 Drv_VidChkSingle_Get(HBP_POS);

	video_time->usVtotal	=	 Drv_VidChkSingle_Get(VTOTAL_POS);
	video_time->usVact	  =	 Drv_VidChkSingle_Get(VACTIVE_POS);
	video_time->usVfp	   =	 Drv_VidChkSingle_Get(VFP_POS);
	video_time->usVs		=	 Drv_VidChkSingle_Get(VSW_POS);
	video_time->usVbp	   =	 Drv_VidChkSingle_Get(VBP_POS);

	video_time->ucHspol	 =	 Drv_VidChkSingle_Get(HSPOL_POS);
	video_time->ucVspol	 =	 Drv_VidChkSingle_Get(VSPOL_POS);
	video_time->ucFrameRate =	 Drv_VidChk_FrmRt_Get();
	PRINT_DEG("Htotal=%u Hact=%u Hfp=%u Hs=%u Hbp=%u Vtotal=%u Vact=%u Vfp=%u Vs=%u Vbp=%u FrameRate=%u \n",
			  video_time->usHtotal, video_time->usHact, video_time->usHfp, video_time->usHs, video_time->usHbp,
			  video_time->usVtotal,  video_time->usVact, video_time->usVfp, video_time->usVs, video_time->usVbp, video_time->ucFrameRate);
}

void Drv_LvdsTxPort_Set(void)
{

	lt9211_write(pdata->client, 0xff, 0x85);
#if ((LVDSTX_PORT_SEL == PORTA) || (LVDSTX_PORT_SEL == PORTB))
	lt9211_write(pdata->client, 0x6f, (lt9211_read(pdata->client,0x6f) | 0x80)); //[7]lvds function enable //[4]0:output 1port; [4]1:output 2port;
	//only portb output must use port copy from porta, so lvds digtial output port sel 2ports.
#elif (LVDSTX_PORT_SEL == DOU_PORT)
	lt9211_write(pdata->client, 0x6f, (lt9211_read(pdata->client,0x6f) | 0x90)); //[7]lvds function enable //[4]0:output 1port; [4]1:output 2port;
#endif
}

void Drv_LvdsTx_VidTiming_Set(void)
{
	u16 vss,eav,sav;
	usleep_range(100*1000,110*1000);
	lt9211_write(pdata->client,0xff,0x85);

	vss = g_stVidChk.usVs + g_stVidChk.usVbp;
	eav = g_stVidChk.usHs + g_stVidChk.usHbp + g_stVidChk.usHact + 4;
	sav = g_stVidChk.usHs + g_stVidChk.usHbp;

	lt9211_write(pdata->client,0x5f,0x00);
	lt9211_write(pdata->client,0x60,0x00);
	lt9211_write(pdata->client,0x62,(u8)(g_stVidChk.usVact>>8));		 //vact[15:8]
	lt9211_write(pdata->client,0x61,(u8)(g_stVidChk.usVact));			//vact[7:0]
	lt9211_write(pdata->client,0x63,(u8)(vss));						   //vss[7:0]
	lt9211_write(pdata->client,0x65,(u8)(eav>>8));						//eav[15:8]
	lt9211_write(pdata->client,0x64,(u8)(eav));						   //eav[7:0]
	lt9211_write(pdata->client,0x67,(u8)(sav>>8));						//sav[15:8]
	lt9211_write(pdata->client,0x66,(u8)(sav));						   //sav[7:0]
}

void Drv_LvdsTxVidFmt_Set(void)
{

	lt9211_write(pdata->client,0xff,0x85);
#if (LVDSTX_MODE == SYNC_MODE)
	lt9211_write(pdata->client,0X6e,(lt9211_read(pdata->client,0x6e) & BIT3_0));
#elif (LVDSTX_MODE == DE_MODE)
	lt9211_write(pdata->client,0X6e,(lt9211_read(pdata->client,0x6e) | BIT3_1)); //[3]lvdstx de mode
#endif
	if(pdata->format) {
		lt9211_write(pdata->client,0x6f,(lt9211_read(pdata->client,0x6f) | BIT6_1)); //[6]1:JEIDA MODE
		PRINT_DEG("Data Format: JEIDA\n");
	} else {
		lt9211_write(pdata->client,0x6f,(lt9211_read(pdata->client,0x6f) & BIT6_0)); //[6]0:VESA MODE;
		PRINT_DEG("Data Format: VESA\n");
	}
#if (LVDSTX_COLORSPACE == RGB)
	PRINT_DEG("ColorSpace: RGB\n");
#if (LVDSTX_COLORDEPTH == DEPTH_6BIT)
	PRINT_DEG("ColorDepth: 6Bit\n");
	lt9211_write(pdata->client,0x6f,(lt9211_read(pdata->client,0x6f) | 0x40)); //RGB666 [6]RGB666 output must select jeida mode
	lt9211_write(pdata->client,0x6f,(lt9211_read(pdata->client,0x6f) & 0xf3));
#elif (LVDSTX_COLORDEPTH == DEPTH_8BIT)
	PRINT_DEG("ColorDepth: 8Bit\n");
	lt9211_write(pdata->client,0x6f,(lt9211_read(pdata->client,0x6f) | 0X04));
#elif (LVDSTX_COLORDEPTH == DEPTH_10BIT)
	PRINT_DEG("ColorDepth: 10Bit\n");
	lt9211_write(pdata->client,0x6f,(lt9211_read(pdata->client,0x6f) | 0X0c));
#endif
#elif (LVDSTX_COLORSPACE == YUV422)
	PRINT_DEG("ColorSpace: YUV422\n");
	lt9211_write(pdata->client,0xff,0x85);
#if (LVDSTX_COLORDEPTH == DEPTH_8BIT)
	PRINT_DEG("ColorDepth: 8Bit\n");
	lt9211_write(pdata->client,0x6f,(lt9211_read(pdata->client,0x6f) | 0X04));
#if (LVDSTX_LANENUM == FIVE_LANE)
	PRINT_DEG("LvdsLaneNum: 5Lane\n");
	lt9211_write(pdata->client,0x6f,(lt9211_read(pdata->client,0x6f) | 0X40)); //YUV422-8bpc-5lane mode output must sel jeida mode
	lt9211_write(pdata->client,0x6f,(lt9211_read(pdata->client,0x6f) | 0X28));  ////YUV422-8bpc-5lane mode set
#else
	PRINT_DEG("LvdsLaneNum: 4Lane\n");
	lt9211_write(pdata->client,0x6f,(lt9211_read(pdata->client,0x6f) & 0Xbf)); //YUV422-8bpc-5lane mode output must sel jeida mode
#endif
#endif
#endif
#if (LVDSTX_SYNC_INTER_MODE == ENABLED)
	Drv_LvdsTx_VidTiming_Set();
	PRINT_DEG("Lvds Sync Code Mode: Internal\n"); //internal sync code mode
	lt9211_write(pdata->client,0x68,(lt9211_read(pdata->client,0x68) | 0X01));
#if (LVDSTX_VIDEO_FORMAT == I_FORMAT)
	PRINT_DEG("Lvds Video Format: interlaced\n"); //internal sync code mode
	lt9211_write(pdata->client,0x68,(lt9211_read(pdata->client,0x68) | 0X02));
#endif
#if (LVDSTX_SYNC_CODE_SEND == REPECTIVE)
	PRINT_DEG("Lvds Sync Code Send: respectively.\n"); //sync code send method sel respectively
	lt9211_write(pdata->client,0x68,(lt9211_read(pdata->client,0x68) | 0X04));
#endif
#else
	lt9211_write(pdata->client,0x68,0x00);
#endif
}

void Drv_LvdsTxLaneNum_Set(void)
{

	lt9211_write(pdata->client,0xff,0x85);
	lt9211_write(pdata->client,0x4a,0x01); //[0]hl_swap_en; [7:6]tx_pt0_src_sel: 0-pta;1-ptb
	lt9211_write(pdata->client,0x4b,0x00);
	lt9211_write(pdata->client,0x4c,0x10);
	lt9211_write(pdata->client,0x4d,0x20);
	lt9211_write(pdata->client,0x4e,0x50);
	lt9211_write(pdata->client,0x4f,0x30);
#if (LVDSTX_LANENUM  == FOUR_LANE)
	lt9211_write(pdata->client,0x50,0x46); //[7:6]tx_pt1_src_sel: 0-pta;1-ptb
	lt9211_write(pdata->client,0x51,0x10);
	lt9211_write(pdata->client,0x52,0x20);
	lt9211_write(pdata->client,0x53,0x50);
	lt9211_write(pdata->client,0x54,0x30);
	lt9211_write(pdata->client,0x55,0x00); //[7:4]pt1_tx4_src_sel
	lt9211_write(pdata->client,0x56,0x20); //[3:0]pt1_tx5_src_sel
	//[6:5]rgd_mltx_src_sel: 0-mipitx;1-lvdstx
#elif (LVDSTX_LANENUM == FIVE_LANE)
	lt9211_write(pdata->client,0x50,0x44); //[7:6]tx_pt1_src_sel: 0-pta;1-ptb
	lt9211_write(pdata->client,0x51,0x00);
	lt9211_write(pdata->client,0x52,0x10);
	lt9211_write(pdata->client,0x53,0x20);
	lt9211_write(pdata->client,0x54,0x50);
	lt9211_write(pdata->client,0x55,0x30); //[7:4]pt1_tx4_src_sel
	lt9211_write(pdata->client,0x56,0x24); //[3:0]pt1_tx5_src_sel
	//[6:5]rgd_mltx_src_sel: 0-mipitx;1-lvdstx
#endif
}

void Drv_LvdsTxPort_Swap(void)
{
#if (LVDSTX_PORT_SWAP == ENABLED) || (LVDSTX_PORT_SEL == PORTB)
	lt9211_write(pdata->client,0xff,0x85);
	lt9211_write(pdata->client,0x4a,0x41);
	lt9211_write(pdata->client,0x50,(lt9211_read(pdata->client,0x50) & BIT6_0));
#else
	lt9211_write(pdata->client,0xff,0x85);
	lt9211_write(pdata->client,0x4a,0x01);
	lt9211_write(pdata->client,0x50,(lt9211_read(pdata->client,0x50) | BIT6_1));
#endif
}

#if (LVDSTX_PORT_COPY == ENABLED)
void Drv_LvdsTxPort_Copy(void)
{
	lt9211_write(pdata->client,0xff,0x82);
	lt9211_write(pdata->client,0x36,(lt9211_read(pdata->client,0x36) | 0x03)); //port swap enable when porta & portb enable
	lt9211_write(pdata->client,0xff,0x85);
#if (LVDSTX_PORT_SEL == PORTA)
	lt9211_write(pdata->client,0x4a,(lt9211_read(pdata->client,0x4a) & 0xbf));
	lt9211_write(pdata->client,0x50,(lt9211_read(pdata->client,0x50) & 0xbf));
	PRINT_DEG("Port A Copy\n");
#elif (LVDSTX_PORT_SEL == PORTB)
	lt9211_write(pdata->client,0x6f,(lt9211_read(pdata->client,0x6f) | 0x10)); //[7]lvds function enable //[4]0:output 1port; [4]1:output 2port;
	lt9211_write(pdata->client,0x4a,(lt9211_read(pdata->client,0x4a) | 0x40));
	lt9211_write(pdata->client,0x50,(lt9211_read(pdata->client,0x50) | 0x40));
	PRINT_DEG("Port B Copy\n");
#endif
}
#endif

void Drv_LvdsTxSw_Rst(void)
{
	lt9211_write(pdata->client,0xff,0x81);
	lt9211_write(pdata->client,0x08,0x6f); //LVDS TX SW reset
	usleep_range(2000, 2200);//2ms
	lt9211_write(pdata->client,0x08,0x7f);
	PRINT_DEG("LVDS Tx Video Out\n");
}

void Drv_LvdsTxCsc_Set(void)
{
#if LVDSTX_COLORSPACE == RGB
	PRINT_DEG("Csc Set:	RGB\n");
#elif LVDSTX_COLORSPACE == YUV422
	lt9211_write(pdata->client,0xff,0x86);
	if((lt9211_read(pdata->client,0x87) & 0x10) == 0) {
		lt9211_write(pdata->client,0x85,lt9211_read(pdata->client,0x85) | 0x10);
	} else {
		lt9211_write(pdata->client,0x87,lt9211_read(pdata->client,0x87) & 0xef);
	}
	if((lt9211_read(pdata->client,0x86) & 0x04) == 0) {
		lt9211_write(pdata->client,0x86,lt9211_read(pdata->client,0x86) | 0x40);
	} else {
		lt9211_write(pdata->client,0x86,lt9211_read(pdata->client,0x86) & 0xfb);
	}
	PRINT_DEG("Csc Set:	YUV422\n");
#endif
}

void Mod_LvdsTxDig_Set(void)
{
	Drv_LvdsTxPort_Set();
	Drv_LvdsTxVidFmt_Set();
	Drv_LvdsTxLaneNum_Set();
	Drv_LvdsTxPort_Swap();
#if (LVDSTX_PORT_COPY == ENABLED)
	Drv_LvdsTxPort_Copy();
#endif
	Drv_LvdsTxSw_Rst();
	Drv_LvdsTxCsc_Set();
}

unsigned char Drv_LvdsTxPll_Cali(void)
{
	unsigned char ucloopx;
	unsigned char ucRtn = 0;

	lt9211_write(pdata->client,0xff,0x81);
	lt9211_write(pdata->client,0x0c,0xfe); //txpll reset
	usleep_range(10*1000, 20*1000);//1ms
	lt9211_write(pdata->client,0x0c,0xff); //txpll release
	do {
		lt9211_write(pdata->client,0xff,0x87);
		lt9211_write(pdata->client,0x0f,0x00);
		lt9211_write(pdata->client,0x0f,0x01);
		usleep_range(20*1000, 22*1000);//20ms
		ucloopx++;
	} while((ucloopx < 3) && ((lt9211_read(pdata->client,0x39) & 0x01) != 0x01));
	if(lt9211_read(pdata->client,0x39) & 0x04) {
		ucRtn = 1;
		PRINT_DEG("Tx Pll Lock\n");
	} else {
		PRINT_DEG("Tx Pll Unlocked\n");
	}
	return ucRtn;
}

void Drv_MipiRx_PhyPowerOn(void)
{
	lt9211_write(pdata->client,0xff,0xd0);
	lt9211_write(pdata->client,0x00,(lt9211_read(pdata->client,0x00) | (pdata->mipi_lane & 0x3)));	// 0: 4 Lane / 1: 1 Lane / 2 : 2 Lane / 3: 3 Lane
	lt9211_write(pdata->client,0xff,0x82);
	lt9211_write(pdata->client,0x01,0x11); //MIPI RX portA disable
#if MIPIRX_PORT_SEL == PORTA
	lt9211_write(pdata->client,0x01,0x91); //MIPI RX portA enable
	lt9211_write(pdata->client,0x02,0x00); //[5][1]:0 mipi mode, no swap
	lt9211_write(pdata->client,0x03,0xcc); //port A & B eq current reference
	lt9211_write(pdata->client,0x09,0x21); //[3]0: select link clk from port-A, [1]0: mlrx_clk2pll disable
	lt9211_write(pdata->client,0x13,0x0c); //MIPI port A clk lane rterm & high speed en
#elif MIPIRX_PORT_SEL == PORTB
	lt9211_write(pdata->client,0x01,0x19); //MIPI RX portB enable
	lt9211_write(pdata->client,0x02,0x00); //[5][1]:0 mipi mode, no swap
	lt9211_write(pdata->client,0x03,0xaa); //port A & B eq current reference
	lt9211_write(pdata->client,0x09,0x29); //[3]1: select link clk from port-B, [1]0: mlrx_clk2pll disable
	lt9211_write(pdata->client,0x13,0x0c); //MIPI port A clk lane rterm & high speed en
	lt9211_write(pdata->client,0x14,0x03); //Port-B clk lane software enable
#else
	lt9211_write(pdata->client,0x01,0x99); //MIPI RX portB enable
	lt9211_write(pdata->client,0x02,0x00); //[5][1]:0 mipi mode, no swap
	lt9211_write(pdata->client,0x03,0xcc); //port A & B eq current reference
	lt9211_write(pdata->client,0x09,0x29); //[3]1: select link clk from port-B, [1]0: mlrx_clk2pll disable
	lt9211_write(pdata->client,0x13,0x0c); //MIPI port A clk lane rterm & high speed en
	lt9211_write(pdata->client,0x14,0x03); //Port-B clk lane software enable
#endif

	lt9211_write(pdata->client,0xff,0xd0);
	lt9211_write(pdata->client,0x01,0x00); //mipi rx data lane term enable time: 39ns;
	lt9211_write(pdata->client,0x02,0x0e); //mipi rx hs settle time defult set: 0x05;
	lt9211_write(pdata->client,0x05,0x00); //mipi rx lk lane term enable time: 39ns;
	lt9211_write(pdata->client,0xff,0x81);
	lt9211_write(pdata->client,0x09,0xde); //mipi rx dphy reset
	lt9211_write(pdata->client,0x09,0xdf); //mipi rx dphy release
}

void Drv_System_VidChkClk_SrcSel(unsigned char ucSrc)
{
	lt9211_write(pdata->client,0xff,0x81);
	lt9211_write(pdata->client,0x80,(lt9211_read(pdata->client,0x80) & 0xfc));
	switch (ucSrc) {
	case RXPLL_PIX_CLK:
		lt9211_write(pdata->client,0x80,(lt9211_read(pdata->client,0x80) | RXPLL_PIX_CLK));
		break;
	case DESSCPLL_PIX_CLK:
		lt9211_write(pdata->client,0x80,(lt9211_read(pdata->client,0x80) | DESSCPLL_PIX_CLK));
		break;
	case RXPLL_DEC_DDR_CLK:
		lt9211_write(pdata->client,0x80,(lt9211_read(pdata->client,0x80) | RXPLL_DEC_DDR_CLK));
		break;
	case MLRX_BYTE_CLK:
		lt9211_write(pdata->client,0x80,(lt9211_read(pdata->client,0x80) | MLRX_BYTE_CLK));
		break;

	}
}

void Drv_System_VidChk_SrcSel(unsigned char ucSrc)
{
	lt9211_write(pdata->client,0xff,0x86);
	lt9211_write(pdata->client,0x3f,(lt9211_read(pdata->client,0x80) & 0xf8));
	switch (ucSrc) {
	case LVDSRX:
		lt9211_write(pdata->client,0x3f,LVDSRX);
		PRINT_DEG("LVDSRX\n");
		break;
	case MIPIRX:
		lt9211_write(pdata->client,0x3f,MIPIRX);
		PRINT_DEG("MIPIRX\n");
		break;
	case TTLRX:
		lt9211_write(pdata->client,0x3f,TTLRX);
		PRINT_DEG("TTLRX\n");
		break;
	case PATTERN:
		lt9211_write(pdata->client,0x3f,PATTERN);
		PRINT_DEG("PATTERN\n");
		break;
	case LVDSDEBUG:
		lt9211_write(pdata->client,0x3f,LVDSDEBUG);
		PRINT_DEG("LVDSDEBUG\n");
	case MIPIDEBUG:
		lt9211_write(pdata->client,0x3f,MIPIDEBUG);
		PRINT_DEG("MIPIDEBUG\n");
		break;
	case TTLDEBUG:
		lt9211_write(pdata->client,0x3f,TTLDEBUG);
		PRINT_DEG("TTLDEBUG\n");
		break;

	}
}

void Drv_SystemActRx_Sel(IN unsigned char ucSrc)
{
	lt9211_write(pdata->client,0xff,0x85);
	lt9211_write(pdata->client,0x30,(lt9211_read(pdata->client,0x30) & 0xf8));
	switch(ucSrc) {
	case LVDSRX:
		PRINT_DEG("LVDSRX\n");
		lt9211_write(pdata->client,0x30,(lt9211_read(pdata->client,0x30) | LVDSRX));
		lt9211_write(pdata->client,0x30,(lt9211_read(pdata->client,0x30) & 0xcf)); //[5:4]00: LVDSRX
		break;
	case MIPIRX:
		PRINT_DEG("MIPIRX\n");
		lt9211_write(pdata->client,0x30,(lt9211_read(pdata->client,0x30) | MIPIRX));
		lt9211_write(pdata->client,0x30,(lt9211_read(pdata->client,0x30) | BIT4_1)); //[5:4]01: MIPIRX
		break;
	case TTLRX:
		PRINT_DEG("TTLRX\n");
		lt9211_write(pdata->client,0x30,(lt9211_read(pdata->client,0x30) | TTLRX));
		break;
	case PATTERN:
		PRINT_DEG("PATTERN\n");
		lt9211_write(pdata->client,0x30,(lt9211_read(pdata->client,0x30) | PATTERN));
		break;
	default:
		PRINT_DEG("default\n");
		lt9211_write(pdata->client,0x30,(lt9211_read(pdata->client,0x30) | LVDSRX));
		break;

	}
}

void Mod_MipiRxDig_Set(void)
{
	//Drv_MipiRx_InputSel();
	lt9211_write(pdata->client,0xff,0xd0);
#if (MIPIRX_INPUT_SEL == MIPI_CSI)
	lt9211_write(pdata->client,0x04,0x10); //[4]1: CSI enable
	lt9211_write(pdata->client,0x21,0xc6); //[7](dsi: hsync_level(for pcr adj) = hsync_level; csi:hsync_level(for pcr adj) = de_level)
	PRINT_DEG("Mipi CSI Input\n");
#else
	lt9211_write(pdata->client,0x04,0x00); //[4]0: DSI enable
	lt9211_write(pdata->client,0x21,0x46); //[7](dsi: hsync_level(for pcr adj) = hsync_level; csi:hsync_level(for pcr adj) = de_level)
	PRINT_DEG("Mipi DSI Input\n");
#endif
// Drv_MipiRx_LaneSet();
	lt9211_write(pdata->client,0xff,0x85);
	lt9211_write(pdata->client,0x3f,0x08); //MLRX HS/LP control conmand enable
	lt9211_write(pdata->client,0x40,0x04); //[2:0]pa_ch0_src_sel ch4 data
	lt9211_write(pdata->client,0x41,0x03); //[2:0]pa_ch1_src_sel ch3 data
	lt9211_write(pdata->client,0x42,0x02); //[2:0]pa_ch2_src_sel ch2 data
	lt9211_write(pdata->client,0x43,0x01); //[2:0]pa_ch3_src_sel ch1 data
	lt9211_write(pdata->client,0x45,0x04); //[2:0]pb_ch0_src_sel ch9 data
	lt9211_write(pdata->client,0x46,0x03); //[2:0]pb_ch1_src_sel ch8 data
	lt9211_write(pdata->client,0x47,0x02); //[2:0]pb_ch2_src_sel ch7 data
	lt9211_write(pdata->client,0x48,0x01); //[2:0]pb_ch3_src_sel ch6 data

#if MIPIRX_PORT_SWAP == ENABLED
	lt9211_write(pdata->client,0x44,0x40); //[6]mlrx port A output select port B;[2:0]pa_ch4_src_sel ch0 data
	lt9211_write(pdata->client,0x49,0x00); //[6]mlrx port B output select port B;[2:0]pb_ch4_src_sel ch5 data
#else
	lt9211_write(pdata->client,0x44,0x00); //[6]mlrx port A output select port A;[2:0]pa_ch4_src_sel ch0 data
	lt9211_write(pdata->client,0x49,0x40); //[6]mlrx port B output select port A;[2:0]pb_ch4_src_sel ch5 data
#endif
}

unsigned char Drv_MipiRx_VidTiming_Get(void)
{
	//unsigned char val1, val2;
	//unsigned int frame_time, line_time,loopx;
	lt9211_write(pdata->client,0xff,0xd0);
	usleep_range(50000,100000);
	g_stMipiRxVidTiming_Get.ucLane0SetNum  = lt9211_read(pdata->client,0x88);
	g_stMipiRxVidTiming_Get.ucLane0SotData = lt9211_read(pdata->client,0x89);
	g_stMipiRxVidTiming_Get.ucLane1SetNum  = lt9211_read(pdata->client,0x8a);
	g_stMipiRxVidTiming_Get.ucLane1SotData = lt9211_read(pdata->client,0x8b);
	g_stMipiRxVidTiming_Get.ucLane2SetNum  = lt9211_read(pdata->client,0x8c);
	g_stMipiRxVidTiming_Get.ucLane2SotData = lt9211_read(pdata->client,0x8d);
	g_stMipiRxVidTiming_Get.ucLane3SetNum  = lt9211_read(pdata->client,0x8e);
	g_stMipiRxVidTiming_Get.ucLane3SotData = lt9211_read(pdata->client,0x8f);
	if((g_stMipiRxVidTiming_Get.ucLane0SetNum > 0x10) && (g_stMipiRxVidTiming_Get.ucLane0SetNum < 0x50)) {
		PRINT_DEG("Set Mipi Rx Settle: 0x%x\n", (g_stMipiRxVidTiming_Get.ucLane0SetNum - 5));
		lt9211_write(pdata->client,0xff,0xd0);
		lt9211_write(pdata->client,0x02,(g_stMipiRxVidTiming_Get.ucLane0SetNum - 10));
	} else {
		PRINT_DEG("Set Mipi Rx Settle: 0x0e\n"); //mipi rx cts test need settle 0x0e
		lt9211_write(pdata->client,0xff,0xd0);
		lt9211_write(pdata->client,0x02,0x08);
	}

	//Drv_MipiRx_HactGet
	lt9211_write(pdata->client,0xff,0xd0);
	g_stMipiRxVidTiming_Get.usVact = (lt9211_read(pdata->client,0x85) << 8) +lt9211_read(pdata->client,0x86);
	g_stMipiRxVidTiming_Get.ucFmt  = (lt9211_read(pdata->client,0x84) & 0x0f);
	g_stMipiRxVidTiming_Get.ucPa_Lpn = lt9211_read(pdata->client,0x9c);
	g_stMipiRxVidTiming_Get.uswc = (lt9211_read(pdata->client,0x82) << 8) + lt9211_read(pdata->client,0x83); //
	switch (g_stMipiRxVidTiming_Get.ucFmt) {
	case 0x01: //DSI-YUV422-10bpc
	case 0x0e: //CSI-YUV422-10bpc
		g_stMipiRxVidTiming_Get.usHact = (g_stMipiRxVidTiming_Get.uswc << 3) / 20; //wc = hact * 20bpp/8
		break;
	case 0x02: //DSI-YUV422-12bpc
		g_stMipiRxVidTiming_Get.usHact = (g_stMipiRxVidTiming_Get.uswc << 3) / 24; //wc = hact * 24bpp/8
		break;
	case 0x03: //YUV422-8bpc
		g_stMipiRxVidTiming_Get.usHact = g_stMipiRxVidTiming_Get.uswc >> 1; //wc = hact * 16bpp/8
		break;
	case 0x04: //RGB10bpc
		g_stMipiRxVidTiming_Get.usHact = (g_stMipiRxVidTiming_Get.uswc << 3) / 30; //wc = hact * 30bpp/8
		break;
	case 0x05: //RGB12bpc
		g_stMipiRxVidTiming_Get.usHact = (g_stMipiRxVidTiming_Get.uswc << 3) / 36; //wc = hact * 36bpp/8
		break;
	case 0x06: //YUV420-8bpc
	case 0x0a: //RGB8bpc
		g_stMipiRxVidTiming_Get.usHact = (g_stMipiRxVidTiming_Get.uswc << 3) / 24; //wc = hact * 24bpp/8
		break;
	case 0x07: //RGB565
		g_stMipiRxVidTiming_Get.usHact = g_stMipiRxVidTiming_Get.uswc >> 1; //wc = hact * 16bpp/8
		break;
	case 0x08: //RGB6bpc
	case 0x09: //RGB6bpc_losely
		g_stMipiRxVidTiming_Get.usHact = (g_stMipiRxVidTiming_Get.uswc << 3) / 18; //wc = hact * 18bpp/8
		break;
	case 0x0b: //RAW8
		g_stMipiRxVidTiming_Get.usHact = g_stMipiRxVidTiming_Get.uswc; //wc = hact * 8bpp/8
		break;
	case 0x0c: //RAW10
		g_stMipiRxVidTiming_Get.usHact = (g_stMipiRxVidTiming_Get.uswc << 3) / 10; //wc = hact * 10bpp/8
		break;
	case 0x0d: //RAW12
		g_stMipiRxVidTiming_Get.usHact = (g_stMipiRxVidTiming_Get.uswc << 3) / 12; //wc = hact * 12bpp/8
		break;
	default:
		g_stMipiRxVidTiming_Get.usHact = (g_stMipiRxVidTiming_Get.uswc << 3) / 24; //wc = hact * 24bpp/8
		break;
	}

	if((g_stMipiRxVidTiming_Get.usHact < 400) || (g_stMipiRxVidTiming_Get.usVact < 400)) {
		PRINT_DEG("RX No Video Get\n");
		return FAIL;
	} else {
		PRINT_DEG("hact = %d\n",g_stMipiRxVidTiming_Get.usHact);
		PRINT_DEG("vact = %d\n",g_stMipiRxVidTiming_Get.usVact);
		PRINT_DEG("fmt = 0x%x\n", g_stMipiRxVidTiming_Get.ucFmt);
		PRINT_DEG("pa_lpn = 0x%x\n", g_stMipiRxVidTiming_Get.ucPa_Lpn);
		return SUCCESS;
	}
}

unsigned char Drv_MipiRx_VidFmt_Get(IN unsigned char VidFmt)
{
	unsigned char ucRxVidFmt;

	switch (VidFmt) {
	case 0x01: //DSI-YUV422-10bpc
		ucRxVidFmt = YUV422_10bit;
		break;
	case 0x02: //DSI-YUV422-12bpc
		ucRxVidFmt = YUV422_12bit;
		break;
	case 0x03: //YUV422-8bpc
		ucRxVidFmt = YUV422_8bit;
		break;
	case 0x04: //RGB30bpp
		ucRxVidFmt = RGB_10Bit;
		break;
	case 0x05: //RGB36bpp
		ucRxVidFmt = RGB_12Bit;
		break;
	case 0x06: //YUV420-8bpc
		ucRxVidFmt = YUV420_8bit;
		break;
	case 0x07: //RGB565
		break;
	case 0x08: //RGB666
		ucRxVidFmt = RGB_6Bit;
		break;
	case 0x09: //DSI-RGB6L
		break;
	case 0x0a: //RGB888
		ucRxVidFmt = RGB_8Bit;
		break;
	case 0x0b: //RAW8
		break;
	case 0x0c: //RAW10
		break;
	case 0x0d: //RAW12
		break;
	case 0x0e: //CSI-YUV422-10
		ucRxVidFmt = YUV422_10bit;
		break;
	default:
		ucRxVidFmt = RGB_8Bit;
		break;
	}
	PRINT_DEG("MipiRx Input Format: %s\n",g_szStrRxFormat[VidFmt]);
	return ucRxVidFmt;
}

void Drv_MipiRx_VidTiming_Set(void)
{
	lt9211_write(pdata->client,0xff,0xd0);
	lt9211_write(pdata->client,0x0d,(unsigned char)(g_stRxVidTiming.usVtotal >> 8));	 //vtotal[15:8]
	lt9211_write(pdata->client,0x0e,(unsigned char)(g_stRxVidTiming.usVtotal));		  //vtotal[7:0]
	lt9211_write(pdata->client,0x0f,(unsigned char)(g_stRxVidTiming.usVact >> 8));	   //vactive[15:8]
	lt9211_write(pdata->client,0x10,(unsigned char)(g_stRxVidTiming.usVact));			//vactive[7:0]
	lt9211_write(pdata->client,0x15,(unsigned char)(g_stRxVidTiming.usVs));			  //vs[7:0]
	lt9211_write(pdata->client,0x17,(unsigned char)(g_stRxVidTiming.usVfp >> 8));		//vfp[15:8]
	lt9211_write(pdata->client,0x18,(unsigned char)(g_stRxVidTiming.usVfp));			 //vfp[7:0]
	lt9211_write(pdata->client,0x11,(unsigned char)(g_stRxVidTiming.usHtotal >> 8));	 //htotal[15:8]
	lt9211_write(pdata->client,0x12,(unsigned char)(g_stRxVidTiming.usHtotal));		  //htotal[7:0]
	lt9211_write(pdata->client,0x13,(unsigned char)(g_stRxVidTiming.usHact >> 8));	   //hactive[15:8]
	lt9211_write(pdata->client,0x14,(unsigned char)(g_stRxVidTiming.usHact));			//hactive[7:0]
	lt9211_write(pdata->client,0x4c,(unsigned char)(g_stRxVidTiming.usHs >> 8));		 //hs[15:8]
	lt9211_write(pdata->client,0x16,(unsigned char)(g_stRxVidTiming.usHs));			  //hs[7:0]
	lt9211_write(pdata->client,0x19,(unsigned char)(g_stRxVidTiming.usHfp >> 8));		//hfp[15:8]
	lt9211_write(pdata->client,0x1a,(unsigned char)(g_stRxVidTiming.usHfp));			 //hfp[7:0]
}

unsigned char Drv_MipiRx_VidTiming_Sel(void)
{
	unsigned char rtn = FAIL;

#if 1 //timing form dts
	g_stMipiRxVidTiming_Get.ucFrameRate = Drv_VidChk_FrmRt_Get();
	g_stRxVidTiming.usVtotal = pdata->vtotal;
	g_stRxVidTiming.usVact   = pdata->vact;
	g_stRxVidTiming.usVs	 = pdata->vs;
	g_stRxVidTiming.usVfp	= pdata->vfp;
	g_stRxVidTiming.usVbp	= pdata->vbp;
	g_stRxVidTiming.usHtotal = pdata->htotal;
	g_stRxVidTiming.usHact   = pdata->hact;
	g_stRxVidTiming.usHs	 = pdata->hs;
	g_stRxVidTiming.usHfp	= pdata->hfp;
	g_stRxVidTiming.usHbp	= pdata->hbp;
	//g_stRxVidTiming.ulPclk_Khz = (unsigned int)pdata->pclk;
	g_stRxVidTiming.ulPclk_Khz = (unsigned int)((unsigned int)(g_stRxVidTiming.usHtotal) * (g_stRxVidTiming.usVtotal) * (g_stMipiRxVidTiming_Get.ucFrameRate) / 1000);
	PRINT_DEG("Htotal=%u Hact=%u Hfp=%u Hs=%u Hbp=%u Vtotal=%u Vact=%u Vfp=%u Vs=%u Vbp=%u FrameRate=%u Pclk_Khz=%u\n",
			  g_stRxVidTiming.usHtotal, g_stRxVidTiming.usHact, g_stRxVidTiming.usHfp, g_stRxVidTiming.usHs, g_stRxVidTiming.usHbp,
			  g_stRxVidTiming.usVtotal, g_stRxVidTiming.usVact, g_stRxVidTiming.usVfp, g_stRxVidTiming.usVs, g_stRxVidTiming.usVbp,
			  g_stMipiRxVidTiming_Get.ucFrameRate, g_stRxVidTiming.ulPclk_Khz);
	Drv_MipiRx_VidTiming_Set();
	//PRINT_DEG("Video Timing Set %d*%d_%d\n",g_stRxVidTiming.usHact,g_stRxVidTiming.usVact,g_stMipiRxVidTiming_Get.ucFrameRate);
	rtn = SUCCESS;
#else
	unsigned char uci;
	unsigned char ucResolutionnum = 0;
	ucResolutionnum = sizeof(resolution) / sizeof(resolution[0]);
	for (uci = 0; uci < ucResolutionnum; uci++) {
		if ((g_stMipiRxVidTiming_Get.usHact == resolution[uci].usHact ) &&
			( g_stMipiRxVidTiming_Get.usVact == resolution[uci].usVact )) {
			g_stMipiRxVidTiming_Get.ucFrameRate = Drv_VidChk_FrmRt_Get();
			PRINT_DEG("FrameRate = %d\n", g_stMipiRxVidTiming_Get.ucFrameRate);
			if ((g_stMipiRxVidTiming_Get.ucFrameRate > (resolution[uci].ucFrameRate - 3)) &&
				(g_stMipiRxVidTiming_Get.ucFrameRate < (resolution[uci].ucFrameRate + 3))) {
				g_stRxVidTiming.usVtotal = resolution[uci].usVtotal;
				g_stRxVidTiming.usVact   = resolution[uci].usVact;
				g_stRxVidTiming.usVs	 = resolution[uci].usVs;
				g_stRxVidTiming.usVfp	= resolution[uci].usVfp;
				g_stRxVidTiming.usVbp	= resolution[uci].usVbp;

				g_stRxVidTiming.usHtotal = resolution[uci].usHtotal;
				g_stRxVidTiming.usHact   = resolution[uci].usHact;
				g_stRxVidTiming.usHs	 = resolution[uci].usHs;
				g_stRxVidTiming.usHfp	= resolution[uci].usHfp;
				g_stRxVidTiming.usHbp	= resolution[uci].usHbp;
				g_stRxVidTiming.ulPclk_Khz = (unsigned int)((unsigned int)(resolution[uci].usHtotal) * (resolution[uci].usVtotal) * (resolution[uci].ucFrameRate) / 1000);

				PRINT_DEG("Htotal=%u Hact=%u Hfp=%u Hs=%u Hbp=%u Vtotal=%u Vact=%u Vfp=%u Vs=%u Vbp=%u FrameRate=%u \n", g_stRxVidTiming.usHtotal,
						  g_stRxVidTiming.usHact, g_stRxVidTiming.usHfp, g_stRxVidTiming.usHs, g_stRxVidTiming.usHbp, g_stRxVidTiming.usVtotal,
						  g_stRxVidTiming.usVact, g_stRxVidTiming.usVfp, g_stRxVidTiming.usVs, g_stRxVidTiming.usVbp, g_stMipiRxVidTiming_Get.ucFrameRate);
				Drv_MipiRx_VidTiming_Set();
				//PRINT_DEG("Video Timing Set %d*%d_%d\n",g_stRxVidTiming.usHact,g_stRxVidTiming.usVact,g_stMipiRxVidTiming_Get.ucFrameRate);
				rtn = SUCCESS;
				break;
			}
		}
	}
#endif
	return rtn;
}

void DRV_DesscPll_SdmCal(void)
{
	lt9211_write(pdata->client,0xff,0xd0);//
	lt9211_write(pdata->client,0x08,0x00);//sel mipi rx sdm
	lt9211_write(pdata->client,0x26,0x80 | ((unsigned char)g_stPcrPara.Pcr_M)); //m
	lt9211_write(pdata->client,0x2d,g_stPcrPara.Pcr_UpLimit); //PCR M overflow limit setting.
	lt9211_write(pdata->client,0x31,g_stPcrPara.Pcr_DownLimit); //PCR M underflow limit setting.
	lt9211_write(pdata->client,0x27,(unsigned char)(g_stPcrPara.Pcr_K >> 16)); //a5
	lt9211_write(pdata->client,0x28,(unsigned char)(g_stPcrPara.Pcr_K >> 8)); //00
	lt9211_write(pdata->client,0x29,(unsigned char)(g_stPcrPara.Pcr_K)); //00
	lt9211_write(pdata->client,0x26,(lt9211_read(pdata->client,0x26) & 0x7f));

}

void Drv_MipiRx_DesscPll_Set(void)
{
	unsigned char ucdesscpll_pixck_div = 0;

	lt9211_write(pdata->client,0xff,0x82);
	lt9211_write(pdata->client,0x26,0x20); //[7:6]desscpll reference select Xtal clock as reference
	//[4]1'b0: dessc-pll power down
	lt9211_write(pdata->client,0x27,0x40); //prediv = 0;
	//PRINT_DEG("Mipi Rx PixClk: %ld\n",g_stRxVidTiming.ulPclk_Khz);
	if (g_stRxVidTiming.ulPclk_Khz >= 352000) {
		lt9211_write(pdata->client,0x2f,0x04);
		ucdesscpll_pixck_div = 2;
	} else if (g_stRxVidTiming.ulPclk_Khz >= 176000 && g_stRxVidTiming.ulPclk_Khz < 352000) {
		lt9211_write(pdata->client,0x2f,0x04);
		ucdesscpll_pixck_div = 2;
	} else if (g_stRxVidTiming.ulPclk_Khz >= 88000 && g_stRxVidTiming.ulPclk_Khz < 176000) {
		lt9211_write(pdata->client,0x2f,0x05);
		ucdesscpll_pixck_div = 4;
	} else if (g_stRxVidTiming.ulPclk_Khz >= 44000 && g_stRxVidTiming.ulPclk_Khz < 88000) {
		lt9211_write(pdata->client,0x2f,0x06);
		ucdesscpll_pixck_div = 8;
	} else if (g_stRxVidTiming.ulPclk_Khz >= 22000 && g_stRxVidTiming.ulPclk_Khz < 44000) {
		lt9211_write(pdata->client,0x2f,0x07);
		ucdesscpll_pixck_div = 16;
	} else {
		lt9211_write(pdata->client,0x2f,0x07);
		ucdesscpll_pixck_div = 16;
	}
	g_stPcrPara.Pcr_M = (g_stRxVidTiming.ulPclk_Khz * ucdesscpll_pixck_div) / 25;
	g_stPcrPara.Pcr_K = g_stPcrPara.Pcr_M % 1000;
	g_stPcrPara.Pcr_M = g_stPcrPara.Pcr_M / 1000;

	g_stPcrPara.Pcr_UpLimit   = g_stPcrPara.Pcr_M + 1;
	g_stPcrPara.Pcr_DownLimit = g_stPcrPara.Pcr_M - 1;
	g_stPcrPara.Pcr_K <<= 14;
	//PRINT_DEG("Mipi Rx Pcr_M:%ld Pcr_UpLimit:%ld Pcr_DownLimit:%ld Pcr_K:%ld\n",
	//	g_stPcrPara.Pcr_M, g_stPcrPara.Pcr_UpLimit, g_stPcrPara.Pcr_DownLimit, g_stPcrPara.Pcr_K);
	DRV_DesscPll_SdmCal();
	lt9211_write(pdata->client,0xff,0x81);
	lt9211_write(pdata->client,0x03,0xfe); //desscpll rst
	usleep_range(1000, 2000);//2ms
	lt9211_write(pdata->client,0x03,0xff); //desscpll rst
}

unsigned char Drv_MipiRx_PcrCali(void)
{
	unsigned char ucRtn = SUCCESS;
	unsigned char ucPcr_Cal_Cnt = 0;

//usleep_range(50*1000, 100*1000);
	lt9211_write(pdata->client,0xff,0xd0);
	lt9211_write(pdata->client,0x0a,0x02);
	lt9211_write(pdata->client,0x1e,0x51); //[7:4]pcr diff first step,[3:0]pcr diff second step
	lt9211_write(pdata->client,0x23,0x80); //0x05 MIPIRX PCR capital data set for PCR second step
	lt9211_write(pdata->client,0x24,0x70);
	lt9211_write(pdata->client,0x25,0x80);
	lt9211_write(pdata->client,0x2a,0x10);
	lt9211_write(pdata->client,0x21,0x4f);
	lt9211_write(pdata->client,0x22,0xf0);
	lt9211_write(pdata->client,0x38,0x04); //MIPIRX PCR de mode delay offset0
	lt9211_write(pdata->client,0x39,0x08);
	lt9211_write(pdata->client,0x3a,0x10);
	lt9211_write(pdata->client,0x3b,0x20); //MIPIRX PCR de mode delay offset2
	lt9211_write(pdata->client,0x3f,0x04); //PCR de mode step0 setting
	lt9211_write(pdata->client,0x40,0x08);
	lt9211_write(pdata->client,0x41,0x10);
	lt9211_write(pdata->client,0x42,0x20);
	lt9211_write(pdata->client,0x2b,0xA0);

	if (g_stRxVidTiming.ulPclk_Khz < 44000) {
		lt9211_write(pdata->client,0x0c,0x60); //[7:0]rgd_vsync_dly(sram rd delay)
		lt9211_write(pdata->client,0x1b,0x00); //pcr wr dly[15:0]
		lt9211_write(pdata->client,0x1c,0x60); //pcr wr dly[7:0]
	} else {
		lt9211_write(pdata->client,0x0c,0x40); //[7:0]rgd_vsync_dly(sram rd delay)
		lt9211_write(pdata->client,0x1b,0x00); //pcr wr dly[15:0]
		lt9211_write(pdata->client,0x1c,0x40); //pcr wr dly[7:0]
	}
	lt9211_write(pdata->client,0xff,0x81);
	lt9211_write(pdata->client,0x09,0xdb);
	lt9211_write(pdata->client,0x09,0xdf); //pcr rst
	lt9211_write(pdata->client,0xff,0xd0);
	lt9211_write(pdata->client,0x08,0x80);
	lt9211_write(pdata->client,0x08,0x00);
	//usleep_range(5*1000, 10*1000); //10ms
	do {
		usleep_range(20*1000, 40*1000);//100ms
		ucPcr_Cal_Cnt++;
		//PRINT_DEG("LT9211 PCR M = 0x%x\n",(lt9211_read(pdata->client,0x94)&0x7F));
	} while((ucPcr_Cal_Cnt < 200) && ((lt9211_read(pdata->client,0x87) & 0x18) != 0x18));

	if((lt9211_read(pdata->client,0x87) & 0x18) != 0x18) {
		PRINT_DEG("LT9211 pcr unstable\n");
		ucRtn = FAIL;
	}

	return ucRtn;
}

unsigned char Mod_MipiRx_VidChk_Stable(void)
{
	lt9211_write(pdata->client,0xff, 0x86);
	if((lt9211_read(pdata->client,0x40) & 0x01) == 0x01) {
		PRINT_DEG("%s SUCCESS\n",__func__);
		return SUCCESS;
	} else {
		PRINT_DEG("%s FAIL\n",__func__);
		return false;
	}

	return SUCCESS;
}

void Drv_MipiRxClk_Sel(void)
{
	/* CLK sel */
	lt9211_write(pdata->client,0xff,0x85);
	lt9211_write(pdata->client,0xe9,0x88); //sys clk sel from XTAL
	lt9211_write(pdata->client,0xff,0x81);
	lt9211_write(pdata->client,0x80,0x51); //[7:6]rx sram rd clk src sel ad dessc pcr clk
	//[5:4]rx sram wr clk src sel mlrx bytr clk
	//[1:0]video check clk sel from desscpll pix clk
#if MIPIRX_PORT_SEL == PORTA
	lt9211_write(pdata->client,0x81,0x10); //[5]0: mlrx byte clock select from ad_mlrxa_byte_clk
	//[4]1: rx output pixel clock select from ad_desscpll_pix_clk
#elif MIPIRX_PORT_SEL == PORTB
	lt9211_write(pdata->client,0x81,0x30); //[5]1: mlrx byte clock select from ad_mlrxb_byte_clk
	//[4]1: rx output pixel clock select from ad_desscpll_pix_clk
#endif
	lt9211_write(pdata->client,0xff,0x86);
	lt9211_write(pdata->client,0x32,0x03); //video check frame cnt set: 3 frame
}

static int lt9211_parse_dt(struct device *dev, struct lt9211_data *pdata)
{
	int ret = 0;
	struct device_node *np = dev->of_node;

	pdata->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(pdata->supply)) {
		dev_err(dev, "power requtest failed.\n");
		//return PTR_ERR(pdata->supply);
	} else {
		pdata->power_invert =
			of_property_read_bool(np, "power-invert");
	}

	pdata->pwr_gpio = of_get_named_gpio(np, "power-gpio", 0);
	if(!gpio_is_valid(pdata->pwr_gpio)) {
		dev_err(dev, "No valid pwr gpio");
		// return -1;
	}
	pdata->rst_gpio = of_get_named_gpio(np, "reset-gpio", 0);
	if(!gpio_is_valid(pdata->rst_gpio)) {
		dev_err(dev, "No valid rst gpio");
		//return -1;
	} else if(gpio_is_valid(pdata->rst_gpio)) {
		gpio_direction_output(pdata->rst_gpio, false);
	}

	ret = of_property_read_u32(np, "lontium,mipi_lane", &pdata->mipi_lane);
	if (ret) {
		dev_err(dev, "Parse mipi_lane failed");
		return -1;
	}
	ret = of_property_read_u32(np, "lontium,format", &pdata->format);
	if (ret) {
		dev_err(dev, "Parse format failed");
		return -1;
	}
	ret = of_property_read_u32(np, "lontium,test", &pdata->test);
	if (ret) {
		dev_err(dev, "Parse test failed");
		return -1;
	}
	ret = of_property_read_u32(np, "lontium,hact", &pdata->hact);
	if (ret) {
		dev_err(dev, "Parse hact failed");
		return -1;
	}
	ret = of_property_read_u32(np, "lontium,vact", &pdata->vact);
	if (ret) {
		dev_err(dev, "Parse vact failed");
		return -1;
	}
	ret = of_property_read_u32(np, "lontium,hbp", &pdata->hbp);
	if (ret) {
		dev_err(dev, "Parse hbp failed");
		return -1;
	}
	ret = of_property_read_u32(np, "lontium,hfp", &pdata->hfp);
	if (ret) {
		dev_err(dev, "Parse hfp failed");
		return -1;
	}
	ret = of_property_read_u32(np, "lontium,hs", &pdata->hs);
	if (ret) {
		dev_err(dev, "Parse hs failed");
		return -1;
	}
	ret = of_property_read_u32(np, "lontium,vbp", &pdata->vbp);
	if (ret) {
		dev_err(dev, "Parse vbp failed");
		return -1;
	}
	ret = of_property_read_u32(np, "lontium,vfp", &pdata->vfp);
	if (ret) {
		dev_err(dev, "Parse vfp failed");
		return -1;
	}
	ret = of_property_read_u32(np, "lontium,vs", &pdata->vs);
	if (ret) {
		dev_err(dev, "Parse vs failed");
		return -1;
	}
	ret = of_property_read_u32(np, "lontium,pclk", &pdata->pclk);
	if (ret) {
		dev_err(dev, "Parse pclk failed");
		return -1;
	} else {
		pdata->pclk = pdata->pclk/1000; // 1khz
	}
	pdata->htotal = pdata->hact + pdata->hbp + pdata->hfp + pdata->hs;
	pdata->vtotal = pdata->vact + pdata->vbp + pdata->vfp + pdata->vs;
	return 0;
}

static int lt9211_request_io_port(struct lt9211_data *pdata)
{
	int ret = 0;
	if(gpio_is_valid(pdata->pwr_gpio)) {
		ret = gpio_request(pdata->pwr_gpio, "pdata_pwr");
		if(ret < 0) {
			dev_err(&pdata->client->dev,
					"Failed to request GPIO:%d, ERRNO:%d\n",
					(s32)pdata->pwr_gpio, ret);
			return -ENODEV;
		}

		dev_info(&pdata->client->dev, "Success request pwr-gpio\n");
	}
	if(gpio_is_valid(pdata->rst_gpio)) {
		ret = gpio_request(pdata->rst_gpio, "pdata_rst");
		if(ret < 0) {
			dev_err(&pdata->client->dev,
					"Failed to request GPIO:%d, ERRNO:%d\n",
					(s32)pdata->rst_gpio, ret);
			if(gpio_is_valid(pdata->pwr_gpio))
				gpio_free(pdata->pwr_gpio);
			return -ENODEV;
		}

		dev_info(&pdata->client->dev,  "Success request rst-gpio\n");
	}
	return 0;
}

void lt9211_init_config(void)
{
	int ret = -1, i = 0;
in_init:
	Drv_MipiRx_PhyPowerOn();
	Drv_MipiRxClk_Sel();
	Drv_System_VidChkClk_SrcSel(MLRX_BYTE_CLK);
	Drv_System_VidChk_SrcSel(MIPIDEBUG);
	Drv_SystemActRx_Sel(MIPIRX);
	Mod_MipiRxDig_Set();

STATE_CHIPRX_VIDTIMING_CONFIG:
	for(i=0; i<5; i++) {
		if(Drv_MipiRx_VidTiming_Get() == SUCCESS) {//MIPIrx的数据格式 核对
			g_stChipRx.ucRxFormat = Drv_MipiRx_VidFmt_Get(g_stMipiRxVidTiming_Get.ucFmt);
			if (Drv_MipiRx_VidTiming_Sel() == SUCCESS) {
				break;
			} else {
				PRINT_DEG("No Video Timing Matched\n");
				goto in_init;
			}
		}
		if(i == 5) {
			PRINT_DEG("%s error\n", __func__);
			return ;
		}
	}
	Drv_MipiRx_DesscPll_Set();
	if(Drv_MipiRx_PcrCali() == SUCCESS)	{
		PRINT_DEG("LT9211 pcr stable\n");
		Drv_System_VidChkClk_SrcSel(DESSCPLL_PIX_CLK);
		Drv_System_VidChk_SrcSel(MIPIRX);
		for(i=0; i<5; i++) {
			if (SUCCESS == Mod_MipiRx_VidChk_Stable()) {
				PRINT_DEG("Video Check Stable\n");
				break;
			}
			usleep_range(50*1000, 100*1000);
			if(i == 5) {
				PRINT_DEG("%s error\n", __func__);
				return ;
			}
		}

	} else {
		goto STATE_CHIPRX_VIDTIMING_CONFIG;
	}
	Drv_SystemTxSram_Sel();
	Drv_LvdsTxPhy_Poweron();

//out_init
	for(i=0; i<5; i++) {
		Mod_LvdsTxPll_RefPixClk_Get();
		Drv_LvdsTxPll_RefPixClk_Set();
		Drv_LvdsTxPll_Config();
		ret = Drv_LvdsTxPll_Cali();
		if(SUCCESS == Drv_LvdsTxPll_Cali())
			break;
		if(i == 5) {
			PRINT_DEG("%s error\n", __func__);
			return ;
		}
	}
	Drv_VidChkAll_Get(&g_stVidChk);
	Mod_LvdsTxDig_Set();
	//gpio_direction_output(pdata->rst_gpio,1);
	printk("%s succeed\n",__func__);

	return ;
}
EXPORT_SYMBOL_GPL(lt9211_init_config);

static int lt9211_probe(struct i2c_client * client, const struct i2c_device_id * id)
{
	int ret = -1;

	PRINT_DEG("LT9211 I2C Address: 0x%02x\n", client->addr);
	//g_stVidChk申请空间

	//lt9211 复位低100ms高100ms
	if(!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "Failed check I2C functionality");
		return -ENODEV;
	}
	pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
	if(pdata == NULL) {
		dev_err(&client->dev, "Failed alloc pdata memory");
		return -ENOMEM;
	}

	if(client->dev.of_node) {
		ret = lt9211_parse_dt(&client->dev, pdata);
		if(ret < 0) {
			dev_err(&client->dev, "Failed parse lt9211\n");
			goto exit_free_client_data;
		}
	}
	pdata->client = client;
	i2c_set_clientdata(client, pdata);
	ret = lt9211_request_io_port(pdata);
	if(ret < 0) {
		dev_err(&client->dev, "Failed request IO port\n");
		//goto exit_free_client_data;
	}

	if (pdata->power_invert) {
		if (regulator_is_enabled(pdata->supply) > 0)
			regulator_disable(pdata->supply);
	} else {
		ret = regulator_enable(pdata->supply);
		if (ret < 0)
			return ret;
	}

	if(gpio_is_valid(pdata->rst_gpio)) {
		gpio_direction_output(pdata->rst_gpio, true);
	}
	mdelay(5);

#ifdef LT9211_READ_ID
	//读ID, 验证IIC正常
	ret = lt9211_read_ID();
	if(ret) {
		PRINT_DEG("can not match lt9211\n");
		goto exit_free_io_port;
	}

	//设置mipi为输入
in_init:
	Drv_MipiRx_PhyPowerOn();
	Drv_MipiRxClk_Sel();
	Drv_System_VidChkClk_SrcSel(MLRX_BYTE_CLK);
	Drv_System_VidChk_SrcSel(MIPIDEBUG);
	Drv_SystemActRx_Sel(MIPIRX);
	Mod_MipiRxDig_Set();
//STATE_CHIPRX_VIDTIMING_CONFIG
STATE_CHIPRX_VIDTIMING_CONFIG:
	if(Drv_MipiRx_VidTiming_Get() == SUCCESS) {//MIPIrx的数据格式 核对
		g_stChipRx.ucRxFormat = Drv_MipiRx_VidFmt_Get(g_stMipiRxVidTiming_Get.ucFmt);
		if (Drv_MipiRx_VidTiming_Sel() == SUCCESS) {
		} else {
			PRINT_DEG("No Video Timing Matched\n");
			goto in_init;
		}
	} else {
		PRINT_DEG("Drv_MipiRx_VidTiming_Get error\n");
		goto STATE_CHIPRX_VIDTIMING_CONFIG;
	}
	Drv_MipiRx_DesscPll_Set();
	if(Drv_MipiRx_PcrCali() == SUCCESS)	{
		PRINT_DEG("LT9211 pcr stable\n");
		Drv_System_VidChkClk_SrcSel(DESSCPLL_PIX_CLK);
		Drv_System_VidChk_SrcSel(MIPIRX);
STATE_CHIPRX_VIDEO_CHECK:
		if (Mod_MipiRx_VidChk_Stable() == SUCCESS) {
			PRINT_DEG("Video Check Stable\n");
			//g_stChipRx.pHdmiRxNotify(MIPIRX_VIDEO_ON_EVENT);
		} else {
			usleep_range(50*1000, 100*1000);
			goto STATE_CHIPRX_VIDEO_CHECK;
		}
	} else {
		goto STATE_CHIPRX_VIDTIMING_CONFIG;
	}
	//设置模式
	//1. 设置等待模式
	//Mod_LvdsTx_StateHandler
	//开启lvds
	//Drv_SystemTxSram_Sel
	Drv_SystemTxSram_Sel();
	//Drv_LvdsTxPhy_Poweron
	Drv_LvdsTxPhy_Poweron();
	//开启LVDS的供电
	//gpio_direction_output(pdata->pwr_gpio,1);
out_init:
	//Mod_LvdsTxPll_RefPixClk_Get
	Mod_LvdsTxPll_RefPixClk_Get();
	//Drv_LvdsTxPll_RefPixClk_Set
	Drv_LvdsTxPll_RefPixClk_Set();
	//Drv_LvdsTxPll_Config
	Drv_LvdsTxPll_Config();
	//检测输出是否正常Drv_LvdsTxPll_Cali
	ret = Drv_LvdsTxPll_Cali();
	//ret < 1 goto Mod_LvdsTxPll_RefPixClk_Get
	if(ret != SUCCESS)
		goto out_init;
	// ret == 1 正常
	//Drv_VidChkAll_Get
	Drv_VidChkAll_Get(&g_stVidChk);
	//Mod_LvdsTxDig_Set
	Mod_LvdsTxDig_Set();
	//拉高lvds 的复位脚
	//gpio_direction_output(pdata->rst_gpio,1);
#endif
	printk("%s succeed\n",__func__);

	return 0;

#ifdef LT9211_READ_ID
exit_free_io_port:
	if(gpio_is_valid(pdata->rst_gpio))
		gpio_free(pdata->rst_gpio);
	if(gpio_is_valid(pdata->pwr_gpio))
		gpio_free(pdata->pwr_gpio);
#endif
exit_free_client_data:
	devm_kfree(&client->dev, pdata);
	i2c_set_clientdata(client, NULL);
	printk("%s failed\n",__func__);

	return 0;
}

static int lt9211_remove(struct i2c_client * client)
{
	struct lt9211_data *lt9211 = i2c_get_clientdata(client);

	if(gpio_is_valid(lt9211->rst_gpio))
		gpio_free(lt9211->rst_gpio);
	if(gpio_is_valid(lt9211->pwr_gpio))
		gpio_free(lt9211->pwr_gpio);

	return 0;
}

static const struct of_device_id lt9211_match_table[] = {
	{.compatible = "lontium,lt9211",},
	{ },
};

static const struct i2c_device_id lt9211_device_id[] = {
	{ lt9211_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_PM_SLEEP
static int  lt9211_pm_suspend(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	struct lt9211_data* lt9211 = dev_get_drvdata(dev);
	int err;

	// lt9211_reset(0);
	if(gpio_is_valid(lt9211->rst_gpio))
		gpio_direction_output(lt9211->rst_gpio, false);
	if(gpio_is_valid(lt9211->pwr_gpio))
		gpio_direction_output(lt9211->rst_gpio, false);

	if (lt9211->power_invert) {
		if (!regulator_is_enabled(lt9211->supply)) {
			err = regulator_enable(lt9211->supply);
			if (err < 0)
				return err;
		}
	} else {
		regulator_disable(lt9211->supply);
	}

	return 0;
}

static int lt9211_pm_resume(struct device *dev)
{
	//struct i2c_client *client = to_i2c_client(dev);
	struct lt9211_data* lt9211 = dev_get_drvdata(dev);
	int err;

	if(gpio_is_valid(lt9211->rst_gpio))
		gpio_direction_output(lt9211->rst_gpio, true);
	if(gpio_is_valid(lt9211->pwr_gpio))
		gpio_direction_output(lt9211->rst_gpio, true);

	if (lt9211->power_invert) {
		if (regulator_is_enabled(lt9211->supply) > 0)
			regulator_disable(lt9211->supply);
	} else {
		err = regulator_enable(lt9211->supply);
		if (err < 0)
			return err;
	}
	msleep(20);

	//lt9211_init_config();

	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(lt9211_pm_ops,
						 lt9211_pm_suspend, lt9211_pm_resume);
static struct i2c_driver lt9211_driver = {
	.probe	  = lt9211_probe,
	.remove	 = lt9211_remove,
	.id_table   = lt9211_device_id,
	.driver = {
		.name	 = lt9211_I2C_NAME,
		.owner	= THIS_MODULE,
		.pm = &lt9211_pm_ops,
		.of_match_table = lt9211_match_table,
	},
};

static int __init lt9211_init(void)
{
	s32 ret;
	PRINT_DEG("Lontium lt9211 driver installing....\n\n");
	ret = i2c_add_driver(&lt9211_driver);
	return ret;
}

static void __exit lt9211_exit(void)
{
	PRINT_DEG("Lontium lt9211 driver exited\n\n");
	i2c_del_driver(&lt9211_driver);
}

fs_initcall(lt9211_init);
module_exit(lt9211_exit);
MODULE_DESCRIPTION("Lontium lt9211 Driver\n");
MODULE_LICENSE("GPL V2\n");
