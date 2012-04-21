#ifndef __BROADCAST_T3900_H__
#define __BROADCAST_T3900_H__


void tdmb_t3900_set_userstop(void);
int tdmb_t3900_mdelay(int32 ms);
void tdmb_t3900_must_mdelay(int32 ms);
int tdmb_t3900_power_on(void);
int tdmb_t3900_power_off(void);
int tdmb_t3900_select_antenna(unsigned int sel);
int tdmb_t3900_i2c_write_burst(uint16 waddr, uint8* wdata, int length);
int tdmb_t3900_i2c_read_burst(uint16 raddr, uint8* rdata, int length);
int tdmb_t3900_i2c_write16(unsigned short reg, unsigned short val);
int tdmb_t3900_i2c_read16(uint16 reg, uint16 *ret);

#endif /* __BROADCAST_T3900_H__ */
