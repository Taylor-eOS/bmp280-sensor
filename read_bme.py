#!/usr/bin/env python3

from smbus2 import SMBus
import time

def u16(lo,hi):return (hi<<8)|lo

def s16(lo,hi):v=(hi<<8)|lo;return v-65536 if v&0x8000 else v

def read_cal(bus,addr):
    d=bus.read_i2c_block_data(addr,0x88,24)
    dig_T1=u16(d[0],d[1]);dig_T2=s16(d[2],d[3]);dig_T3=s16(d[4],d[5])
    dig_P1=u16(d[6],d[7]);dig_P2=s16(d[8],d[9]);dig_P3=s16(d[10],d[11])
    dig_P4=s16(d[12],d[13]);dig_P5=s16(d[14],d[15]);dig_P6=s16(d[16],d[17])
    dig_P7=s16(d[18],d[19]);dig_P8=s16(d[20],d[21]);dig_P9=s16(d[22],d[23])
    try:
        dig_H1=bus.read_byte_data(addr,0xA1)
        e=bus.read_i2c_block_data(addr,0xE1,7)
        dig_H2=s16(e[0],e[1]);dig_H3=e[2]
        dig_H4=(e[3]<<4)|(e[4]&0xF);dig_H5=(e[5]<<4)|(e[4]>>4)
        dig_H5=dig_H5 if dig_H5<32768 else dig_H5-65536
        dig_H6=e[6] if e[6]<128 else e[6]-256
    except Exception:
        dig_H1=dig_H2=dig_H3=dig_H4=dig_H5=dig_H6=None
    return dict(T1=dig_T1,T2=dig_T2,T3=dig_T3,P1=dig_P1,P2=dig_P2,P3=dig_P3,P4=dig_P4,P5=dig_P5,P6=dig_P6,P7=dig_P7,P8=dig_P8,P9=dig_P9,H1=dig_H1,H2=dig_H2,H3=dig_H3,H4=dig_H4,H5=dig_H5,H6=dig_H6)

def write_config(bus,addr):
    try:
        bus.write_byte_data(addr,0xF2,0x01)
        bus.write_byte_data(addr,0xF4,0x27)
        bus.write_byte_data(addr,0xF5,0xA0)
    except Exception:
        pass

def read_raw(bus,addr,is_bme):
    if is_bme:
        d=bus.read_i2c_block_data(addr,0xF7,8)
        pres=(d[0]<<12)|(d[1]<<4)|(d[2]>>4)
        temp=(d[3]<<12)|(d[4]<<4)|(d[5]>>4)
        hum=(d[6]<<8)|d[7]
        return temp,pres,hum
    else:
        d=bus.read_i2c_block_data(addr,0xF7,6)
        pres=(d[0]<<12)|(d[1]<<4)|(d[2]>>4)
        temp=(d[3]<<12)|(d[4]<<4)|(d[5]>>4)
        return temp,pres,None

def compensate(cal,raw_t,raw_p,raw_h):
    var1=(raw_t/16384.0 - cal['T1']/1024.0)*cal['T2']
    var2=((raw_t/131072.0 - cal['T1']/8192.0)**2)*cal['T3']
    t_fine=var1+var2
    temp=t_fine/5120.0
    var1=t_fine/2.0 - 64000.0
    var2=var1*var1*cal['P6']/32768.0
    var2=var2 + var1*cal['P5']*2.0
    var2=var2/4.0 + cal['P4']*65536.0
    var1=(cal['P3']*var1*var1/524288.0 + cal['P2']*var1)/524288.0
    var1=(1.0 + var1/32768.0)*cal['P1']
    pressure=1048576.0 - raw_p
    if var1!=0:
        pressure=(pressure - var2/4096.0) * 6250.0 / var1
        var1=cal['P9']*pressure*pressure/2147483648.0
        var2=pressure*cal['P8']/32768.0
        pressure=pressure + (var1 + var2 + cal['P7'])/16.0
    else:
        pressure=0
    humidity=None
    if cal['H1'] is not None and raw_h is not None:
        var_h=t_fine - 76800.0
        var_h=(raw_h - (cal['H4']*64.0 + cal['H5']/16384.0*var_h))* (cal['H2']/65536.0*(1.0 + cal['H6']/67108864.0*var_h*(1.0 + cal['H3']/67108864.0*var_h)))
        var_h=var_h*(1.0 - cal['H1']*var_h/524288.0)
        if var_h>100:var_h=100.0
        elif var_h<0:var_h=0.0
        humidity=var_h
    return temp,pressure,humidity

def find_addr(bus):
    for a in (0x76,0x77):
        try:
            cid=bus.read_byte_data(a,0xD0)
            return a,cid
        except Exception:
            pass
    return None,None

def main():
    i = 0
    with SMBus(1) as bus:
        addr,cid=find_addr(bus)
        if not addr:
            print("No BME/BMP found at 0x76/0x77")
            return
        is_bme=(cid==0x60)
        print("Found device at 0x%02X chip id 0x%02X %s" % (addr,cid,"BME280" if is_bme else "BMP280"))
        cal=read_cal(bus,addr)
        write_config(bus,addr)
        while True:
            try:
                raw_t,raw_p,raw_h=read_raw(bus,addr,is_bme)
                temp,press,hum=compensate(cal,raw_t,raw_p,raw_h)
                print("Temperature: %.2f C Pressure: %.2f hPa Humidity: %s, %.0f" % (temp, press/100.0, ("%.2f %%" % hum) if hum is not None else "N/A", i))
                i = i + 1
            except Exception as e:
                print("Read error",e)
            time.sleep(1)

if __name__=="__main__":
    main()

