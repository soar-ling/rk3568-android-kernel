#!/usr/bin/env python
# SPDX-License-Identifier: (GPL-2.0+ OR MIT)
# Copyright (c) 2018 Fuzhou Rockchip Electronics Co., Ltd
#


"""
Multiple dtb package tool

Usage: scripts/mkmultidtb.py board
The board is what you defined in DTBS dictionary like DTBS['board'],
Such as: PX30-EVB, RK3308-EVB

"""
import os
import sys
import shutil
from collections import OrderedDict

DTBS = {}

DTBS['PX30-EVB'] = OrderedDict([('px30-evb-ddr3-v10', '#_saradc_ch0=1024'),
				('px30-evb-ddr3-lvds-v10', '#_saradc_ch0=512')])

DTBS['RK3308-EVB'] = OrderedDict([('rk3308-evb-dmic-i2s-v10', '#_saradc_ch3=288'),
				  ('rk3308-evb-dmic-pdm-v10', '#_saradc_ch3=1024'),
				  ('rk3308-evb-amic-v10', '#_saradc_ch3=407')])

DTBS['RK3568-S0'] = OrderedDict([('rk3568-iot-lp4x-s1-v10-lvds-rk628-gvi', '#_saradc_ch1=1023#_saradc_ch3=1023'),
				  ('rk3568-iot-lp4x-s1-v11-lvds-rk628-gvi', '#_saradc_ch3=1023')])

DTBS['RK3568-S1'] = OrderedDict([('rk3568-iot-lp4x-bs2-v10', '#_saradc_ch1=690'),
				  ('rk3568-iot-lp4x-s1-v12-rk628-gvi', '#_saradc_ch1=780#_saradc_ch3=1023'),
				  ('rk3568-iot-lp4x-s1-v12-lvds', '#_saradc_ch1=780#_saradc_ch3=860'),
				  ('rk3568-iot-lp4x-s1-v11-rk628-gvi', '#_saradc_ch3=1023'),
				  ('rk3568-iot-lp4x-s1-v11-lvds', '#_saradc_ch3=860')])

def main():
    if (len(sys.argv) < 2) or (sys.argv[1] == '-h'):
        print __doc__
        sys.exit(2)

    BOARD = sys.argv[1]
    TARGET_DTBS = DTBS[BOARD]
    target_dtb_list = ''
    default_dtb = True

    for dtb, value in TARGET_DTBS.items():
        if default_dtb:
            ori_file = 'arch/arm64/boot/dts/rockchip/' + dtb + '.dtb'
            shutil.copyfile(ori_file, "rk-kernel.dtb")
            target_dtb_list += 'rk-kernel.dtb '
            default_dtb = False
        new_file = dtb + value + '.dtb'
        ori_file = 'arch/arm64/boot/dts/rockchip/' + dtb + '.dtb'
        shutil.copyfile(ori_file, new_file)
        target_dtb_list += ' ' + new_file

    print target_dtb_list
    os.system('scripts/resource_tool logo_270.bmp logo.bmp logo_90.bmp logo_180.bmp ' + target_dtb_list)
    os.system('rm ' + target_dtb_list)

if __name__ == '__main__':
    main()
