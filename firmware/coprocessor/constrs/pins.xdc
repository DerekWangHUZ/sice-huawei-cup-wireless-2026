################################################################################
# 约束文件 : pins.xdc
# 工程     : coprocessor (Artix-7 xc7a35tftg256-1)
# 说明     : 所有引脚均取自 docs/FPGA实验板资料参考.md，各段标注出处章节。
#            端口名与 top.v 严格一一对应。
################################################################################

##================================================================
## 1. 时钟：50MHz 晶振接 D4（资料 §2）
##================================================================
set_property PACKAGE_PIN D4 [get_ports clk]
set_property IOSTANDARD  LVCMOS33 [get_ports clk]
create_clock -period 20.000 -name clk [get_ports clk]

##================================================================
## 2. 复位：低有效，接 T9。板上无外部上拉，必须开内部 PULLUP，
##    否则设计一直卡在复位态、LED 全灭（资料 §2 实测记录）
##================================================================
set_property PACKAGE_PIN T9 [get_ports rst_n]
set_property IOSTANDARD  LVCMOS33 [get_ports rst_n]
set_property PULLUP true [get_ports rst_n]

##================================================================
## 3. UART：rx 接 S3 的 TX
##    实测勘误（2026-10-16）：课程资料 §4 写的 F12/F13 在本板接的是
##    板上 USB-UART 桥（板右侧 UART USB 口），排针上没有 F13。
##    S3 对接改用扩展排母空闲脚 B15（右排母上排第 1 脚，同排末位即 GND）。
##    rxd = B15（FPGA 接收，来自 ESP32-S3 GPIO17）
##    txd = F12（板上 USB-UART 桥，本期不驱动，恒 1）
##================================================================
set_property PACKAGE_PIN B15 [get_ports rxd]
set_property IOSTANDARD  LVCMOS33 [get_ports rxd]
set_property PACKAGE_PIN F12 [get_ports txd]
set_property IOSTANDARD  LVCMOS33 [get_ports txd]

##================================================================
## 4. 蜂鸣器：L2（资料 §8，电子琴实验 XDC）
##================================================================
set_property PACKAGE_PIN L2 [get_ports buzzer]
set_property IOSTANDARD  LVCMOS33 [get_ports buzzer]

##================================================================
## 5. 数码管 7 段（资料 §7）
##    段选 seg[0..7]  （seg[7] 为 dp/第 8 段，本设计未用但保留引脚）
##    位选 an[0..5]   （资料中 dig[0..5] 与本文件 an[0..5] 为同一组引脚）
##================================================================
set_property PACKAGE_PIN P11 [get_ports {seg[0]}]
set_property PACKAGE_PIN N12 [get_ports {seg[1]}]
set_property PACKAGE_PIN L14 [get_ports {seg[2]}]
set_property PACKAGE_PIN K13 [get_ports {seg[3]}]
set_property PACKAGE_PIN K12 [get_ports {seg[4]}]
set_property PACKAGE_PIN P13 [get_ports {seg[5]}]
set_property PACKAGE_PIN M14 [get_ports {seg[6]}]
set_property PACKAGE_PIN L13 [get_ports {seg[7]}]

set_property PACKAGE_PIN G12 [get_ports {an[0]}]
set_property PACKAGE_PIN H13 [get_ports {an[1]}]
set_property PACKAGE_PIN M12 [get_ports {an[2]}]
set_property PACKAGE_PIN N13 [get_ports {an[3]}]
set_property PACKAGE_PIN N14 [get_ports {an[4]}]
set_property PACKAGE_PIN N11 [get_ports {an[5]}]

##================================================================
## 6. LED 16 颗，高电平点亮（资料 §3）
##    注意：led[15]=M1 在该板物理不亮，属板卡个体硬件问题，验收忽略。
##================================================================
set_property PACKAGE_PIN P9  [get_ports {led[0]}]
set_property PACKAGE_PIN R8  [get_ports {led[1]}]
set_property PACKAGE_PIN R7  [get_ports {led[2]}]
set_property PACKAGE_PIN T5  [get_ports {led[3]}]
set_property PACKAGE_PIN N6  [get_ports {led[4]}]
set_property PACKAGE_PIN T4  [get_ports {led[5]}]
set_property PACKAGE_PIN T3  [get_ports {led[6]}]
set_property PACKAGE_PIN T2  [get_ports {led[7]}]
set_property PACKAGE_PIN R1  [get_ports {led[8]}]
set_property PACKAGE_PIN G5  [get_ports {led[9]}]
set_property PACKAGE_PIN H3  [get_ports {led[10]}]
set_property PACKAGE_PIN E3  [get_ports {led[11]}]
set_property PACKAGE_PIN P5  [get_ports {led[12]}]
set_property PACKAGE_PIN P1  [get_ports {led[13]}]
set_property PACKAGE_PIN N1  [get_ports {led[14]}]
set_property PACKAGE_PIN M1  [get_ports {led[15]}]

##================================================================
## 7. I/O 电平标准：本板全部 3.3V LVCMOS33
##================================================================
set_property IOSTANDARD LVCMOS33 [get_ports {led[15]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[14]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[13]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[12]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[11]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[10]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[9]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[8]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports {seg[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {seg[0]}]

set_property IOSTANDARD LVCMOS33 [get_ports {an[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {an[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {an[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {an[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {an[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {an[0]}]
