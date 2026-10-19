################################################################################
# 约束文件 : pins.xdc
# 工程     : coprocessor (Artix-7 xc7a35tftg256-1)
# 说明     : 所有引脚均取自 docs/FPGA实验板资料参考.md，各段标注出处章节。
#            端口名与 top.v 严格一一对应。
################################################################################

##================================================================
## 0. 配置 bank 电压（2026-10-18 补）
##
##   决定 bank 0（配置引脚所在 bank，含 SPI Flash 的 CCLK/DIN/DOUT/FCS）
##   的 I/O 电压支持。缺失时 DRC 会报 CFGBVS-1 警告。
##
##   本板 I/O 均为 LVCMOS33，配置 Flash 为 3.3V 的 N25Q32，
##   故 CFGBVS=VCCO、CONFIG_VOLTAGE=3.3。
##   （换 1.8V Flash 时改为 CFGBVS=GND / CONFIG_VOLTAGE=1.8。）
##
##   注意：rebuild.tcl 从零重建工程时不会自带这两条，必须写在 XDC 里。
##================================================================
set_property CFGBVS          VCCO [current_design]
set_property CONFIG_VOLTAGE  3.3  [current_design]

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
## 3. UART：rx 接 S3 的 TX；tx 为 FPGA->S3 上行链路
##    下行：S3 GPIO17(TX) -> FPGA B15（右扩展排母上排第 1 脚，同排末位即 GND）
##    上行：FPGA C14（右扩展排母，B15 邻位） -> S3 GPIO18（UART1 默认 RX）
##    板上 USB-UART 桥（F12）与本上行链路无关；电脑要观测上行帧需把
##    约束临时改回 F12（未接 S3 观测用，正常接 S3 时用 C14）。
##================================================================
set_property PACKAGE_PIN B15 [get_ports rxd]
set_property IOSTANDARD  LVCMOS33 [get_ports rxd]
set_property PACKAGE_PIN C14 [get_ports txd]
set_property IOSTANDARD  LVCMOS33 [get_ports txd]

##================================================================
## 3b. 拨码开关（辅助输入，仅"持久电平"型功能）
##     板上拨码丝印为 SW1~SW11（T9 那颗是复位按键，不是拨码，故 SW 编号
##     从 T8 起算）：
##       SW1 = T8   演示模式总开关（0=正常运行 1=演示模式）
##       SW2 = T7   本地硬静音（1=蜂鸣器完全静音，含 POST）
##     引脚出处：资料 §6。
##================================================================
set_property PACKAGE_PIN T8 [get_ports sw_demo]
set_property IOSTANDARD  LVCMOS33 [get_ports sw_demo]
set_property PULLUP true [get_ports sw_demo]
set_property PACKAGE_PIN T7 [get_ports sw_mute]
set_property IOSTANDARD  LVCMOS33 [get_ports sw_mute]
set_property PULLUP true [get_ports sw_mute]

##================================================================
## 3c. 4x4 矩阵键盘（主输入：页面/设置/演示场景导航）
##     引脚出处：资料 §5（电子琴实验 XDC，列线上拉惯例）。
##     行 = 输出逐行扫描（低有效）；列 = 输入 + 内部上拉（按下=低）。
##     若上板实测无响应，先把行列角色对调，再考虑换 PULLDOWN。
##================================================================
set_property PACKAGE_PIN K3  [get_ports {key_row[0]}]
set_property PACKAGE_PIN M6  [get_ports {key_row[1]}]
set_property PACKAGE_PIN P10 [get_ports {key_row[2]}]
set_property PACKAGE_PIN R10 [get_ports {key_row[3]}]
set_property PACKAGE_PIN R12 [get_ports {key_col[0]}]
set_property PACKAGE_PIN T12 [get_ports {key_col[1]}]
set_property PACKAGE_PIN R11 [get_ports {key_col[2]}]
set_property PACKAGE_PIN T10 [get_ports {key_col[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_row[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_row[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_row[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_row[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_col[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_col[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_col[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {key_col[0]}]
set_property PULLUP true [get_ports {key_col[3]}]
set_property PULLUP true [get_ports {key_col[2]}]
set_property PULLUP true [get_ports {key_col[1]}]
set_property PULLUP true [get_ports {key_col[0]}]

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
## 6. LED（16 个引脚约束，资料 §3）
##    项目要求 LED12~LED16 禁用：本工程只驱动 led[0..11]（板上主排 12 颗），
##    led[15:12] 在 RTL 内恒灭（引脚约束保留，RTL 已保证不点亮）。
##    板上另有一颗丝印 LED16 无课程引脚映射，从未被驱动。
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
