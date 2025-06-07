# 基于Verilog HDL实现的五阶段流水线处理器

## 基本介绍

该库记录了本人基于Verilog HDL实现的五阶段流水线CPU，支持RISC-V架构的RV32I指令集中的如下指令：

a)	**R类指令**：ADD, SUB, AND, OR, XOR, SLT, SLTU, SLL, SRL, SRA
b)	**I类指令**：ADDI, SLTI, SLTIU, XORI, ORI, ANDI, SLLI, SRLI, SRAI
c)	**加载类指令**：LW, LH, LB
d)	**存储类指令**：SW, SH, SB
e)	**比较分支指令**：BEQ, BNE, BLT, BGE, BLTU, BGEU
f)	**跳转指令**：JAL

## 文件说明
```
/reference:保存使用的相关资料
|--/Verilog HDL review.pdf             Verilog HDL语言回顾
|--/RISC-V-Reader-Chinese-v2p1.pdf     中文RISC-V指导手册

/source:保存源文件
|--/ALU.v                              ALU实现
|--/Control_Unit.v                     控制单元实现
|--/CPU.v                              CPU顶层封装
|--/Hazard_and_Forwarding.v            冒险检测和数据转发实现
|--/Immediate_Generator.v              立即数获取实现
|--/Memory.v                           存储器实现
|--/PC.v                               程序计数器实现
|--/Pipeline_Register.v                流水线寄存器实现
|--/Register_File.v                    一般寄存器组实现

/test:保存测试用文件
|--/pop.s                             实现五个整数冒泡排序的汇编程序
```
