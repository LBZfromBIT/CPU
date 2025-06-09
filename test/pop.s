# 基于RV32I实现五个整数的冒泡排序,实现数据从大到小排序
.data 
array: .word 4,5,3,1,2    # 待排序的整数数组

.text
main:
    # x8记录数据区基地址
    addi x8,x0,0x0

    # 初始化外层循环计数器
    addi x5,x0,0     # x5记录外层循环次数i
    addi x6,x0,4     # x6记录外层循环次数上限,五个元素进行四次浮动即可

outer_loop:
    bge x5,x6,exit_outer    # 如果外层循环计数器达到上限，跳转到结束
    addi x7,x0,0            # x7记录内层循环次数j
inner_loop:
    # 每次内层循环时，后i个元素不需要再比
    sub x28,x6,x5          # x28 = 4-i,对应内层循环的上限(由于被复用，所以重新计算)
    bge x7,x28,exit_inner  # 如果内层循环计数器达到上限，跳转到外层循环

    # arr[j]
    slli x29,x7,2        # j * 4
    add x29,x8,x29       # 基地址 + j * 4
    lw x30,0(x29)        # 读取第j个元素

    # arr[j+1]
    addi x31,x7,1        # j + 1
    slli x31,x31,2       # (j + 1) * 4
    add x31,x8,x31       # 基地址 + (j + 1) * 4
    lw x31,0(x31)        # 读取第j+1个元素(位置不够,复用一下)

    # 比较 arr[j] 和 arr[j+1]
    
    bge x30,x31,skip_swap  # 如果 arr[j] >= arr[j+1]，跳过交换

    # 交换 arr[j] 和 arr[j+1]
    sw x31,0(x29)   # arr[j] = arr[j+1]
    # 重新计算 arr[j+1] 的地址
    addi x28,x7,1        # j + 1
    slli x28,x28,2       # (j + 1) * 4
    add x28,x8,x28       # 基地址 + (j + 1) * 4
    sw x30,0(x28)        # arr[j+1] = arr[j]

skip_swap:
    addi x7,x7,1         # j++
    jal x0,inner_loop    # 跳转到内层循环开始
exit_inner:
    addi x5,x5,1         # i++
    jal x0,outer_loop    # 跳转到外层循环开始
exit_outer:
# 排序完成，程序结束
    jal x0,exit_outer
