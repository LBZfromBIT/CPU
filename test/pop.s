# ����RV32Iʵ�����������ð������,ʵ�����ݴӴ�С����
.data 
array: .word 4,5,3,1,2    # ���������������

.text
main:
    # x8��¼����������ַ
    addi x8,x0,0x0

    # ��ʼ�����ѭ��������
    addi x5,x0,0     # x5��¼���ѭ������i
    addi x6,x0,4     # x6��¼���ѭ����������,���Ԫ�ؽ����Ĵθ�������

outer_loop:
    bge x5,x6,exit_outer    # ������ѭ���������ﵽ���ޣ���ת������
    addi x7,x0,0            # x7��¼�ڲ�ѭ������j
inner_loop:
    # ÿ���ڲ�ѭ��ʱ����i��Ԫ�ز���Ҫ�ٱ�
    sub x28,x6,x5          # x28 = 4-i,��Ӧ�ڲ�ѭ��������(���ڱ����ã��������¼���)
    bge x7,x28,exit_inner  # ����ڲ�ѭ���������ﵽ���ޣ���ת�����ѭ��

    # arr[j]
    slli x29,x7,2        # j * 4
    add x29,x8,x29       # ����ַ + j * 4
    lw x30,0(x29)        # ��ȡ��j��Ԫ��

    # arr[j+1]
    addi x31,x7,1        # j + 1
    slli x31,x31,2       # (j + 1) * 4
    add x31,x8,x31       # ����ַ + (j + 1) * 4
    lw x31,0(x31)        # ��ȡ��j+1��Ԫ��(λ�ò���,����һ��)

    # �Ƚ� arr[j] �� arr[j+1]
    
    bge x30,x31,skip_swap  # ��� arr[j] >= arr[j+1]����������

    # ���� arr[j] �� arr[j+1]
    sw x31,0(x29)   # arr[j] = arr[j+1]
    # ���¼��� arr[j+1] �ĵ�ַ
    addi x28,x7,1        # j + 1
    slli x28,x28,2       # (j + 1) * 4
    add x28,x8,x28       # ����ַ + (j + 1) * 4
    sw x30,0(x28)        # arr[j+1] = arr[j]

skip_swap:
    addi x7,x7,1         # j++
    jal x0,inner_loop    # ��ת���ڲ�ѭ����ʼ
exit_inner:
    addi x5,x5,1         # i++
    jal x0,outer_loop    # ��ת�����ѭ����ʼ
exit_outer:
# ������ɣ��������
    jal x0,exit_outer
