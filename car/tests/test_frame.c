/*
 * ============================================================================
 * test_frame.c —— 统一帧协议层的「电脑端」单元测试
 * ============================================================================
 *
 * 【为什么要有这个文件】协议层不碰硬件，所以能在 PC 上用 gcc 直接编译跑。
 *   一块板子都不用接，就能验证：校验和算得对不对、帧拼得对不对、
 *   参数超量程会不会被挡住、灌垃圾字节会不会把程序搞崩。
 *   这比"烧进板子拿串口助手手敲"快得多，也更能证明代码真的能跑。
 *
 * 【规范定义】这叫 host-side unit test / 依赖注入测试：
 *   用两个假的（mock）回调函数顶替真实的「发串口」与「动电机」，
 *   只测被测模块自己的逻辑。IEEE 829 / ISTQB 里属于单元测试层级。
 *
 * 编译（在 car/ 目录下）：
 *   gcc -std=c99 -Wall -Wextra -I. car_frame.c tests/test_frame.c -o build/test_frame.exe
 *   ./build/test_frame.exe
 *
 * 校验和全部**硬编码**，是先用独立脚本算出来的 —— 这样测的是
 * 「协议层与既定帧格式是否对齐」，而不是「两个地方犯同一个错还互相印证」。
 */

#include "car_frame.h"
#include <stdio.h>
#include <string.h>

/* ---------------- 假硬件 ---------------- */
static char          g_reply[160];   /* 假串口：把应答存这儿 */
static car_command_t g_cmd;          /* 假电机：记住最后一次命令 */
static int           g_cmd_cnt;
static int           g_fail;

static void mock_send(const char *frame_with_ck)
{
    /* strncpy 而不是 strcpy：定长缓冲，防止测试自己越界 */
    strncpy(g_reply, frame_with_ck, sizeof(g_reply) - 1);
    g_reply[sizeof(g_reply) - 1] = '\0';
}

static void mock_on_cmd(const car_command_t *cmd)
{
    g_cmd = *cmd;                     /* 结构体整体赋值，C 里可以直接这么写 */
    g_cmd_cnt++;
}

static void feed_str(const char *s)
{
    while (*s) {
        car_frame_feed_byte((uint8_t)*s);
        s++;
    }
}

static void reset_mocks(void)
{
    g_reply[0] = '\0';
    g_cmd_cnt = 0;
    memset(&g_cmd, 0, sizeof(g_cmd));
    car_frame_init(mock_on_cmd, mock_send);   /* 每例重新注册，状态机也复位 */
}

/* ---------------- 一条用例 ---------------- */
static void expect(const char *name,
                   const char *frame,
                   const char *want_reply,
                   int want_cmd_cnt,
                   car_action_t want_act,
                   uint32_t want_arg1,
                   int want_has2,
                   uint32_t want_arg2)
{
    int ok = 1;

    reset_mocks();
    feed_str(frame);

    if (strcmp(g_reply, want_reply) != 0) {
        printf("  [回包不符] 期望 %-16s 实际 %s\n", want_reply, g_reply);
        ok = 0;
    }
    if (g_cmd_cnt != want_cmd_cnt) {
        printf("  [回调次数] 期望 %d 实际 %d\n", want_cmd_cnt, g_cmd_cnt);
        ok = 0;
    }
    if (want_cmd_cnt > 0) {
        if (g_cmd.action != want_act) {
            printf("  [动作不符] 期望 %d 实际 %d\n", (int)want_act, (int)g_cmd.action);
            ok = 0;
        }
        if (g_cmd.arg1 != want_arg1) {
            printf("  [arg1不符] 期望 %lu 实际 %lu\n",
                   (unsigned long)want_arg1, (unsigned long)g_cmd.arg1);
            ok = 0;
        }
        if ((int)g_cmd.has_arg2 != want_has2 || (want_has2 && g_cmd.arg2 != want_arg2)) {
            printf("  [arg2不符] 期望 has=%d val=%lu 实际 has=%d val=%lu\n",
                   want_has2, (unsigned long)want_arg2,
                   (int)g_cmd.has_arg2, (unsigned long)g_cmd.arg2);
            ok = 0;
        }
    }

    printf("%s %-34s %s\n", ok ? "[PASS]" : "[FAIL]", name, ok ? "" : "<-- 看上面");
    if (!ok) g_fail++;
}

/* 生成一串超长 payload，测缓冲区上限 */
static void expect_overflow(void)
{
    char frame[128];
    int i;

    reset_mocks();
    strcpy(frame, "$");
    for (i = 0; i < 70; i++) strcat(frame, "A");   /* 70 > CAR_FRAME_MAX_LEN=64 */
    strcat(frame, "*00\r\n");
    feed_str(frame);
    /* 超长时协议层立刻回 ERR,1 并复位，不应崩溃、也不该执行命令 */
    if (strcmp(g_reply, "$ERR,1*58\r\n") == 0 && g_cmd_cnt == 0) {
        printf("[PASS] %-34s\n", "超长帧被拦下（ERR,1）");
    } else {
        printf("[FAIL] %-34s reply=%s cnt=%d\n", "超长帧被拦下（ERR,1）",
               g_reply, g_cmd_cnt);
        g_fail++;
    }
}

int main(void)
{
    printf("=== car_frame.c host 单元测试 ===\n\n");

    /* 1. 正常帧：$CAR,FWD,800*3D —— 文档里的验收判据「发一帧，车前进 0.8s 停」 */
    expect("$CAR,FWD,800*3D 正常前进",
           "$CAR,FWD,800*3D\r\n", "$OK*04\r\n", 1, CAR_ACT_FWD, 800U, 0, 0U);

    /* 2. 立即停车 */
    expect("$CAR,STOP*64 立即停",
           "$CAR,STOP*64\r\n", "$OK*04\r\n", 1, CAR_ACT_STOP, 0U, 0, 0U);

    /* 3. PING：两个节点都认的通用命令 */
    expect("$PING*10 探活",
           "$PING*10\r\n", "$OK,PONG*3E\r\n", 1, CAR_ACT_PING, 0U, 0, 0U);

    /* 4. 校验和错 —— 线上干扰 / 波特率不对的典型症状 */
    expect("校验和错 -> ERR,1",
           "$CAR,FWD,800*00\r\n", "$ERR,1*58\r\n", 0, CAR_ACT_UNKNOWN, 0U, 0, 0U);

    /* 5. 别的节点前缀发到小车 -> ERR,2 */
    expect("未知命令 -> ERR,2",
           "$FOO,BAR*3B\r\n", "$ERR,2*5B\r\n", 0, CAR_ACT_UNKNOWN, 0U, 0, 0U);

    /* 6. 缺参数 -> ERR,3 */
    expect("缺时长参数 -> ERR,3",
           "$CAR,FWD*29\r\n", "$ERR,3*5A\r\n", 0, CAR_ACT_UNKNOWN, 0U, 0, 0U);

    /* 7. 时长超量程：本地仲裁闸门，防止上位机让车一直跑 */
    expect("时长 99999ms 被仲裁拦下",
           "$CAR,FWD,99999*3C\r\n", "$ERR,3*5A\r\n", 0, CAR_ACT_UNKNOWN, 0U, 0, 0U);

    /* 8. 带第二参数（速度 PWM） */
    expect("$CAR,FWD,800,900 带速度",
           "$CAR,FWD,800,900*28\r\n", "$OK*04\r\n", 1, CAR_ACT_FWD, 800U, 1, 900U);

    /* 9/10. SPD 量程 */
    expect("$CAR,SPD,850 设速度",
           "$CAR,SPD,850*2A\r\n", "$OK*04\r\n", 1, CAR_ACT_SPD, 850U, 0, 0U);
    expect("SPD 1000 超 999 被拦下",
           "$CAR,SPD,1000*16\r\n", "$ERR,3*5A\r\n", 0, CAR_ACT_UNKNOWN, 0U, 0, 0U);

    /* 11/12. MODE 量程 */
    expect("$CAR,MODE,1 切避障",
           "$CAR,MODE,1*62\r\n", "$OK*04\r\n", 1, CAR_ACT_MODE, 1U, 0, 0U);
    expect("MODE 9 越界被拦下",
           "$CAR,MODE,9*6A\r\n", "$ERR,3*5A\r\n", 0, CAR_ACT_UNKNOWN, 0U, 0, 0U);

    /* 13. 帧外垃圾字节不应产生任何应答 */
    expect("帧外噪声被忽略",
           "\r\nxyz\r\n", "", 0, CAR_ACT_UNKNOWN, 0U, 0, 0U);

    /* 14. 超长帧 */
    expect_overflow();

    printf("\n=== 结果：%s（失败 %d 项）===\n", g_fail ? "不通过" : "全部通过", g_fail);
    return g_fail ? 1 : 0;
}
