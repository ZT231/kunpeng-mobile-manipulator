#include "teach.h"

/* 示教点缓冲：teach_buf[i][ch] = 第 i 个采样点第 ch 通道的目标脉宽(us) */
static uint16_t teach_buf[TEACH_MAX_POINTS][ARM_CHANNEL_COUNT];
static uint16_t teach_count = 0U;
static uint8_t  teach_recording = 0U;
static uint8_t  teach_playing = 0U;
static uint16_t teach_play_index = 0U;
static uint8_t  teach_sample_tick = 0U;
static uint8_t  teach_play_tick = 0U;

void Teach_Init(void)
{
    teach_count = 0U;
    teach_recording = 0U;
    teach_playing = 0U;
    teach_play_index = 0U;
    teach_sample_tick = 0U;
    teach_play_tick = 0U;
}

void Teach_StartRecord(void)
{
    if (teach_playing) return;          /* 回放中不允许记录 */

    teach_count = 0U;
    teach_play_index = 0U;
    teach_sample_tick = 0U;
    teach_recording = 1U;
    Arm_CommandRecord();
}

void Teach_StopRecord(void)
{
    teach_recording = 0U;
    Arm_CommandManual();
}

void Teach_StartPlayback(void)
{
    if (teach_recording) Teach_StopRecord();
    if (teach_count == 0U) return;      /* 无轨迹可播 */

    teach_play_index = 0U;
    teach_play_tick = 0U;
    teach_playing = 1U;
    Arm_CommandPlay();
}

void Teach_Clear(void)
{
    /* 仅在非记录/回放态允许清空，避免破坏进行中的轨迹 */
    if (teach_recording || teach_playing) return;
    teach_count = 0U;
    teach_play_index = 0U;
}

void Teach_Tick(void)
{
    uint8_t ch;

    /* 防御：状态机被外部改变（如急停/被 MOVE 接管）时，自动退出本模块 */
    if (teach_recording && Arm_GetState() != ARM_STATE_RECORD)   teach_recording = 0U;
    if (teach_playing   && Arm_GetState() != ARM_STATE_PLAYBACK) teach_playing = 0U;

    if (teach_recording)
    {
        if (++teach_sample_tick >= TEACH_TICKS_PER_SAMPLE)
        {
            teach_sample_tick = 0U;

            if (teach_count < TEACH_MAX_POINTS)
            {
                for (ch = 0U; ch < ARM_CHANNEL_COUNT; ch++)
                {
                    teach_buf[teach_count][ch] = Arm_GetCurrent(ch);
                }
                teach_count++;
            }
            else
            {
                /* 缓冲已满，自动停止记录并回手动 */
                teach_recording = 0U;
                Arm_CommandManual();
            }
        }
    }
    else if (teach_playing)
    {
        if (++teach_play_tick >= TEACH_TICKS_PER_SAMPLE)
        {
            teach_play_tick = 0U;

            if (teach_play_index < teach_count)
            {
                Arm_SetPlaybackTargets(teach_buf[teach_play_index]);
                teach_play_index++;
            }
            else
            {
                /* 播放完毕，回手动 */
                teach_playing = 0U;
                Arm_CommandManual();
            }
        }
    }
}

uint16_t Teach_GetPointCount(void)
{
    return teach_count;
}
