/* ─────────────────────────────────────────────────────────────
 * AM5729 Cortex-M4 IPU1 — Linker Script (memory.x)
 * ─────────────────────────────────────────────────────────────
 *
 * AM5729 IPU1 메모리 맵 (AMMU 기본 설정 기준):
 *
 *   IPU1 내부 RAM:
 *     물리: 0x55020000 (64KB)
 *     논리: 0x00000000 (AMMU가 0x00000000 → 0x55020000 변환)
 *
 *   OCMC RAM1 (On-Chip Memory Controller):
 *     물리: 0x40300000 (512KB)
 *     논리: AMMU large page로 매핑 필요
 *
 *   L3 RAM (공유 메모리, rpmsg IPC용):
 *     물리: 0x40500000 (1MB, OCMC RAM3)
 *
 *   페리페럴 영역:
 *     L4_PER: 0x48000000 ~ 0x48FFFFFF (GPIO, UART, I2C, Timer 등)
 *     L4_CFG: 0x4A000000 ~ 0x4AFFFFFF (시스템 설정)
 *
 * 참고: TRM SPRUHZ6 — Chapter 2: Memory Map
 * ───────────────────────────────────────────────────────────── */

MEMORY
{
    /* IPU1 내부 RAM — 코드 + 벡터 테이블 */
    /* AMMU 기본 매핑: 논리 0x00000000 → 물리 0x55020000 */
    IPU_RAM (rwx)  : ORIGIN = 0x00000000, LENGTH = 64K

    /* OCMC RAM1 — 확장 코드/데이터 영역 */
    /* AMMU large page 설정 후 사용 */
    OCMC1 (rwx)    : ORIGIN = 0x40300000, LENGTH = 512K

    /* 공유 메모리 — rpmsg IPC 버퍼 (A15 ↔ M4) */
    /* remoteproc resource table에서 carve-out으로 할당 */
    IPC_SHARED (rw) : ORIGIN = 0x9F000000, LENGTH = 1M

    /* Resource Table — remoteproc이 파싱하는 영역 */
    RESOURCE_TABLE (r) : ORIGIN = 0x000E0000, LENGTH = 4K
}

/* 섹션 배치 */
SECTIONS
{
    /* 벡터 테이블 (IPU RAM 시작) */
    .vector_table ORIGIN(IPU_RAM) :
    {
        KEEP(*(.vector_table));
        KEEP(*(.vector_table.exceptions));
        KEEP(*(.vector_table.interrupts));
    } > IPU_RAM

    /* 코드 */
    .text :
    {
        *(.text .text.*);
    } > IPU_RAM

    /* 읽기 전용 데이터 */
    .rodata :
    {
        *(.rodata .rodata.*);
    } > IPU_RAM

    /* 초기화된 데이터 */
    .data :
    {
        *(.data .data.*);
    } > IPU_RAM

    /* BSS (제로 초기화) */
    .bss (NOLOAD) :
    {
        *(.bss .bss.*);
        *(COMMON);
    } > IPU_RAM

    /* 스택 (IPU RAM 끝에서 아래로) */
    .stack (NOLOAD) :
    {
        . = ALIGN(8);
        _stack_end = .;
        . += 4K;  /* 4KB 메인 스택 */
        _stack_start = .;
    } > IPU_RAM

    /* remoteproc Resource Table */
    .resource_table :
    {
        KEEP(*(.resource_table));
    } > RESOURCE_TABLE

    /* rpmsg 공유 버퍼 */
    .ipc_shared (NOLOAD) :
    {
        *(.ipc_shared);
    } > IPC_SHARED

    /* Trace 버퍼 (remoteproc 디버그 로그) */
    .trace_buf (NOLOAD) :
    {
        *(.trace_buf);
    } > IPU_RAM

    /* OCMC1에 배치할 대용량 데이터/버퍼 */
    .ocmc_data (NOLOAD) :
    {
        *(.ocmc_data);
    } > OCMC1
}

/* cortex-m-rt 필요 심볼 */
_stack_start = ORIGIN(IPU_RAM) + LENGTH(IPU_RAM);
