/* diag_export.c —— 见 diag_export.h。只放"被别人读、自己不读别人"的诊断状态。 */
#include "diag_export.h"
#include <string.h>

/* 带 magic 常量初值 ⇒ 落在 .data(而非 .bss), 内容确定, SWD 扫描必然能命中。 */
spi_dbg_t spi_dbg = { SPI_DBG_MAGIC0, SPI_DBG_MAGIC1,
                      0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u };

lnk_diag_t lnk_diag;

uint8_t g_boot_override = 0u;


void spi_dbg_clear(void)
{
    spi_dbg.magic0 = SPI_DBG_MAGIC0;
    spi_dbg.magic1 = SPI_DBG_MAGIC1;
    spi_dbg.clk_boot = 0u;
    spi_dbg.scan_count_m = 0u;
    spi_dbg.ms_tick_m = 0u;
    spi_dbg.stage = 0u;
    spi_dbg.clk_set_cnt = 0u;
    spi_dbg.clk_set_last = 0u;
    spi_dbg.clk_now = 0u;
    spi_dbg.setparam_cmd = 0u;
    spi_dbg.snap_pub = 0u;
}

/* 链路计数整体归零(启动时与 spi_dbg_clear 成对调用)。 */
void lnk_diag_clear(void)
{
    memset(&lnk_diag, 0, sizeof(lnk_diag));
}
