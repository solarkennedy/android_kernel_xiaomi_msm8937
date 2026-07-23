# pepito (Palm PVG100 / MSM8940) — notes for this kernel fork

LineageOS 23.2 runs this device on a **4.19** kernel while the vendor blobs, TZ
and modem firmware are the stock Android 8.1 (3.18-era) set. Most of the pepito
patches in this branch exist to bridge that gap.

## ⚠️ `[TEMP]` and `fixup!` subject lines are NOT reliable

Several commits were authored during bring-up as throwaway instrumentation and
then silently accreted load-bearing logic without the subject being updated. A
2026-07-22 cleanup pass reverted three of them on the assumption the labels were
accurate; each produced a boot failure.

**Read the actual diff before reverting anything here.** After a batch of
reverts, a pepito-touched file matching upstream base is a *red flag*, not a
clean result — base is the pre-port behavior these patches exist to correct.

### Known load-bearing, despite the label

| Area | What it does | Failure if reverted |
|---|---|---|
| `drivers/clk/msm/clock.c` | `clock_late_init` handoff denylist skipping `gcc_blsp1_uart2_apps_clk` | Stock Palm TZ keeps its own debug console on BLSP1 UART2. Dropping the bootloader handoff vote makes **TZ fire PS_HOLD** → hard reset ~6 s in, before the kernel can print. The skip was added by two commits titled `fixup! [TEMP] … log each handoff drop`, which do not add logging — they *replace* the `pr_info` with the skip. |
| `drivers/iommu/iommu.c` | `iommu_set_fault_handler()` `BUG_ON(!domain)` softened to `WARN_ON_ONCE` | pepito's SMMUs are **disabled in DTS**, so `iommu_get_domain_for_dev()` returns NULL for drivers registering fault handlers during early probe. The stock `BUG_ON` panics at boot. Camera/GPU/display working does **not** imply the SMMUs are on — they run with SMMUs off. |
| `drivers/i2c/busses/i2c-msm-v2.c` | probe-end `i2c_msm_pm_clk_disable` / `unprepare` / `clk_path_unvote` left commented out | `iface_clk` is `GCC_BLSP1_AHB_CLK`, which also clocks the BLSP1 UART console. The 0→1→0 enable/disable at probe gates the console clock off before `msm_serial` probes (earlycon holds no CCF vote). |
| `net/qrtr/qrtr.c` | `[TEMP][pepito]` synchronous HELLO reply + local server replay, replay-shaping knobs, legacy HELLO payload knob | Part of the legacy-IPC/qmux radio port, not diagnostics. Reverting risks silent modem-attach failure. |
| `drivers/soc/qcom/smsm.c` | `[TEMP][pepito]` stock-parity apps SMSM word (`0x1029`) | Same — radio bring-up handshake the stock modem expects, not scaffolding. |

## Serial console

Console, `earlycon`, `ignore_loglevel` and `sysrq_always_enabled=1` are **off by
default** in release builds. Enable them for bench work with:

    PEPITO_SERIAL_CONSOLE=true m ...

(see `device/xiaomi/mithorium-common/BoardConfigCommon.mk`). This is independent
of the clock handoff denylist above — that skip is required whether or not Linux
claims the console, because TZ owns the UART regardless.
