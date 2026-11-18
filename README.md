# Add AirPrint support to your old printer, using just a cheap ESP32-S3

> [!IMPORTANT]
> This project is actively looking for contributors to add support for more printers. Most importantly, you need to really own the printer you add: every driver must be tested on the real hardware, not only in emulation. See [Adding a printer](#adding-a-printer) to get started.


Turns any USB-only printers into a wireless printer. An ESP32-S3 board plugs into the printer's USB port and joins your WiFi:

- **Print** from iPhone, iPad and Mac (AirPrint), Android (Mopria) and Linux, with no drivers or apps.
- **A web app** on your phone or computer: print PDFs and pictures, keep the files in the browser, and put pages together (rotate, crop, ID card copies) into a PDF.

![](./docs/IMG_3577.jpeg)

| | |
|---|---|
| ![](./docs/screenshot-printer.png) | ![](./docs/screenshot-files.png) |
| ![](./docs/screenshot-process.png) | |

## Supported printers

| Series | Models |
|---|---|
| 🟢 MG2500 | MG2550S |
| 🟡 MG2400 | MG2410 to MG2490 |
| 🟡 MG2500 | MG2510 to MG2590 (other than the MG2500S) |
| 🟡 P200 | P200 |
| 🟡 MG2900 | MG2910 to MG2990 |
| 🟡 MG3000 | MG3010 to MG3090 |
| 🟡 E470 | E470 to E478 |
| 🟡 E460 | E460 to E468 |
| 🟡 iP2800 | iP2810 to iP2899 |
| 🟡 E3100 | E3170 to E3195 |
| 🟡 TS3100 | TS3110 to TS3195 |
| 🟡 MP230 | MP230 to MP237 |
| 🟡 iP2700 | iP2700 to iP2788 |
| 🟡 MP250, MP270, MP280 | MP250 to MP288 |
| 🟡 MP490, MP493, MP495 | MP490 to MP499 |
| 🟡 MX340, MX350 | MX340 to MX358 |
| 🟡 MX360, MX410, MX420 | MX360 to MX428 |
| 🟢 LBP3300 | LBP3300 (black and white laser, CAPT): A4, one- and two-sided |

- 🟢 Validated on real printer
- 🟡 Expected to work, not tested on real hardware yet.
- The LBP3300 speaks a different protocol (Canon CAPT), implemented from the open-source captdriver and from [thientranvan/esp32_printer_canon_3300](https://github.com/thientranvan/esp32_printer_canon_3300), which printed on this model. Its driver is GPLv3 (see License). Tested on a real printer: A4 on plain paper, one-sided and two-sided (long and short edge). Not tried: Letter, other paper sources, toner and paper status.

The firmware recognises the printer when it is plugged in. Other printers? Please open an issue or a PR.

## What you need

Just one thing: An ESP32-S3 dev board with two USB-C ports:

![](./docs/amazon.png)

Optionally: a SSD1315 OLED screen and a 3D-printed case, see later in this tutorial

## Setup

**Step 1: Hardware.** Plug the ESP32-S3 into your computer via the COM port on the ESP32 board (NOT the port labelled "USB")

![](./docs/IMG_3538.jpeg)

**Step 2: Settings.** Build once (`pio run`): this creates `.env`. Edit it:
   ```
   WIFI_SSID=your-network
   WIFI_PASSWORD=your-password
   PRINTER_NAME=Canon MG2500 (ESP32)
   HOSTNAME=canon-mg2500
   MODE=debug
   ```
   `MODE=debug` lets you update the board over WiFi later (see below); `MODE=normal` does without. `.env` is git-ignored, so your WiFi password stays local.

**Step 3: Flash.** Connect the board's **COM** port to your computer and run `pio run -t upload`. PlatformIO finds the board's serial port; if several are connected, add `--upload-port /dev/cu.usbmodem…`.

## Using it

Plug the printer to the port labelled "USB"

NOTE: You may also need to bridge the "USB-OTG" solder join on the board, so that it supplies 5V to the printer. While the printer doesn't actively use 5V, it sense the USB plug using this signal.

- **The web app:** open `http://canon-mg2500.local/` (your `HOSTNAME`) on your phone or computer.
- **The button on the board** (BOOT, or one on GPIO4): press twice for a test page, hold 2 s to cancel.
- **Print from a phone or a Mac:** on an iPhone, Share → Print and pick the printer. On a Mac, add it in System Settings → Printers & Scanners.

## Adding OLED screen and buttons

This step is **optional**, you can connect a SSD1315 OLED screen with the pinout below:

| OLED | ESP32-S3 |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SCL | GPIO41 |
| SDA | GPIO42 |

Connect it before powering on: the board only looks for it at startup.

![](./docs/IMG_3568.jpeg)

For buttons, wire each one between a pin and GND:

| Button | ESP32-S3 | Action |
|---|---|---|
| 1 | GPIO4 | Same as BOOT (test page, cancel) |
| 2 | GPIO5 | Not assigned: add your own in `on_user_button()` (`src/main.c`) |

While the screen is off, a press only wakes it up.

## Development

Set `MODE=dev` and flash: print jobs are processed but not printed.

On the computer, without the board: `make -C host && cd host && ./airprint_host`, then open http://localhost:8631/ (an emulated printer). To work on the web app with live reloading: `cd web && npm run dev` (it talks to that server; `DEVICE=http://<board IP> npm run dev` talks to the board).

Technical details are in [AGENTS.md](AGENTS.md).

## Adding a printer

Each printer model is a driver in `components/printers/`, listed in `printers.c`; the firmware picks the one of the printer that is plugged in. A Canon PIXMA whose stream is close to the MG2500's is one line in `canon_pixma.c`; another printer is a new file in the same folder. See [AGENTS.md](AGENTS.md) for details.

## License

MIT, see [LICENSE](LICENSE), except for the CAPT laser printer driver (`components/printers/canon_capt.c`, `capt.c`, `hiscoa.c`, `capt.h`, `hiscoa.h` and `host/capt_host.c`): it is ported from [captdriver](https://github.com/mounaiban/captdriver) (Copyright (C) 2013 Alexey Galakhov and contributors) and licensed under the GNU GPL version 3 or later, see [COPYING-GPL3](COPYING-GPL3). A firmware built with those files is distributed under the GPLv3; MIT code can be combined with it. Without them (remove `canon_capt.c`, `capt.c`, `hiscoa.c` and the `canon_lbp3300_driver` line in `printers.c`) the firmware is plain MIT.

The printer protocol follows [Gutenprint](https://gimp-print.sourceforge.io/). The web UI bundles [Svelte](https://svelte.dev/) (MIT) and [pdf.js](https://mozilla.github.io/pdf.js/) (Apache-2.0).
