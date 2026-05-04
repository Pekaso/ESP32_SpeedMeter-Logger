const path = require("path");
const fs = require("fs");
const { expect, test } = require("@playwright/test");

const root = path.resolve(__dirname, "..");
const viewerUrl = `file://${path.join(root, "tool", "log_viewer.html")}`;
const dumpPath = path.join(root, "tool", "dump.txt");
const dumpText = fs.readFileSync(dumpPath, "utf8");

test("loads SCUB hex dump and links table, detail, and chart", async ({ page }) => {
  await page.goto(viewerUrl);

  await page.locator("#fileInput").setInputFiles(dumpPath);

  await expect(page.locator("#statusLine")).toContainText("Full binary log");
  await expect(page.locator("#statusLine")).toContainText("records 206");
  await expect(page.locator("#summaryGrid")).toContainText("Summary block");
  await expect(page.locator("#summaryGrid")).toContainText("Yes");
  await expect(page.locator("#summaryGrid")).toContainText("206 / 0");
  await expect(page.locator("#exportBtn")).toBeEnabled();

  const rows = page.locator("#recordBody tr[data-index]");
  await expect(rows).toHaveCount(206);
  await expect(rows.first()).toContainText("154");
  await expect(rows.first()).toContainText("0.000s");

  await rows.nth(1).click();
  await expect(page.locator("#detailRecord")).toContainText("index: 1");
  await expect(page.locator("#detailApp")).toContainText("data_type: 0x01");

  const hasDrawnPixels = await page.locator("#chart").evaluate((canvas) => {
    const ctx = canvas.getContext("2d");
    const { width, height } = canvas;
    const data = ctx.getImageData(0, 0, width, height).data;
    for (let i = 0; i < data.length; i += 4) {
      if (data[i] !== 255 || data[i + 1] !== 255 || data[i + 2] !== 255) {
        return true;
      }
    }
    return false;
  });
  expect(hasDrawnPixels).toBeTruthy();
});

test("parses pasted payload line and supports warnings filter", async ({ page }) => {
  await page.goto(viewerUrl);

  const payload = [
    "RX ok: seq=41 sender=0x0001 appCounter=41 txMillis=42437 rssi=-78 dBm",
    "Payload: E2 01 00 01 00 00 00 29 00 00 A5 C5 00 29 01 01 02 6F 00 30 FF 80 00 80 00 00 00 12 34"
  ].join("\n");
  await page.locator("#pasteInput").fill(payload);
  await page.locator("#parseBtn").click();

  await expect(page.locator("#statusLine")).toContainText("Serial Payload lines");
  await expect(page.locator("#recordBody tr[data-index]")).toHaveCount(1);
  await expect(page.locator("#recordBody")).toContainText("62.3");
  await expect(page.locator("#recordBody")).toContainText("4800");

  await page.locator("#warningsOnly").check();
  await expect(page.locator("#recordBody tr[data-index]")).toHaveCount(1);
  await expect(page.locator("#detailWarn")).toContainText("Transport CRC16 mismatch");
});

test("requests LOGDUMP over Web Serial and parses received SCUB_HEX text", async ({ page }) => {
  await page.addInitScript((scubDump) => {
    const encoder = new TextEncoder();
    const chunks = [
      "LOGDUMP\r\n",
      scubDump.slice(0, Math.floor(scubDump.length / 2)),
      scubDump.slice(Math.floor(scubDump.length / 2))
    ].map((text) => encoder.encode(text));

    window.__serialWrites = [];
    window.__serialOpenOptions = null;
    window.__serialClosed = false;

    const port = {
      async open(options) {
        window.__serialOpenOptions = options;
      },
      async close() {
        window.__serialClosed = true;
      },
      readable: {
        getReader() {
          let index = 0;
          return {
            async read() {
              if (index >= chunks.length) return { done: true };
              return { value: chunks[index++], done: false };
            },
            async cancel() {},
            releaseLock() {}
          };
        }
      },
      writable: {
        getWriter() {
          return {
            async write(value) {
              window.__serialWrites.push(new TextDecoder().decode(value));
            },
            releaseLock() {}
          };
        }
      }
    };

    Object.defineProperty(navigator, "serial", {
      configurable: true,
      value: {
        async requestPort() {
          return port;
        }
      }
    });
  }, dumpText);

  await page.goto(viewerUrl);

  await expect(page.locator("#statusLine")).toContainText("serial disconnected");
  await page.locator("#serialConnectBtn").click();
  await expect(page.locator("#statusLine")).toContainText("serial connected");

  await page.locator("#logDumpBtn").click();
  await expect(page.locator("#statusLine")).toContainText("Full binary log");
  await expect(page.locator("#statusLine")).toContainText("records 206");
  await expect(page.locator("#statusLine")).toContainText("LOGDUMP received");
  await expect(page.locator("#pasteInput")).toHaveValue(/SCUB_HEX_BEGIN/);
  await expect(page.locator("#pasteInput")).toHaveValue(/SCUB_HEX_END/);

  await expect(page.locator("#recordBody tr[data-index]")).toHaveCount(206);
  await expect(page.locator("#exportBtn")).toBeEnabled();

  const serialState = await page.evaluate(() => ({
    writes: window.__serialWrites,
    openOptions: window.__serialOpenOptions
  }));
  expect(serialState.writes).toEqual(["LOGDUMP\n"]);
  expect(serialState.openOptions).toEqual({ baudRate: 115200 });
});
