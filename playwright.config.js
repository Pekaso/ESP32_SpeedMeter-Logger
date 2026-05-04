const { defineConfig } = require("@playwright/test");

module.exports = defineConfig({
  testDir: "./test",
  testMatch: "**/*.spec.js",
  reporter: "list",
  use: {
    browserName: "chromium",
    viewport: { width: 1280, height: 900 },
    launchOptions: {
      executablePath: "/snap/bin/chromium",
      args: ["--no-sandbox"]
    }
  }
});
