const { chromium } = require('playwright');
const path = require('path');

(async () => {
  const browser = await chromium.launch();
  const page = await browser.newPage({
    viewport: { width: 1280, height: 720 }
  });
  
  page.on('console', msg => console.log(`CONSOLE [${msg.type()}]: ${msg.text()}`));
  page.on('pageerror', err => console.log(`ERROR: ${err.message}`));
  
  console.log("Navigating to http://localhost:8080...");
  try {
    await page.goto('http://localhost:8080');
    await page.waitForTimeout(3000);
    
    const screenshotPath = '/Users/hekote/.gemini/antigravity/brain/b88d204b-4963-4686-9b62-09f725f1c62c/screenshot.png';
    await page.screenshot({ path: screenshotPath });
    console.log(`Screenshot saved to ${screenshotPath}`);
  } catch(e) {
    console.error("Failed:", e);
  }
  
  await browser.close();
})();
