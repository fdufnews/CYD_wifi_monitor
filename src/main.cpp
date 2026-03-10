#include <Arduino.h>
#include <WiFi.h>
#include "display.h"
#if ESP32_2432S022
#include "CST820.h"
#endif

enum ViewMode { VIEW_ALL = 0, VIEW_SSID = 1 };
ViewMode gView = VIEW_ALL;
ViewMode prev_gView = VIEW_SSID;

// cached scan results (2.4 GHz)
static const int CH_MIN = 1, CH_MAX = 13;
static int    gChCount[CH_MAX + 1] = {0};
static double gChWeight[CH_MAX + 1] = {0.0};
static int    gLastN = 0;

// Persistent SSID cache with timestamps
struct SsidItem { 
  String ssid; 
  int32_t rssi; 
  int32_t ch; 
  uint32_t lastSeen;
  bool active;
};

static const int MAX_CACHED_SSIDS = 100;  
static SsidItem gSsidItems[MAX_CACHED_SSIDS];
static int gSsidCount = 0;  

// async scan state
static uint32_t gLastScanMs = 0;
static const uint32_t SCAN_INTERVAL_MS = 1500;
static bool gIsScanning = false;
static uint32_t gLastRenderMs = 0;
static const uint32_t RENDER_INTERVAL_MS = 5000;

#if ESP32_2432S022
// Touch screen
#define I2C_SDA 21
#define I2C_SCL 22
CST820 touch(I2C_SDA, I2C_SCL, -1, -1);
#endif

// Screen update tracking
static uint32_t gLastDataHash = 0;
static bool gDataChanged = false;
static uint32_t gLastRedrawMs = 0;

// Forward declarations
void renderCurrentView();
int findSsidInCache(const String& ssid, int32_t ch);
void updateSsidCache(const String& ssid, int32_t rssi, int32_t ch, uint32_t timestamp);
uint32_t calculateDataHash();

// Helper functions
int findSsidInCache(const String& ssid, int32_t ch) {
  for (int i = 0; i < gSsidCount; i++) {
    if (gSsidItems[i].ssid == ssid && gSsidItems[i].ch == ch) {
      return i;
    }
  }
  return -1;
}

void updateSsidCache(const String& ssid, int32_t rssi, int32_t ch, uint32_t timestamp) {
  int idx = findSsidInCache(ssid, ch);
  
  if (idx >= 0) {
    // Update existing entry
    gSsidItems[idx].rssi = rssi;
    gSsidItems[idx].lastSeen = timestamp;
    gSsidItems[idx].active = true;
  } else {
    // Add new entry if room
    if (gSsidCount < MAX_CACHED_SSIDS) {
      idx = gSsidCount++;
      gSsidItems[idx].ssid = ssid;
      gSsidItems[idx].rssi = rssi;
      gSsidItems[idx].ch = ch;
      gSsidItems[idx].lastSeen = timestamp;
      gSsidItems[idx].active = true;
    }
  }
}

uint32_t calculateDataHash() {
  uint32_t hash = 0;
  
  for (int ch = CH_MIN; ch <= CH_MAX; ++ch) {
    hash = hash * 31 + gChCount[ch];
    hash = hash * 31 + (uint32_t)((gChWeight[ch] + 5) / 10) * 10;
  }
  
  // Hash active SSID data (only recently seen ones)
  uint32_t cutoffTime = millis() - 30000;
  int activeCount = 0;

  String activeSsids[MAX_CACHED_SSIDS];
  int activeRssis[MAX_CACHED_SSIDS];
  
  for (int i = 0; i < gSsidCount; i++) {
    if (gSsidItems[i].lastSeen > cutoffTime) {
      activeSsids[activeCount] = gSsidItems[i].ssid;
      // Round RSSI to nearest 5 dBm to reduce noise
      activeRssis[activeCount] = ((gSsidItems[i].rssi + 2) / 5) * 5;
      activeCount++;
    }
  }
  
  // Simple sorting active networks
  for (int i = 0; i < activeCount - 1; i++) {
    for (int j = 0; j < activeCount - i - 1; j++) {
      if (activeSsids[j] > activeSsids[j + 1]) {
        String tempStr = activeSsids[j];
        activeSsids[j] = activeSsids[j + 1];
        activeSsids[j + 1] = tempStr;
        
        int tempRssi = activeRssis[j];
        activeRssis[j] = activeRssis[j + 1];
        activeRssis[j + 1] = tempRssi;
      }
    }
  }
  
  for (int i = 0; i < activeCount; i++) {
    hash = hash * 31 + activeSsids[i].length();
    hash = hash * 31 + activeRssis[i];
  }
  hash = hash * 31 + activeCount;
  
  return hash;
}

// -------Scanning---------

void startAsyncScan() 
{
  if (!gIsScanning) 
  {
    WiFi.scanNetworks(/*async=*/true, /*show_hidden=*/true,/*passive*/false,/*ms_per_ch*/110);
    gIsScanning = true;
    gLastScanMs = millis();
  }
}

void pollScanAndTally() 
{
  int n = WiFi.scanComplete();         // -1 scanning, -2 idle, >=0 done

  if (n == -1) return;                 // still scanning
  if (n == -2) 
  {                       
    if (!gIsScanning && millis() - gLastScanMs >= SCAN_INTERVAL_MS) 
    {
      WiFi.scanNetworks(true, true);   // start scan
      gIsScanning = true;
      gLastScanMs = millis();
    }
    return;
  }

  uint32_t scanTime = millis();
  int oldSsidCount = gSsidCount;  // Remember how many we had before

  for (int ch = CH_MIN; ch <= CH_MAX; ++ch) 
  {
    gChCount[ch]  = 0;
    gChWeight[ch] = 0.0;
  }

  // Mark all as inactive for this scan
  for (int i = 0; i < gSsidCount; i++) {
    gSsidItems[i].active = false;
  }

  // Process scan results
  for (int i = 0; i < n; ++i) 
  {
    String ssid = WiFi.SSID(i);
    int32_t rssi = WiFi.RSSI(i);
    int32_t chan = WiFi.channel(i);
    
    // Update SSID cache
    updateSsidCache(ssid, rssi, chan, scanTime);
    
    // Tally channel usage
    if (chan >= CH_MIN && chan <= CH_MAX) 
    {
      gChCount[chan] += 1;
      int w = 100 + (int)rssi; if (w < 0) w = 0;
      gChWeight[chan] += w;
    }
  }

  gLastN = n;
  WiFi.scanDelete();
  gIsScanning = false;

  // Redraw on new networks
  if ((gSsidCount > oldSsidCount) || (gLastRenderMs - millis() > RENDER_INTERVAL_MS)) {
    gLastRenderMs = millis();
    Serial.printf("New networks found- Cache: %d -> %d\n\r", oldSsidCount, gSsidCount);
    renderCurrentView();
  }
}

void doScanAndTally() 
{
  int n = WiFi.scanNetworks(false, true);   // blocking is fine here
  // reset tallies
  for (int ch = CH_MIN; ch <= CH_MAX; ++ch) {
    gChCount[ch] = 0;
    gChWeight[ch] = 0.0;
  }
  if (n > 0) {
    for (int i = 0; i < n; ++i) {
      int32_t rssi = WiFi.RSSI(i);
      int32_t chan = WiFi.channel(i);
      if (chan >= CH_MIN && chan <= CH_MAX) {
        gChCount[chan] += 1;
        int w = 100 + (int)rssi; if (w < 0) w = 0;
        gChWeight[chan] += w;
      }
    }
  }
  gLastN = n;
  gLastScanMs = millis();
}

// ------- LAYOUT ---------

int colorList[] = {TFT_BLACK, TFT_GREEN, TFT_YELLOW, TFT_ORANGE, TFT_RED};

int16_t rssiToColor(uint32_t rssi){
  int16_t color;
  if (rssi <= -90){
    color = TFT_RED;
  } else if ((rssi > -90) && (rssi <= -80)){
    color = TFT_ORANGE;
  } else if ((rssi > -80) && (rssi <= -70)){
    color = TFT_YELLOW;
  } else {
    color = TFT_GREEN;
  }
  return color;
}

void drawBar(double value, double maxValue, int width = 40) 
{
  int n = 0;
  if (maxValue > 0) 
  {
    n = (int)((value / maxValue) * width + 0.5);
    if (n > width) n = width;
  }
  for (int i = 0; i < n; ++i) Serial.print('#');
  for (int i = n; i < width; ++i) Serial.print('-');
}

void drawSsidFeed(int n)
{
  Serial.printf("Found %d networks \n\r", n);
    for (int i = 0; i < n; i++)
    {
      String ssid = WiFi.SSID(i);
      int32_t rssi = WiFi.RSSI(i);
      int32_t chan  = WiFi.channel(i);
      wifi_auth_mode_t enc = (wifi_auth_mode_t)WiFi.encryptionType(i);

      Serial.printf("%2d) ch%-2ld  %-32s  RSSI:%4ld dBm  sec:%d\n\r", i+1, (long)chan, ssid.c_str(), (long)rssi, (int)enc); //ssid list
    }
}

void drawAllChannelsBars(const int* chCount, const double* chWeight, int chMin, int chMax, int width = 40) 
{
  // find the max weight for scaling
  double maxW = 0.0;
  for (int ch = chMin; ch <= chMax; ++ch) {
    if (chWeight[ch] > maxW) maxW = chWeight[ch];
  }

  Serial.println("\nChannel usage (2.4 GHz) — bars = RSSI-weighted congestion:");
  for (int ch = chMin; ch <= chMax; ++ch) {
    Serial.printf("  ch%-2d : %2d APs | weight:%7.1f | ", ch, chCount[ch], chWeight[ch]);
    drawBar(chWeight[ch], maxW, width);
    Serial.println();
  }
}

void drawAllChannelsLCD(LGFX_CYD& lcd, const int* chCount, const double* chWeight, int chMin, int chMax)
{
  // --- layout ---
  const int W = lcd.width();
  const int H = lcd.height();
  const int L = 6;    // left margin
  const int T = 25;   // top margin (leave room for title)
  const int R = 6;    // right margin
  const int rowH = 16;                 // row height per channel
  const int rowSpace = 2;     // space between rows
  const int barW = W - L - R - 110;    // pixels reserved for bar (rest for labels)

  // find max for scaling
  double maxW = 0.0;
  for (int ch = chMin; ch <= chMax; ++ch) 
  {
    if (chWeight[ch] > maxW) maxW = chWeight[ch];
  }
  if (maxW <= 0) maxW = 1.0; // avoid divide by zero

  // clear and title
  lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setCursor(L, 2);
  lcd.print("2.4 GHz: Congestion");

  // rows
  lcd.setTextSize(1);
  int labelOffset = (rowH - lcd.fontHeight()) >> 1;  // to center the label in the bar height
  for (int ch = chMin; ch <= chMax; ++ch) 
  {
    int y = T + (ch - chMin) * rowH;

    lcd.fillRect(0, y, rowH - rowSpace, L, TFT_BLACK);

    // label
    lcd.setCursor(L, y + labelOffset);
    lcd.setTextColor(TFT_WHITE, TFT_BLACK);
    lcd.printf("CH%-2d - %2d", ch, chCount[ch]);

    // bar
    int filled = (int)((chWeight[ch] / maxW) * barW + 0.5);
    if (filled < 0) filled = 0;
    if (filled > barW) filled = barW;

    int bx = L + 110;
    int by = y - (rowSpace >> 1);
    int bh = rowH - rowSpace;

    int color = colorList[min(chCount[ch], 4)];
    lcd.fillRect(bx, by, barW, bh, TFT_DARKGREY);
    lcd.fillRect(bx, by, filled, bh, color);
  }
}

// Draw strongest SSIDs on the LCD (using cached data)
void drawSsidLCD(LGFX_CYD& lcd) 
{
  const int maxShow = 13;  // Show fewer to make room for cache bar
  int show = (gSsidCount < maxShow) ? gSsidCount : maxShow;

  // Clear screen and draw title
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  lcd.setTextSize(2);
  lcd.setCursor(6, 2);
  lcd.print("WiFi Networks");

  // --- CACHE BAR ---
  const int L = 6;
  const int W = lcd.width();
  lcd.setTextSize(1);
  lcd.setTextColor(TFT_WHITE, TFT_BLACK);
  lcd.setCursor(L, 22);
  lcd.printf("Cache: %d/%d networks", gSsidCount, MAX_CACHED_SSIDS);
  
  // Cache fill bar
  int cacheBarY = 34;
  int cacheBarW = W - L - 6;
  int cacheBarH = 4;
  float cacheFillPercent = (float)gSsidCount / MAX_CACHED_SSIDS;
  int cacheFilled = (int)(cacheFillPercent * cacheBarW);
  
  // Background bar (dark)
  lcd.fillRect(L, cacheBarY, cacheBarW, cacheBarH, TFT_DARKGREY);
  
  // Filled portion
  uint16_t cacheColor;
  if (cacheFillPercent < 0.5) {
    cacheColor = TFT_GREEN;        // Green < 50%
  } else if (cacheFillPercent < 0.8) {
    cacheColor = TFT_YELLOW;       // Yellow 50-80
  } else {
    cacheColor = TFT_RED;          // Red > 80
  }
  
  if (cacheFilled > 0) {
    lcd.fillRect(L, cacheBarY, cacheFilled, cacheBarH, cacheColor);
  }

  // --- NETWORK LIST ---
  lcd.setTextSize(1);
  int y = 45;
  int displayed = 0;
  int colorRssi;

  for (int i = 0; i < gSsidCount && displayed < maxShow; i++) 
  {
    const SsidItem& network = gSsidItems[i];
    
    // Trim long SSID names
    String name = network.ssid;
    if (name.length() > 30) {
      name = name.substring(0, 30) + "..";
    }

    lcd.setCursor(6, y);
    
    // Color: green for active, white for cached
    if (network.active) {
      lcd.setTextColor(TFT_GREEN, TFT_BLACK);
      colorRssi = rssiToColor(network.rssi);
      lcd.print("*");
    } else {
      lcd.setTextColor(TFT_WHITE, TFT_BLACK);
      colorRssi = TFT_WHITE;
      lcd.print(" ");
    }
    
    lcd.printf("%2d) ch%-2d ", displayed + 1, network.ch);
    lcd.setTextColor(colorRssi, TFT_BLACK);
    lcd.printf("%4ddBm ", network.rssi);
    lcd.setTextColor(TFT_GREEN, TFT_BLACK);
    lcd.print(name);
    
    y += 14;
    displayed++;
  }

  // Footer (if there's room)
  if (y < lcd.height() - 12) {
    lcd.setTextColor(TFT_DARKGREY, TFT_BLACK);
    lcd.setCursor(6, lcd.height() - 12);
    lcd.printf("Showing %d of %d", show, gSsidCount);
  }
}

void renderCurrentView() 
{
  if (prev_gView != gView){
      lcd.fillScreen(TFT_BLACK);
  }
  if (gView == VIEW_ALL) 
  {
    drawAllChannelsLCD(lcd, gChCount, gChWeight, CH_MIN, CH_MAX);
    prev_gView = gView;
  } 
  else 
  { 
    drawSsidLCD(lcd);
    prev_gView = gView;
  }
}

// --------- MAIN LOOP AND SETUP -------------

void setup() 
{
  Serial.begin(115200);
  delay(300);
  WiFi.mode(WIFI_STA);     // station mode (don’t create an AP)
  WiFi.disconnect(true);   // ensure we’re not connected while scanning
  startAsyncScan();
  delay(200);

  // --- LCD smoke test ---
  lcd.init();            // power up panel
  lcd.setRotation(1);    // try 1 (landscape). Try 0/2/3 if orientation is weird.
  lcd.setColorDepth(16);
  lcd.setBrightness(200);// 0..255 (if BL pin is correct)
  
  #if ESP32_2432S022
  // touch screen initialization
  touch.begin();
  #endif
}

void loop() 
{
  // --- input: tap or 't' toggles immediately ---
  static bool wasDown = false;
  static uint32_t lastToggle = 0;
  #if ESP32DEV
  int32_t tx, ty;
  bool down = lcd.getTouch(&tx, &ty);
  #elif ESP32_2432S022
  uint16_t tx, ty;
  uint8_t gesture;
  bool down = touch.getTouch(&tx, &ty, &gesture);
  #endif

  if (Serial.available() > 0) 
  {
    char c = Serial.read();
    if (c == 't' || c == 'T') 
    { 
      down = false; wasDown = true; 
    } // force a toggle path
  }
  uint32_t now = millis();
  if (wasDown && !down && (now - lastToggle) > 250) {
    gView = (gView == VIEW_ALL) ? VIEW_SSID : VIEW_ALL;
    lastToggle = now;
    
    Serial.printf("View changed to: %s\n\r", gView == VIEW_ALL ? "CHANNELS" : "SSIDS");
    renderCurrentView();              // <- redraw right away for view change
    gLastRedrawMs = now;
  }
  wasDown = down;

  // --- periodic scan (non-blocking cadence) ---
  pollScanAndTally();

  delay(0);
}
