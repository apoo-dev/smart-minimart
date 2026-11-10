/*
 * SmartCart - MASTER LOOP (organisation version, styled display)
 *
 * Shoppers use the mobile app (Blynk). The web dashboard is the STAFF console:
 * it watches every cart, manages products and customers, and shows sales.
 *
 * FLOW ON THE CART
 *   IDLE      -> animated "Tap your card" screen
 *   tap card  -> registered customer is welcomed, shopping starts
 *   scan items-> receipt builds up on the TFT, the app and the web console
 *   tap again -> payment is taken from the customer's balance in Firebase
 *   done      -> result screen, then back to IDLE for the next shopper
 *
 * FIREBASE LAYOUT (everything lives under smartcart/)
 *   catalog/<productId>/{name, price}                    managed on the web console
 *   customers/<cardUID>/{name, balance}                  managed on the web console
 *   carts/<CART_ID>/{cartStatus, customerName, customerUid, totalDue,
 *                    itemsInCart, deviceOnline, lastSeen, rssi, label,
 *                    command, receiptLines/<productId>/{...}}
 *   transactions/<pushId>/{cartId, customerUid, customerName, total,
 *                          itemCount, timestamp, items/<productId>/{name,qty,unitPrice}}
 *   events/<pushId>/{cartId, type, message, timestamp}
 *   lastUnknownCode/{code, cartId, timestamp}
 *   lastUnregisteredCard/{uid, cartId, timestamp}
 *   shelfCounters/<productId>/{taken, returned, claimed}
 *        taken / returned are counted by the SHELF module from weight changes,
 *        claimed is counted by the carts (items currently in a cart or bought)
 *   catalog/<productId>/unitWeight                       grams per item, used by the shelf
 *
 * ADD / REMOVE
 *   Scan an item to ADD it. To REMOVE one, press the remove button (or scan a
 *   "CMD:REMOVE" code), then scan the item. It only leaves the cart once the
 *   shelf confirms by weight that it was put back; otherwise it stays.
 *
 * ONE SKETCH PER CART: change CART_ID (and the Blynk token) for each board.
 * Save it inside a folder named exactly "smart_cart_master".
 *
 * ---------------------------------------------------------------
 * PIN MAP  (adjust to your actual wiring)
 * ---------------------------------------------------------------
 *   TFT ILI9341 (hardware SPI), portrait 240x320
 *     CS -> 10   DC -> 9   RST -> 8   SCK -> 13   MOSI -> 11   MISO -> 12
 *   PN532 RFID (software SPI)
 *     SCK -> 4   MISO -> 5   MOSI -> 6   SS -> 7
 *   GM861S scanner (UART1)
 *     Scanner TX -> ESP32 RX -> GPIO 17   (scanner RX -> GPIO 18, optional)
 *     115200 baud for your module. If the scanner's TX is 5 V logic, use a
 *     voltage divider before GPIO 17 - ESP32 pins are 3.3 V only.
 *
 * Libraries: Adafruit GFX (its Fonts folder is used for the nicer text),
 *            Adafruit ILI9341, Adafruit PN532, Adafruit BusIO, Blynk,
 *            Firebase Arduino Client Library for ESP8266 and ESP32
 *
 * Blynk Console events (all Info type): payment_approved, payment_declined,
 * unknown_card, unknown_item
 * ---------------------------------------------------------------
 */

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <Adafruit_PN532.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <WiFi.h>
#include <esp_system.h>

#define BLYNK_TEMPLATE_ID   "TMPLxxxxxxx"
#define BLYNK_TEMPLATE_NAME "SmartCart"
#define BLYNK_AUTH_TOKEN    "bcH2f0mc5sDIF3whg81ZEBTO_2Te6gJG"
#define BLYNK_PRINT Serial
#include <BlynkSimpleEsp32.h>

#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

// ---------------- WHICH CART IS THIS? ----------------
#define CART_ID "cart-01"          // unique per cart: cart-01, cart-02, ...

// ---------------- Add / remove items ----------------
#define REMOVE_BUTTON_PIN      14      // optional push button between this pin and GND; -1 = no button
#define REQUIRE_PICKUP_FOR_ADD 0       // 1 = refuse a scan unless the shelf saw that item being taken
#define RETURN_TIMEOUT_MS      45000   // time allowed to put a removed item back on the shelf
#define REMOVE_MODE_IDLE_MS    20000   // remove mode switches itself off after this long

const char* WIFI_SSID = "Error_404_3G";
const char* WIFI_PASS = "no hotspotsusme";

// ---------------- TFT ----------------
#define TFT_CS   10
#define TFT_DC   9
#define TFT_RST  8
#define TFT_SCK  13
#define TFT_MOSI 11
#define TFT_MISO 12
Adafruit_ILI9341 tft(TFT_CS, TFT_DC, TFT_RST);

// ---------------- PN532 (software SPI) ----------------
#define PN532_SCK   4
#define PN532_MISO  5
#define PN532_MOSI  6
#define PN532_SS    7
Adafruit_PN532 nfc(PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);

// ---------------- Scanner (UART1) ----------------
#define SCANNER_RX 17
#define SCANNER_TX 18
HardwareSerial scannerSerial(1);
String scanBuffer = "";

// ---------------- Firebase ----------------
#define API_KEY      "AIzaSyDA76GWSVKqAfodI6pdakzMbCm1tHNGr9E"
#define DATABASE_URL "https://smartcart-b9ed2-default-rtdb.firebaseio.com"
FirebaseData   fbdo;
FirebaseAuth   fbauth;
FirebaseConfig fbconfig;
const char* ROOT = "smartcart";
bool fbBegun = false;   // true once Firebase.begin() has run

// ---------------- Buzzer (payment sounds + quiet warnings) ----------------
// Passive buzzer: (+) to GPIO 15, (-) to GND.  Set BUZZER_PIN -1 if you have none.
// BUZZER_PASSIVE 1 = passive buzzer (tones, volume can be turned down).  0 = active buzzer (on/off only).
#define BUZZER_PIN      15
#define BUZZER_PASSIVE  1
#define BZ_CH           7        // LEDC channel (only used by Arduino-ESP32 core 2.x)
#define PAY_VOL_PCT     30       // payment sounds, 1..100
#define WARN_VOL_PCT    6        // warning sounds, 1..100 (small = very quiet; raise it if you cannot hear it)
#define ALERT_REMIND_MS 15000    // while the "not scanned" popup is open, a quiet reminder every 15 s

struct Note { uint16_t freq; uint16_t ms; };      // freq 0 = silence
const Note SND_PAID[]     = { {1047, 90}, {0, 30}, {1319, 90}, {0, 30}, {1568, 220} };    // happy rising tune
const Note SND_DECLINED[] = { {330, 160}, {0, 60}, {247, 340} };                           // two low falling notes
const Note SND_FAIL[]     = { {300, 120}, {0, 70}, {300, 120}, {0, 70}, {300, 120} };      // three low beeps
const Note SND_TAP[]      = { {1568, 60} };                                                // card accepted
const Note SND_WARN[]     = { {700, 60}, {0, 110}, {700, 60} };                            // soft double beep

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define BZ_ATTACH()   ledcAttach(BUZZER_PIN, 2000, 8)
  #define BZ_FREQ(f)    ledcChangeFrequency(BUZZER_PIN, (f), 8)
  #define BZ_DUTY(d)    ledcWrite(BUZZER_PIN, (d))
#else
  #define BZ_ATTACH()   do { ledcSetup(BZ_CH, 2000, 8); ledcAttachPin(BUZZER_PIN, BZ_CH); } while (0)
  #define BZ_FREQ(f)    ledcWriteTone(BZ_CH, (f))
  #define BZ_DUTY(d)    ledcWrite(BZ_CH, (d))
#endif

const Note* bzSeq = nullptr;
int bzLen = 0, bzIdx = 0;
unsigned long bzUntil = 0;
uint8_t bzVol = 10;

void buzzerRaw(int freq, uint8_t volPct) {
  if (BUZZER_PIN < 0) return;
#if BUZZER_PASSIVE
  if (freq <= 0) { BZ_DUTY(0); return; }
  BZ_FREQ(freq);
  int duty = (128 * (int)volPct) / 100;             // 128 of 255 = loudest square wave, smaller = quieter
  if (duty < 1) duty = 1;
  BZ_DUTY(duty);
#else
  digitalWrite(BUZZER_PIN, freq > 0 ? HIGH : LOW);
#endif
}

void buzzerInit() {
  if (BUZZER_PIN < 0) return;
#if BUZZER_PASSIVE
  BZ_ATTACH();
  BZ_DUTY(0);
#else
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
#endif
}

void buzzerPlay(const Note* seq, int len, uint8_t volPct) {   // starts at the next buzzerTick()
  bzSeq = seq; bzLen = len; bzIdx = 0; bzVol = volPct; bzUntil = 0;
}

void buzzerTick() {                                           // call every loop; never blocks
  if (!bzSeq) return;
  unsigned long now = millis();
  if (bzUntil != 0 && (long)(now - bzUntil) < 0) return;
  if (bzIdx >= bzLen) { buzzerRaw(0, 0); bzSeq = nullptr; return; }
  const Note& n = bzSeq[bzIdx++];
  buzzerRaw(n.freq, bzVol);
  bzUntil = now + n.ms;
  if (bzUntil == 0) bzUntil = 1;
}

#define PLAY_PAID()      buzzerPlay(SND_PAID,     sizeof(SND_PAID)     / sizeof(SND_PAID[0]),     PAY_VOL_PCT)
#define PLAY_DECLINED()  buzzerPlay(SND_DECLINED, sizeof(SND_DECLINED) / sizeof(SND_DECLINED[0]), PAY_VOL_PCT)
#define PLAY_FAIL()      buzzerPlay(SND_FAIL,     sizeof(SND_FAIL)     / sizeof(SND_FAIL[0]),     PAY_VOL_PCT)
#define PLAY_TAP()       buzzerPlay(SND_TAP,      sizeof(SND_TAP)      / sizeof(SND_TAP[0]),      PAY_VOL_PCT)
#define PLAY_WARN()      buzzerPlay(SND_WARN,     sizeof(SND_WARN)     / sizeof(SND_WARN[0]),     WARN_VOL_PCT)

bool fbReady() { return fbBegun && Firebase.ready(); }

String cartPath() { return String(ROOT) + "/carts/" + CART_ID; }

// =====================================================
//  Firebase lookups
// =====================================================

// Product: smartcart/catalog/<id>/{name,price}
// Returns:  1 = found,  0 = not in the catalog,  -1 = could not reach Firebase
int lookupProduct(const String &id, String &name, int &price) {
  String base = String(ROOT) + "/catalog/" + id;

  bool ok = false;
  for (int attempt = 1; attempt <= 2 && !ok; attempt++) {   // one retry on a dropped connection
    ok = Firebase.RTDB.getJSON(&fbdo, base);                // one call gets name and price
    if (!ok) {
      String why = fbdo.errorReason();
      if (why.indexOf("not exist") >= 0) return 0;          // Firebase answered: no such product
      Serial.println("[Firebase] lookup attempt " + String(attempt) + " failed: " + why);
      delay(500);
    }
  }
  if (!ok) return -1;                                       // network / secure-connection problem

  FirebaseJson &pj = fbdo.jsonObject();
  FirebaseJsonData pd;
  if (!pj.get(pd, "name") || pd.stringValue.length() == 0) return 0;   // empty node = not registered
  name = pd.stringValue;
  price = 0;
  if (pj.get(pd, "price")) price = pd.intValue;
  return 1;
}

// Customer: smartcart/customers/<cardUID>/{name,balance}
// Returns:  1 = found,  0 = card not registered,  -1 = could not reach Firebase
int loadCustomer(const String &uid, String &name, int &balance) {
  String path = String(ROOT) + "/customers/" + uid;

  bool ok = false;
  for (int attempt = 1; attempt <= 2 && !ok; attempt++) {
    ok = Firebase.RTDB.getJSON(&fbdo, path);
    if (!ok) {
      String why = fbdo.errorReason();
      if (why.indexOf("not exist") >= 0) return 0;
      Serial.println("[Firebase] customer lookup attempt " + String(attempt) + " failed: " + why);
      delay(500);
    }
  }
  if (!ok) return -1;

  FirebaseJson &j = fbdo.jsonObject();
  FirebaseJsonData d;
  if (!j.get(d, "name") || d.stringValue.length() == 0) return 0;   // empty node = not registered
  name = d.stringValue;
  balance = 0;
  if (j.get(d, "balance")) balance = d.intValue;
  return 1;
}

// Shelf counters: smartcart/shelfCounters/<productId>/{taken, returned, claimed}
//   taken / returned : counted by the SHELF module from weight changes
//   claimed          : counted here, items currently in a cart (or bought)
//   taken - returned - claimed = items that left the shelf but are not in any cart yet
// Returns 1 = ok, -1 = could not reach Firebase (no data yet counts as ok with zeros)
double returnedAtMs = 0;       // when the shelf last reported this product put back (server time, epoch ms)
double pendSinceMs = 0;        // when the current removal started (cart clock, epoch ms; 0 = clock not set yet)

int readCounters(const String &productId, int &taken, int &returned, int &claimed) {
  taken = 0; returned = 0; claimed = 0; returnedAtMs = 0;
  String path = String(ROOT) + "/shelfCounters/" + productId;
  if (!Firebase.RTDB.getJSON(&fbdo, path)) {
    String why = fbdo.errorReason();
    if (why.indexOf("not exist") >= 0) return 1;            // shelf has not reported this product yet
    Serial.println("[Firebase] counters read failed: " + why);
    return -1;
  }
  FirebaseJson &cj = fbdo.jsonObject();
  FirebaseJsonData cd;
  if (cj.get(cd, "taken"))    taken    = cd.intValue;
  if (cj.get(cd, "returned")) returned = cd.intValue;
  if (cj.get(cd, "claimed"))  claimed  = cd.intValue;
  if (cj.get(cd, "flagged"))  claimed += cd.intValue;     // pickups the shelf already warned about
  if (cj.get(cd, "returnedAt")) returnedAtMs = cd.doubleValue;
  return 1;
}

// +1 when an item goes into a cart, -1 when a returned item leaves it (atomic on the server)
void adjustClaimed(const String &productId, int delta) {
  if (!fbReady()) return;
  FirebaseJson cj;
  cj.set("claimed/.sv/increment", delta);
  if (!Firebase.RTDB.updateNode(&fbdo, String(ROOT) + "/shelfCounters/" + productId, &cj)) {
    Serial.println("[Firebase] claimed update failed: " + fbdo.errorReason());
  }
}

// ---------------- Cart state ----------------
struct Line { String id; String name; int price; int qty; };
Line receipt[20];
int receiptLines = 0;
int totalItems = 0;
int totalCost  = 0;

// IDLE = waiting for a card tap, SHOPPING = card tapped and items being scanned,
// LOCKED / PAID = payment in progress / done.
enum CartState { IDLE, SHOPPING, LOCKED, PAID };
CartState state = IDLE;
String customerName = "-";
String activeUid = "";                 // card UID of the current shopper
int customerBalance = 0;

// ---- add / remove mode ----
bool removeMode = false;               // true = the next scan REMOVES an item (it must go back on the shelf)
unsigned long removeModeUntil = 0;
String noticeText = "";                // short message for the shopper's app, e.g. "Put Pepsi back on the shelf"
String alertText = "";                 // short text for the app notice, e.g. "Return or scan: Milo"
unsigned long nextAlertPollMs = 0;

// Items the shelf says have been off the shelf for over a minute without a scan (the popup on the LCD).
// The shelf writes smartcart/alertSummary/list = "Name|qty|productId;Name|qty|productId"
struct AlertItem { String id; String name; int qty; };
AlertItem alertItems[4];
int alertCount = 0;
unsigned long nextAlertBeepMs = 0;
String suppId[6];                      // items just scanned here: ignore the shelf's (older) alert for them for a moment
unsigned long suppUntil[6] = {0, 0, 0, 0, 0, 0};
int suppNext = 0;

struct PendingReturn {                 // an item the shopper is putting back on the shelf
  bool active = false;
  int lineIdx = -1;
  String id;
  String name;
  int baseReturned = 0;                // shelf's "returned" counter when the removal started
  unsigned long deadline = 0;
  unsigned long nextPollAt = 0;
  int lastSecsLeft = -1;
};
PendingReturn pend;

// After a short message screen (welcome / result) the display moves on by itself:
//   mode 1 = full reset to idle (after payment), mode 2 = back to the receipt,
//   mode 3 = just redraw the idle screen
unsigned long screenReturnAt = 0;
int screenReturnMode = 0;

unsigned long lastPollMs = 0;

// Display bookkeeping
bool receiptVisible = false;           // true while the receipt screen is what's on the TFT
int flashIdx = -1;                     // receipt line that is briefly highlighted after a scan
unsigned long flashUntil = 0;

// =====================================================
//  TFT: colours, text helpers, screens, animations
// =====================================================
const uint16_t C_BG      = 0x0884;     // deep navy
const uint16_t C_CARD    = 0x1908;     // panel blue
const uint16_t C_ACCENT  = 0x06B5;     // teal
const uint16_t C_ACCENT2 = 0x04AF;     // dimmer teal
const uint16_t C_ACCENT3 = 0x02C9;     // dimmest teal
const uint16_t C_HILITE  = 0x02CA;     // row highlight after a scan
const uint16_t C_TEXT    = 0xFFFF;
const uint16_t C_MUTED   = 0xADB9;     // soft grey-blue
const uint16_t C_GOLD    = 0xFD84;
const uint16_t C_GREEN   = 0x2E6E;
const uint16_t C_RED     = 0xE267;

// Receipt list layout (portrait 240 x 320)
const int ROW_Y0   = 72;               // first row top
const int ROW_H    = 32;               // row height
const int MAX_ROWS = 6;                // rows that fit above the total bar

int textWidth(const GFXfont* f, const String &s) {
  int16_t x1, y1; uint16_t w, h;
  tft.setFont(f);
  tft.setTextSize(1);
  tft.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  return (int)w;
}

// Shortens text with ".." until it fits in maxW pixels
String fitText(const GFXfont* f, String s, int maxW) {
  if (textWidth(f, s) <= maxW) return s;
  while (s.length() > 1 && textWidth(f, s + "..") > maxW) s.remove(s.length() - 1);
  return s + "..";
}

// Custom fonts are positioned by their BASELINE (y = bottom of the letters)
void drawTxt(const GFXfont* f, uint16_t color, int x, int baseline, const String &s) {
  tft.setFont(f);
  tft.setTextSize(1);
  tft.setTextColor(color);
  tft.setCursor(x, baseline);
  tft.print(s);
}

void drawTxtCenter(const GFXfont* f, uint16_t color, int cx, int baseline, const String &s) {
  drawTxt(f, color, cx - textWidth(f, s) / 2, baseline, s);
}

void drawTxtRight(const GFXfont* f, uint16_t color, int xRight, int baseline, const String &s) {
  drawTxt(f, color, xRight - textWidth(f, s), baseline, s);
}

// Centered text that wraps onto several lines at spaces
void drawWrappedCenter(const GFXfont* f, uint16_t color, int cx, int baseline, int lineH, int maxW, const String &msg) {
  String line = "";
  int y = baseline;
  int i = 0, n = msg.length();
  while (i < n) {
    int sp = msg.indexOf(' ', i);
    if (sp < 0) sp = n;
    String word = msg.substring(i, sp);
    String trial = line.length() ? line + " " + word : word;
    if (line.length() && textWidth(f, trial) > maxW) {
      drawTxtCenter(f, color, cx, y, line);
      y += lineH;
      line = word;
    } else {
      line = trial;
    }
    i = sp + 1;
  }
  if (line.length()) drawTxtCenter(f, color, cx, y, line);
}

// A line drawn thicker and "growing" from its start, for the check / cross animation
void thickLine(int x0, int y0, int x1, int y1, uint16_t col) {
  tft.drawLine(x0, y0, x1, y1, col);
  tft.drawLine(x0 + 1, y0, x1 + 1, y1, col);
  tft.drawLine(x0, y0 + 1, x1, y1 + 1, col);
  tft.drawLine(x0 + 1, y0 + 1, x1 + 1, y1 + 1, col);
}

void drawLineGrow(int x0, int y0, int x1, int y1, uint16_t col, int steps) {
  for (int i = 1; i <= steps; i++) {
    int x = x0 + (x1 - x0) * i / steps;
    int y = y0 + (y1 - y0) * i / steps;
    thickLine(x0, y0, x, y, col);
    delay(10);
  }
}

// Circle that grows, then a tick (ok) or a cross (not ok) that draws itself. ~0.3 s.
void animateMark(int cx, int cy, uint16_t fill, bool ok) {
  for (int r = 6; r <= 34; r += 7) {
    tft.fillCircle(cx, cy, r, fill);
    delay(16);
  }
  if (ok) {
    drawLineGrow(cx - 15, cy + 1, cx - 5, cy + 11, C_TEXT, 4);
    drawLineGrow(cx - 5, cy + 11, cx + 16, cy - 11, C_TEXT, 6);
  } else {
    drawLineGrow(cx - 13, cy - 13, cx + 13, cy + 13, C_TEXT, 6);
    drawLineGrow(cx + 13, cy - 13, cx - 13, cy + 13, C_TEXT, 6);
  }
}

// One receipt row. highlight = true paints it briefly in teal (used right after a scan).
void drawItemRow(int idx, bool highlight) {
  int first = max(0, receiptLines - MAX_ROWS);
  int slot = idx - first;
  if (idx < 0 || idx >= receiptLines || slot < 0 || slot >= MAX_ROWS) return;
  if (alertCount > 0) return;                      // the "not scanned" popup covers the rows

  int y = ROW_Y0 + slot * ROW_H;
  tft.fillRect(0, y, 240, ROW_H, highlight ? C_HILITE : C_BG);
  tft.drawFastHLine(10, y + ROW_H - 1, 220, C_CARD);

  int base = y + 22;
  drawTxt(&FreeSans12pt7b, C_TEXT, 10, base, fitText(&FreeSans12pt7b, receipt[idx].name, 120));
  drawTxt(&FreeSansBold12pt7b, C_ACCENT, 136, base, "x" + String(receipt[idx].qty));
  drawTxtRight(&FreeSansBold12pt7b, C_TEXT, 232, base, String(receipt[idx].price * receipt[idx].qty));
}

// Popup over the receipt rows: items off the shelf for over a minute and not scanned
void drawAlertPopup() {
  if (alertCount <= 0) return;
  tft.fillRoundRect(8, 74, 224, 186, 10, C_CARD);
  tft.drawRoundRect(8, 74, 224, 186, 10, C_RED);
  tft.drawRoundRect(9, 75, 222, 184, 9, C_RED);
  tft.fillRoundRect(8, 74, 224, 34, 10, C_RED);
  tft.fillRect(8, 92, 224, 16, C_RED);
  drawTxtCenter(&FreeSansBold12pt7b, C_TEXT, 120, 99, "NOT SCANNED!");
  drawTxtCenter(&FreeSans9pt7b, C_MUTED, 120, 128, "Off the shelf for over 1 minute");

  int shown = (alertCount > 3) ? 3 : alertCount;
  for (int i = 0; i < shown; i++) {
    int base = 158 + i * 30;
    drawTxt(&FreeSansBold12pt7b, C_TEXT, 20, base, fitText(&FreeSansBold12pt7b, alertItems[i].name, 150));
    drawTxtRight(&FreeSansBold12pt7b, C_GOLD, 220, base, "x" + String(alertItems[i].qty));
  }
  if (alertCount > shown) drawTxtCenter(&FreeSans9pt7b, C_MUTED, 120, 238, "+" + String(alertCount - shown) + " more");
  drawTxtCenter(&FreeSans9pt7b, C_TEXT, 120, 253, "Scan it or put it back");
}

void drawReceiptScreen() {
  tft.fillScreen(C_BG);

  // title bar (turns red in REMOVE mode or while the "not scanned" popup is open)
  bool alertBar = (alertCount > 0 && !removeMode);
  tft.fillRect(0, 0, 240, 44, (removeMode || alertBar) ? C_RED : C_CARD);
  tft.fillRect(0, 44, 240, 2, (removeMode || alertBar) ? C_TEXT : C_ACCENT);
  String title = removeMode ? String("REMOVE ITEM")
                            : (alertBar ? String("CHECK YOUR ITEMS")
                                        : ((customerName != "-") ? "Hi, " + customerName : String("Smart Cart")));
  drawTxt(&FreeSansBold12pt7b, C_TEXT, 10, 30, fitText(&FreeSansBold12pt7b, title, 220));

  // column headings
  int first = max(0, receiptLines - MAX_ROWS);
  drawTxt(&FreeSans9pt7b, C_MUTED, 10, 64, "ITEM");
  if (first > 0) drawTxt(&FreeSans9pt7b, C_MUTED, 60, 64, "(+" + String(first) + " earlier)");
  drawTxt(&FreeSans9pt7b, C_MUTED, 136, 64, "QTY");
  drawTxtRight(&FreeSans9pt7b, C_MUTED, 232, 64, "Rs.");

  if (receiptLines == 0) {
    drawTxtCenter(&FreeSansBold12pt7b, C_TEXT, 120, 150, "Scan your first item");
    drawTxtCenter(&FreeSans9pt7b, C_MUTED, 120, 178, "Tap your card again when done");
  } else {
    for (int i = first; i < receiptLines; i++) drawItemRow(i, false);
  }

  // total bar
  tft.fillRect(0, 266, 240, 54, C_CARD);
  tft.fillRect(0, 266, 240, 2, C_ACCENT);
  drawTxt(&FreeSans9pt7b, C_MUTED, 10, 288, "TOTAL");
  drawTxtRight(&FreeSansBold18pt7b, C_TEXT, 232, 300, "Rs." + String(totalCost));
  drawTxt(&FreeSans9pt7b, C_MUTED, 10, 313,
          String(totalItems) + (totalItems == 1 ? " item" : " items") +
          (removeMode ? " - scan item to remove" : " - tap card to pay"));

  drawAlertPopup();                                 // does nothing when there is no alert
  receiptVisible = true;
}

// ---- idle screen with expanding "contactless" rings ----
unsigned long idleAnimAt = 0;
int idleAnimFrame = 0;
int ringPrev[3] = {-1, -1, -1};
const int RING_CX = 120;
const int RING_CY = 125;

void drawIdleScreen() {
  receiptVisible = false;
  tft.fillScreen(C_BG);

  drawTxtCenter(&FreeSansBold18pt7b, C_ACCENT, 120, 44, "SMART CART");

  // little card icon in the middle of the rings
  tft.fillRoundRect(92, 107, 56, 36, 5, C_CARD);
  tft.drawRoundRect(92, 107, 56, 36, 5, C_ACCENT);
  tft.fillRoundRect(98, 114, 12, 9, 2, C_GOLD);
  tft.fillRect(92, 129, 56, 5, C_ACCENT3);

  drawTxtCenter(&FreeSansBold12pt7b, C_TEXT, 120, 232, "Tap your card");
  drawTxtCenter(&FreeSans9pt7b, C_MUTED, 120, 258, "to start shopping");
  drawTxtCenter(&FreeSans9pt7b, C_MUTED, 120, 308, CART_ID);

  for (int k = 0; k < 3; k++) ringPrev[k] = -1;
}

void idleAnimTick() {
  if (millis() < idleAnimAt) return;
  idleAnimAt = millis() + 70;
  idleAnimFrame++;

  for (int k = 0; k < 3; k++) {
    int phase = (idleAnimFrame + k * 10) % 30;            // 0..29
    int r = 34 + (phase * 13) / 10;                       // 34..71 px
    if (ringPrev[k] > 0) {                                // erase where it was
      tft.drawCircle(RING_CX, RING_CY, ringPrev[k], C_BG);
      tft.drawCircle(RING_CX, RING_CY, ringPrev[k] + 1, C_BG);
    }
    uint16_t col = (phase < 12) ? C_ACCENT : ((phase < 22) ? C_ACCENT2 : C_ACCENT3);
    tft.drawCircle(RING_CX, RING_CY, r, col);
    tft.drawCircle(RING_CX, RING_CY, r + 1, col);
    ringPrev[k] = r;
  }
}

void drawWelcomeScreen(const String &name, int balance) {
  receiptVisible = false;
  tft.fillScreen(C_BG);
  animateMark(120, 70, C_ACCENT, true);

  drawTxtCenter(&FreeSans12pt7b, C_MUTED, 120, 150, "Welcome,");
  const GFXfont* nf = (textWidth(&FreeSansBold18pt7b, name) <= 224) ? &FreeSansBold18pt7b : &FreeSansBold12pt7b;
  drawTxtCenter(nf, C_TEXT, 120, 188, fitText(nf, name, 224));

  tft.fillRoundRect(30, 205, 180, 34, 8, C_CARD);
  drawTxt(&FreeSans9pt7b, C_MUTED, 40, 227, "Balance");
  drawTxtRight(&FreeSansBold12pt7b, C_ACCENT, 200, 229, "Rs." + String(balance));

  drawTxtCenter(&FreeSans9pt7b, C_MUTED, 120, 280, "Scan your items");
  drawTxtCenter(&FreeSans9pt7b, C_MUTED, 120, 300, "Tap your card again to pay");
}

void drawStatus(const String &msg) {
  receiptVisible = false;
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, 240, 4, C_ACCENT);
  drawWrappedCenter(&FreeSansBold12pt7b, C_TEXT, 120, 150, 30, 216, msg);
}

// ---- "put it back on the shelf" screens ----
void drawReturnScreen(const String &name) {
  receiptVisible = false;
  tft.fillScreen(C_BG);
  tft.fillRect(0, 0, 240, 44, C_RED);
  drawTxtCenter(&FreeSansBold12pt7b, C_TEXT, 120, 30, "RETURN ITEM");

  drawTxtCenter(&FreeSans12pt7b, C_MUTED, 120, 85, "Please put");
  const GFXfont* nf = (textWidth(&FreeSansBold18pt7b, name) <= 224) ? &FreeSansBold18pt7b : &FreeSansBold12pt7b;
  drawTxtCenter(nf, C_TEXT, 120, 122, fitText(nf, name, 224));
  drawTxtCenter(&FreeSans12pt7b, C_MUTED, 120, 154, "back on the shelf");
  drawTxtCenter(&FreeSans9pt7b, C_MUTED, 120, 304, "The shelf confirms it by weight");
}

void drawReturnArrow(int frame) {            // bouncing arrow pointing down at the shelf
  int off = (frame % 4) * 4;
  int cx = 120, top = 170 + off;
  tft.fillRect(90, 166, 60, 60, C_BG);
  tft.fillRect(cx - 6, top, 12, 20, C_ACCENT);
  tft.fillTriangle(cx - 16, top + 20, cx + 16, top + 20, cx, top + 38, C_ACCENT);
}

void drawReturnProgress(int secsLeft, int totalSecs) {   // draining time bar + seconds
  int w = (200 * secsLeft) / max(1, totalSecs);
  tft.fillRoundRect(20, 236, 200, 14, 7, C_CARD);
  if (w > 14) tft.fillRoundRect(20, 236, w, 14, 7, C_ACCENT);
  else if (w > 0) tft.fillRect(20, 236, w, 14, C_ACCENT);
  tft.fillRect(60, 254, 120, 30, C_BG);
  drawTxtCenter(&FreeSansBold12pt7b, C_TEXT, 120, 276, String(secsLeft) + " s");
}

void drawReturnResult(bool ok, const String &name) {
  receiptVisible = false;
  tft.fillScreen(C_BG);
  animateMark(120, 75, ok ? C_GREEN : C_RED, ok);
  drawTxtCenter(&FreeSansBold12pt7b, ok ? C_GREEN : C_RED, 120, 158, ok ? "RETURNED" : "NOT DETECTED");
  drawTxtCenter(&FreeSansBold12pt7b, C_TEXT, 120, 200, fitText(&FreeSansBold12pt7b, name, 224));
  drawTxtCenter(&FreeSans12pt7b, C_MUTED, 120, 232, ok ? "Removed from your cart" : "It stays in your cart");
  if (!ok) drawTxtCenter(&FreeSans9pt7b, C_MUTED, 120, 290, "Place it on the shelf and retry");
}

void drawPaymentResult(bool approved, const String &line1, const String &line2) {
  receiptVisible = false;
  tft.fillScreen(C_BG);
  animateMark(120, 75, approved ? C_GREEN : C_RED, approved);

  drawTxtCenter(&FreeSansBold18pt7b, approved ? C_GREEN : C_RED, 120, 160, approved ? "PAID" : "DECLINED");
  drawTxtCenter(&FreeSansBold12pt7b, C_TEXT, 120, 205, fitText(&FreeSansBold12pt7b, line1, 224));
  drawTxtCenter(&FreeSans12pt7b, C_MUTED, 120, 238, fitText(&FreeSans12pt7b, line2, 224));
  drawTxtCenter(&FreeSans9pt7b, C_MUTED, 120, 300, approved ? "Thank you!" : "Please try again");
}

// =====================================================
//  Cloud push helpers - Blynk (shopper's phone) AND Firebase (staff console)
// =====================================================
const char* stateText() {
  switch (state) {
    case IDLE:             return "IDLE";
    case SHOPPING:         return "AWAITING_PAYMENT";   // the dashboard's name for "shopping, not paid yet"
    case LOCKED:           return "LOCKED";
    case PAID:             return "PAID";
  }
  return "?";
}

void pushCartState() {
  Blynk.virtualWrite(V0, totalCost);
  Blynk.virtualWrite(V1, totalItems);
  Blynk.virtualWrite(V3, stateText());
  Blynk.virtualWrite(V5, customerName);

  if (!fbReady()) return;   // Firebase not started / signed in yet

  FirebaseJson json;
  json.set("totalDue", totalCost);
  json.set("itemsInCart", totalItems);
  json.set("cartStatus", stateText());
  json.set("customerName", customerName == "-" ? "" : customerName);
  json.set("customerUid", activeUid);
  json.set("notice", noticeText);
  json.set("mode", removeMode ? "REMOVE" : "ADD");
  json.set("deviceOnline", true);
  json.set("lastSeen/.sv", "timestamp");     // Firebase fills in the server time
  if (!Firebase.RTDB.updateNode(&fbdo, cartPath(), &json)) {
    Serial.println("[Firebase] state push failed: " + fbdo.errorReason());
  }
}

// Keyed by the scanned id, so a repeat scan UPDATES the line (quantity)
// instead of creating a duplicate.
void pushReceiptLine(const String &id, const String &name, int price, int qty) {
  Blynk.virtualWrite(V2, name + " x" + qty + "  Rs." + (price * qty) + "\n");

  if (!fbReady()) return;
  FirebaseJson line;
  line.set("productName", name);
  line.set("quantity", qty);
  line.set("unitPrice", price);
  line.set("lineTotal", price * qty);
  String path = cartPath() + "/receiptLines/" + id;
  if (!Firebase.RTDB.setJSON(&fbdo, path, &line)) {
    Serial.println("[Firebase] line push failed: " + fbdo.errorReason());
  }
}

void clearReceiptLines() {
  if (!fbReady()) return;
  Firebase.RTDB.deleteNode(&fbdo, cartPath() + "/receiptLines");
}

void logFirebaseEvent(const String &type, const String &msg) {
  if (!fbReady()) return;
  FirebaseJson event;
  event.set("cartId", CART_ID);
  event.set("type", type);
  event.set("message", msg);
  event.set("timestamp/.sv", "timestamp");
  if (!Firebase.RTDB.pushJSON(&fbdo, String(ROOT) + "/events", &event)) {
    Serial.println("[Firebase] event push failed: " + fbdo.errorReason());
  }
}

// eventCode : a Blynk event code (push notification) - or nullptr for a quiet info message
// fbType    : event type shown on the web console when eventCode is nullptr (nullptr = "info")
void sendAppMessage(const String &msg, const char* eventCode, const char* fbType) {
  Serial.println("[APP] " + msg);
  Blynk.virtualWrite(V2, msg + "\n");
  if (eventCode) Blynk.logEvent(eventCode, msg);

  const char* t = eventCode ? eventCode : (fbType ? fbType : "info");
  logFirebaseEvent(String(t), msg);
}

// A finished sale, kept for the web console's sales report.
void logTransaction() {
  if (!fbReady()) return;
  FirebaseJson tx;
  tx.set("cartId", CART_ID);
  tx.set("customerUid", activeUid);
  tx.set("customerName", customerName);
  tx.set("total", totalCost);
  tx.set("itemCount", totalItems);
  tx.set("timestamp/.sv", "timestamp");
  for (int i = 0; i < receiptLines; i++) {
    String base = "items/" + receipt[i].id;
    tx.set(base + "/name", receipt[i].name);
    tx.set(base + "/qty", receipt[i].qty);
    tx.set(base + "/unitPrice", receipt[i].price);
  }
  if (!Firebase.RTDB.pushJSON(&fbdo, String(ROOT) + "/transactions", &tx)) {
    Serial.println("[Firebase] transaction log failed: " + fbdo.errorReason());
  }
}

// Lets the web console offer "register this card / add this product" buttons.
void rememberUnknown(const char* node, const char* field, const String &value) {
  if (!fbReady()) return;
  FirebaseJson j;
  j.set(field, value);
  j.set("cartId", CART_ID);
  j.set("timestamp/.sv", "timestamp");
  Firebase.RTDB.setJSON(&fbdo, String(ROOT) + "/" + node, &j);
}

BLYNK_CONNECTED() {
  pushCartState();
}

// =====================================================
//  Session control
// =====================================================
// Back to the idle screen: clears the cart and waits for the next card tap.
// "why" is printed so the Serial Monitor shows WHAT ended the session.
void newSession(const char* why) {
  receiptLines = 0;
  totalItems = 0;
  totalCost = 0;
  state = IDLE;
  customerName = "-";
  activeUid = "";
  customerBalance = 0;
  screenReturnAt = 0;
  flashIdx = -1;
  removeMode = false;
  removeModeUntil = 0;
  pend.active = false;
  noticeText = "";
  if (fbReady()) Firebase.RTDB.deleteNode(&fbdo, cartPath() + "/pendingReturn");   // no stale return state for the next shopper
  alertText = "";                                  // the shelf's alert (if still open) is picked up again by the next shopper's poll
  alertCount = 0;
  for (int i = 0; i < 6; i++) suppUntil[i] = 0;
  clearReceiptLines();
  Blynk.virtualWrite(V2, "clr");
  drawIdleScreen();
  pushCartState();
  Serial.println("[IDLE] session reset (" + String(why) + ") - tap a registered card to start shopping");
}

BLYNK_WRITE(V4) {
  if (param.asInt() == 1) newSession("app New Session button");
}

// =====================================================
//  Scan handling (barcode or QR - whatever the module decodes)
// =====================================================
// =====================================================
//  Add / remove items
// =====================================================
void setRemoveMode(bool on) {
  removeMode = on;
  removeModeUntil = on ? millis() + REMOVE_MODE_IDLE_MS : 0;
  Serial.println(on ? "[MODE] REMOVE - scan the item you want to take out" : "[MODE] ADD");
  if (state == SHOPPING && !pend.active) drawReceiptScreen();
  pushCartState();
}

// Remove mode + a scan: the item only leaves the cart once the SHELF confirms,
// by weight, that it was put back.
// Tells the shopper app how the return is going: carts/<id>/pendingReturn
// {productId, name, status: waiting | confirmed | timeout, deadline (epoch ms)}
void writePendingReturn(const char* status) {
  if (!fbReady()) return;
  FirebaseJson j;
  j.set("productId", pend.id);
  j.set("name", pend.name);
  j.set("status", status);
  time_t nowSec = time(nullptr);
  double deadlineMs = 0;
  if (nowSec > 1700000000) {                       // clock is set (NTP)
    unsigned long left = (pend.deadline > millis()) ? (pend.deadline - millis()) : 0;
    deadlineMs = (double)nowSec * 1000.0 + (double)left;
  }
  j.set("deadline", deadlineMs);
  if (!Firebase.RTDB.setJSON(&fbdo, cartPath() + "/pendingReturn", &j))
    Serial.println("[Firebase] pendingReturn write failed: " + fbdo.errorReason());
}

void beginRemoval(const String &id) {
  int idx = -1;
  for (int i = 0; i < receiptLines; i++) {
    if (receipt[i].id == id) { idx = i; break; }
  }
  if (idx < 0) {
    Serial.println("[REMOVE] " + id + " is not in the cart");
    drawStatus("Not in your cart");
    screenReturnMode = 2;
    screenReturnAt = millis() + 1800;
    return;
  }

  int t, r, c;
  if (readCounters(id, t, r, c) < 0) {
    drawStatus("No connection");
    screenReturnMode = 2;
    screenReturnAt = millis() + 2000;
    return;
  }

  pend.active = true;
  pend.lineIdx = idx;
  pend.id = id;
  pend.name = receipt[idx].name;
  pend.baseReturned = r;                       // a rise in "returned" = the shelf felt it come back
  pend.deadline = millis() + RETURN_TIMEOUT_MS;
  pend.nextPollAt = millis() + 1200;
  pend.lastSecsLeft = -1;
  time_t nowSec = time(nullptr);
  pendSinceMs = (nowSec > 1700000000) ? (double)nowSec * 1000.0 : 0;   // for a put-back that happened a moment before the cart started waiting
  screenReturnAt = 0;
  noticeText = "Put " + pend.name + " back on the shelf";

  Serial.println("[REMOVE] waiting for " + pend.name + " to be put back (shelf counter returned=" + String(r) + ")");
  drawReturnScreen(pend.name);
  pushCartState();
  writePendingReturn("waiting");
}

void finishRemoval(bool ok) {
  String name = pend.name;
  String id = pend.id;
  int idx = pend.lineIdx;
  pend.active = false;
  noticeText = "";
  removeMode = false;
  removeModeUntil = 0;

  if (ok) {
    receipt[idx].qty--;
    totalItems--;
    totalCost -= receipt[idx].price;
    if (receipt[idx].qty <= 0) {                // last unit: drop the whole line
      if (fbReady()) Firebase.RTDB.deleteNode(&fbdo, cartPath() + "/receiptLines/" + id);
      for (int i = idx; i < receiptLines - 1; i++) receipt[i] = receipt[i + 1];
      receiptLines--;
    } else {
      pushReceiptLine(id, name, receipt[idx].price, receipt[idx].qty);
    }
    adjustClaimed(id, -1);
    Blynk.virtualWrite(V2, "Removed " + name + "\n");
    Serial.println("[REMOVE] " + name + " confirmed back on the shelf");
    drawReturnResult(true, name);
    sendAppMessage("Removed " + name + " (returned to the shelf).", nullptr, "item_removed");
  } else {
    Serial.println("[REMOVE] return of " + name + " NOT detected - it stays in the cart");
    drawReturnResult(false, name);
    sendAppMessage("Return of " + name + " was not detected on the shelf; it stays in the cart.", nullptr, "return_not_detected");
  }

  pushCartState();
  writePendingReturn(ok ? "confirmed" : "timeout");
  screenReturnMode = 2;                         // then back to the receipt
  screenReturnAt = millis() + 2200;
}

void pollPendingReturn() {
  if (!pend.active) return;
  unsigned long now = millis();

  // animation + countdown
  static unsigned long nextArrowAt = 0;
  static int arrowFrame = 0;
  if (now >= nextArrowAt) { nextArrowAt = now + 250; drawReturnArrow(arrowFrame++); }
  int secsLeft = (now >= pend.deadline) ? 0 : (int)((pend.deadline - now + 999) / 1000);
  if (secsLeft != pend.lastSecsLeft) {
    pend.lastSecsLeft = secsLeft;
    drawReturnProgress(secsLeft, RETURN_TIMEOUT_MS / 1000);
  }

  // ask the shelf whether the item has come back
  if (now >= pend.nextPollAt) {
    pend.nextPollAt = now + 1500;
    int t, r, c;
    int rc = readCounters(pend.id, t, r, c);
    // Returned = the shelf's counter went up since the removal started, OR the shelf reported a put-back
    // in the 15 s just before (the shopper was quicker than the cart's command check).
    bool returnedNow = (rc == 1) && (r > pend.baseReturned || (pendSinceMs > 0 && returnedAtMs >= pendSinceMs - 15000.0));
    if (returnedNow) {
      finishRemoval(true);
      return;
    }
    if (rc == 1) {
      Serial.printf("[REMOVE] waiting for '%s': shelf returned=%d (was %d), taken=%d, claimed=%d%s\n",
                    pend.id.c_str(), r, pend.baseReturned, t, c,
                    (t == 0 && r == 0) ? "  <-- the shelf has NEVER reported this product id: check the PRODUCT_ID table on the shelf" : "");
    }
  }
  if (millis() >= pend.deadline) finishRemoval(false);
}

bool lastButton = HIGH;
unsigned long lastButtonMs = 0;

void pollButton() {
#if REMOVE_BUTTON_PIN >= 0
  bool b = digitalRead(REMOVE_BUTTON_PIN);
  if (b != lastButton && millis() - lastButtonMs > 60) {
    lastButtonMs = millis();
    lastButton = b;
    if (b == LOW && state == SHOPPING && !pend.active) setRemoveMode(!removeMode);
  }
#endif
}

// ---- "not scanned" popup bookkeeping ----
void rebuildAlertText() {
  String old = alertText;
  if (alertCount == 0) alertText = "";
  else if (alertCount == 1) alertText = "Return or scan: " + alertItems[0].name + (alertItems[0].qty > 1 ? " x" + String(alertItems[0].qty) : String(""));
  else alertText = "Return or scan: " + alertItems[0].name + " +" + String(alertCount - 1) + " more";
  if (noticeText == old) noticeText = alertText;     // keep the app's notice in step (only if the alert owned it)
}

bool isSuppressed(const String &id) {
  for (int i = 0; i < 6; i++)
    if (suppUntil[i] != 0 && (long)(millis() - suppUntil[i]) < 0 && suppId[i] == id) return true;
  return false;
}

// The barcode scanner just added this product to the cart: take it out of the popup straight away
// (the shelf's own list can be a few seconds behind, so it is ignored for this product for 12 s).
void alertOnScanned(const String &id) {
  suppId[suppNext] = id;
  suppUntil[suppNext] = millis() + 12000;
  if (suppUntil[suppNext] == 0) suppUntil[suppNext] = 1;
  suppNext = (suppNext + 1) % 6;

  for (int i = 0; i < alertCount; i++) {
    if (alertItems[i].id != id) continue;
    alertItems[i].qty--;
    if (alertItems[i].qty <= 0) {
      for (int k = i; k < alertCount - 1; k++) alertItems[k] = alertItems[k + 1];
      alertCount--;
    }
    Serial.println("[ALERT] scanned " + id + " - popup updated");
    rebuildAlertText();
    break;
  }
}

void handleScan(const String &id) {
  if (state != SHOPPING) {
    if (state == IDLE) Serial.println("[SCAN] ignored - tap your card to start first");
    else Serial.println("[SCAN] ignored - cart is " + String(stateText()));
    return;
  }

  // Control codes (print them as QR / barcodes and stick them on the cart handle)
  if (id == "CMD:REMOVE") { setRemoveMode(true);  return; }
  if (id == "CMD:ADD")    { setRemoveMode(false); return; }

  if (pend.active) {
    Serial.println("[SCAN] ignored - finish putting " + pend.name + " back on the shelf first");
    return;
  }
  if (removeMode) { beginRemoval(id); return; }

  String name; int price;
  int found = lookupProduct(id, name, price);
  if (found < 0) {
    Serial.println("[SCAN] " + id + " could not be checked - no connection to Firebase. Scan again.");
    return;   // not the shopper's fault and not an unknown item, so don't log it as one
  }
  if (found == 0) {
    Serial.println("[SCAN] unregistered code: " + id);
    sendAppMessage("Unknown item scanned (" + id + ")", "unknown_item", nullptr);
    rememberUnknown("lastUnknownCode", "code", id);
    return;
  }

  // Shelf check: did the shelf really feel this item being taken?
  int taken = 0, returned = 0, claimed = 0;
  bool verified = (readCounters(id, taken, returned, claimed) == 1) && (taken - returned - claimed >= 1);
  if (!verified) {
    if (REQUIRE_PICKUP_FOR_ADD) {
      Serial.println("[SCAN] " + name + " refused - the shelf did not see it being taken");
      logFirebaseEvent("unverified_add", "Scan of " + name + " blocked: no matching shelf pickup (" + CART_ID + ")");
      drawStatus("Take it from the shelf first");
      screenReturnMode = 2;
      screenReturnAt = millis() + 2200;
      return;
    }
    Serial.println("[SCAN] " + name + " has no matching shelf pickup (added anyway, flagged for staff)");
    logFirebaseEvent("unverified_add", "Scanned " + name + " without a matching shelf pickup (" + CART_ID + ")");
  }

  int lineIdx = -1;
  for (int i = 0; i < receiptLines; i++) {
    if (receipt[i].id == id) { lineIdx = i; break; }
  }
  if (lineIdx >= 0) {
    receipt[lineIdx].qty++;
    receipt[lineIdx].name  = name;    // picks up a rename mid-session too
    receipt[lineIdx].price = price;
  } else if (receiptLines < 20) {
    receipt[receiptLines].id    = id;
    receipt[receiptLines].name  = name;
    receipt[receiptLines].price = price;
    receipt[receiptLines].qty   = 1;
    lineIdx = receiptLines;
    receiptLines++;
  } else {
    Serial.println("[SCAN] receipt full, ignoring");
    return;
  }

  totalItems++;
  totalCost += price;

  Serial.println("[SCAN] " + name + " qty=" + receipt[lineIdx].qty);

  alertOnScanned(id);                    // an open "not scanned" popup for this item goes away right now
  drawReceiptScreen();
  flashIdx = lineIdx;                    // the new / updated row glows teal for a moment
  flashUntil = millis() + 600;
  drawItemRow(lineIdx, true);

  pushReceiptLine(id, name, price, receipt[lineIdx].qty);
  pushCartState();
  adjustClaimed(id, 1);                  // tell the shelf bookkeeping this item is now in a cart
}

#define SCANNER_DEBUG 1          // 1 = print every raw byte from the scanner; set 0 when it works
unsigned long lastScanByteMs = 0;

// Common scanner speeds. Type "b" + Enter in the Serial Monitor to step to the next one.
const unsigned long scannerBauds[] = {9600, 19200, 38400, 57600, 115200};
const int SCANNER_BAUD_COUNT = sizeof(scannerBauds) / sizeof(scannerBauds[0]);
int scannerBaudIdx = 4;   // 115200 baud - the speed that worked for this scanner

void cycleScannerBaud() {
  scannerBaudIdx = (scannerBaudIdx + 1) % SCANNER_BAUD_COUNT;
  scannerSerial.end();
  scannerSerial.begin(scannerBauds[scannerBaudIdx], SERIAL_8N1, SCANNER_RX, SCANNER_TX);
  scanBuffer = "";
  Serial.println("[SCANNER] speed now " + String(scannerBauds[scannerBaudIdx]) + " baud - scan a code");
}

void finishScan() {
  scanBuffer.trim();
  bool printable = scanBuffer.length() > 0;
  for (unsigned int i = 0; i < scanBuffer.length(); i++) {
    char ch = scanBuffer[i];
    if (ch < 32 || ch > 126) { printable = false; break; }
  }

  if (scanBuffer.length() > 0 && !printable) {
    Serial.println("[SCANNER] unreadable data (wrong baud rate?). Type b + Enter to try the next speed.");
  } else if (printable) {
    Serial.println("[SCANNER] got: " + scanBuffer);
    handleScan(scanBuffer);
  }
  scanBuffer = "";
}

void pollScanner() {
  while (scannerSerial.available()) {
    int b = scannerSerial.read();
    lastScanByteMs = millis();
#if SCANNER_DEBUG
    Serial.printf("[RAW] 0x%02X '%c'\n", b, (b >= 32 && b < 127) ? (char)b : '.');
#endif
    char c = (char)b;
    if (c == '\r' || c == '\n') {
      finishScan();                 // scanner sent an end-of-line
    } else {
      scanBuffer += c;
      if (scanBuffer.length() > 48) scanBuffer = "";   // guard against junk
    }
  }

  // Scanner sent no end-of-line: treat 80 ms of silence as "scan finished"
  if (scanBuffer.length() > 0 && millis() - lastScanByteMs > 80) {
    finishScan();
  }
}

// =====================================================
//  RFID: start session / pay
// =====================================================
String uidToString(uint8_t *uid, uint8_t len) {
  String s = "";
  for (uint8_t i = 0; i < len; i++) {
    if (uid[i] < 0x10) s += "0";
    s += String(uid[i], HEX);
    if (i < len - 1) s += ":";
  }
  s.toUpperCase();
  return s;
}

// Card tap logic:
//   IDLE      + registered card  -> start shopping, welcome the user
//   SHOPPING  + the same card    -> pay (or just end the session if nothing was scanned)
//   SHOPPING  + a different card -> ignored, the cart is in use
void handleTap(const String &id) {
  Serial.println("[NFC] card " + id + "  state=" + stateText() + " items=" + totalItems);

  // ---------- IDLE: a registered card starts a session ----------
  if (state == IDLE) {
    String name; int balance = 0;
    int r = loadCustomer(id, name, balance);

    if (r < 0) {
      Serial.println("[NFC] could not check the card - no connection to Firebase");
      drawStatus("No connection");
      screenReturnMode = 3;                 // just redraw the idle screen
      screenReturnAt = millis() + 2500;
      return;
    }
    if (r == 0) {
      sendAppMessage("Unregistered card " + id + " tried to start the cart.", "unknown_card", nullptr);
      rememberUnknown("lastUnregisteredCard", "uid", id);
      PLAY_DECLINED();
      drawStatus("Card not registered");
      screenReturnMode = 3;
      screenReturnAt = millis() + 2500;
      return;
    }

    activeUid = id;
    customerName = name;
    customerBalance = balance;
    state = SHOPPING;
    PLAY_TAP();
    drawWelcomeScreen(customerName, customerBalance);
    screenReturnMode = 2;                   // after the welcome, show the receipt screen
    screenReturnAt = millis() + 2500;
    pushCartState();
    sendAppMessage("Welcome, " + customerName + "! Shopping started.", nullptr, "session_started");
    return;
  }

  // ---------- SHOPPING: tapping again pays ----------
  if (state == SHOPPING) {
    if (pend.active) {
      Serial.println("[NFC] ignored - finish putting " + pend.name + " back on the shelf first");
      return;
    }
    if (id != activeUid) {
      Serial.println("[NFC] ignored - this cart is in use by " + customerName);
      drawStatus("Cart in use");
      screenReturnMode = 2;
      screenReturnAt = millis() + 2000;
      return;
    }

    if (totalItems == 0) {                  // nothing scanned: tapping again just ends the session
      sendAppMessage(customerName + " ended the session with an empty cart.", nullptr, "session_ended");
      newSession("card tapped with an empty cart");
      return;
    }

    // Read the balance fresh, so a top-up made on the web console counts straight away
    String n; int bal = 0;
    int r = loadCustomer(activeUid, n, bal);
    if (r != 1) {
      Serial.println("[NFC] could not read the balance - tap again");
      drawStatus(r < 0 ? "No connection" : "Account not found");
      screenReturnMode = 2;
      screenReturnAt = millis() + 2500;
      return;
    }
    customerBalance = bal;

    if (bal < totalCost) {
      String m = customerName + ": declined, need Rs." + totalCost + ", have Rs." + bal;
      sendAppMessage(m, "payment_declined", nullptr);
      PLAY_DECLINED();
      drawPaymentResult(false, "Insufficient balance", customerName);
      screenReturnMode = 2;                 // stays in SHOPPING (staff or the app's New Session button can cancel)
      screenReturnAt = millis() + 4000;
      pushCartState();
      return;
    }

    state = LOCKED;
    pushCartState();

    int newBal = bal - totalCost;
    if (!Firebase.RTDB.setInt(&fbdo, String(ROOT) + "/customers/" + activeUid + "/balance", newBal)) {
      // The money could not be saved, so the sale is NOT approved
      Serial.println("[Firebase] could not save the new balance: " + fbdo.errorReason());
      state = SHOPPING;
      pushCartState();
      PLAY_FAIL();
      drawStatus("Payment failed - tap again");
      screenReturnMode = 2;
      screenReturnAt = millis() + 3000;
      return;
    }
    customerBalance = newBal;

    state = PAID;
    pushCartState();
    logTransaction();

    String m = "Payment approved. " + customerName + " paid Rs." + totalCost +
               ". Balance: Rs." + newBal;
    sendAppMessage(m, "payment_approved", nullptr);
    PLAY_PAID();
    drawPaymentResult(true, customerName, "Rs." + String(totalCost) + " paid");
    screenReturnMode = 1;                   // then back to idle for the next shopper
    screenReturnAt = millis() + 6000;
    return;
  }

  // LOCKED / PAID: ignore taps while the result is on screen
}

// A tap only counts once the card has been LIFTED away from the reader, so one
// long hold can't both start the session and pay in the same moment.
bool cardLatched = false;
unsigned long lastCardSeenMs = 0;

void pollCard() {
  if (millis() - lastPollMs < 200) return;
  lastPollMs = millis();

  uint8_t uid[7]; uint8_t uidLength = 0;
  bool seen = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 50);

  if (seen) {
    lastCardSeenMs = millis();
    if (cardLatched) return;               // same card still resting on the reader
    cardLatched = true;
    handleTap(uidToString(uid, uidLength));
  } else if (cardLatched && millis() - lastCardSeenMs > 700) {
    cardLatched = false;                   // card was removed: the next tap counts
  }
}

// =====================================================
//  Heartbeat + staff commands from the web console
// =====================================================
unsigned long nextHeartbeatMs = 0;

void heartbeatAndCommands() {
  if (!fbReady() || state == LOCKED || millis() < nextHeartbeatMs) return;
  nextHeartbeatMs = millis() + 15000;

  FirebaseJson hb;
  hb.set("deviceOnline", true);
  hb.set("lastSeen/.sv", "timestamp");
  hb.set("rssi", WiFi.RSSI());
  if (!Firebase.RTDB.updateNode(&fbdo, cartPath(), &hb)) {
    Serial.println("[Firebase] heartbeat failed: " + fbdo.errorReason());
    nextHeartbeatMs = millis() + 60000;     // back off while the network is bad
    return;
  }

}

// Commands written to carts/<CART_ID>/command:
//   "end_session"        from the staff console
//   "remove:<productId>" from the shopper app (starts the put-it-back-on-the-shelf flow)
// Polled every 3 s while shopping (so the app feels responsive), every 15 s otherwise.
unsigned long nextCmdPollMs = 0;

void pollCommands() {
  if (!fbReady() || state == LOCKED || pend.active || millis() < nextCmdPollMs) return;
  if (cardLatched) return;     // a card is on the reader right now: never block the loop (a slow Firebase call would miss the tap)
  nextCmdPollMs = millis() + (state == SHOPPING ? 5000 : 15000);

  if (!Firebase.RTDB.getString(&fbdo, cartPath() + "/command")) return;   // no command node = nothing to do
  String cmd = fbdo.stringData();
  if (cmd.length() == 0) return;

  // Delete it first so it runs once. If the delete fails (bad network), do NOT run it now -
  // otherwise the same command would fire again on every poll.
  if (!Firebase.RTDB.deleteNode(&fbdo, cartPath() + "/command")) {
    Serial.println("[ADMIN] could not clear command '" + cmd + "' (" + fbdo.errorReason() + ") - will retry");
    return;
  }
  Serial.println("[ADMIN] command: " + cmd);

  if (cmd == "end_session") {
    if (state == IDLE) {
      Serial.println("[ADMIN] end_session ignored - no session is running");
    } else {
      sendAppMessage("Session ended by staff.", nullptr, "admin_action");
      newSession("staff command");
    }
  } else if (cmd.startsWith("remove:")) {
    String pid = cmd.substring(7);
    pid.trim();
    if (state == SHOPPING) {
      beginRemoval(pid);                    // shows the return screen and waits for the shelf
    } else {
      Serial.println("[ADMIN] remove ignored - no shopping session is running");
    }
  } else {
    Serial.println("[ADMIN] unknown command ignored");
  }
}

// =====================================================
//  Setup / loop
// =====================================================
const char* resetReasonText() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "power on";
    case ESP_RST_SW:       return "software reset";
    case ESP_RST_PANIC:    return "CRASH (panic)";
    case ESP_RST_INT_WDT:  return "watchdog (interrupt)";
    case ESP_RST_TASK_WDT: return "watchdog (task stuck)";
    case ESP_RST_WDT:      return "watchdog";
    case ESP_RST_BROWNOUT: return "BROWNOUT (power dip)";
    default:               return "other";
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("[BOOT] last reset reason: " + String(resetReasonText()));
  buzzerInit();

  scannerSerial.begin(scannerBauds[scannerBaudIdx], SERIAL_8N1, SCANNER_RX, SCANNER_TX);

#if REMOVE_BUTTON_PIN >= 0
  pinMode(REMOVE_BUTTON_PIN, INPUT_PULLUP);
#endif

  SPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.begin();
  tft.setRotation(0);   // portrait

  nfc.begin();
  if (!nfc.getFirmwareVersion()) {
    Serial.println("[NFC] PN532 not found - check wiring/mode switches");
  } else {
    nfc.SAMConfig();
    Serial.println("[NFC] PN532 ready");
  }

  // Wi-Fi with a 20 s timeout, so a wrong password shows up on the TFT and
  // Serial Monitor instead of freezing the whole sketch.
  drawStatus("Connecting WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  if (wifiOk) {
    Serial.println("[WiFi] connected, IP " + WiFi.localIP().toString());
    WiFi.setSleep(false);   // no Wi-Fi power saving: steadier, lower latency
    configTime(19800, 0, "pool.ntp.org", "time.google.com");   // clock for secure connections (UTC+5:30)
    drawStatus("Connecting cloud...");
    Blynk.config(BLYNK_AUTH_TOKEN);
    Blynk.connect(8000);
  } else {
    Serial.println("[WiFi] FAILED (status " + String(WiFi.status()) + "). Check SSID/password; ESP32 needs 2.4 GHz.");
    drawStatus("WiFi FAILED");
    delay(3000);
  }

  fbconfig.api_key = API_KEY;
  fbconfig.database_url = DATABASE_URL;
  // Slow network: give the secure handshake much longer before giving up.
  fbconfig.timeout.socketConnection = 30 * 1000;
  fbconfig.timeout.serverResponse   = 30 * 1000;
  fbconfig.timeout.sslHandshake     = 120 * 1000;
  fbdo.setBSSLBufferSize(4096, 1024);
  fbdo.setResponseSize(2048);
  // Anonymous sign-in. Needs: Firebase Console > Authentication > Sign-in
  // method > Anonymous = Enabled, and a correct API_KEY (Project settings > General > Web API Key).
  if (wifiOk) {
    drawStatus("Signing in...");
    bool signedIn = false;
    for (int attempt = 1; attempt <= 3 && !signedIn; attempt++) {
      Serial.println("[Firebase] sign-in attempt " + String(attempt) + " of 3");
      signedIn = Firebase.signUp(&fbconfig, &fbauth, "", "");
      if (!signedIn) {
        Serial.println("[Firebase] sign-in FAILED: " + String(fbconfig.signer.signupError.message.c_str()));
        delay(2000);
      }
    }
    if (signedIn) Serial.println("[Firebase] anonymous sign-in OK");
  }
  fbconfig.token_status_callback = tokenStatusCallback;
  Firebase.begin(&fbconfig, &fbauth);
  fbBegun = true;
  Firebase.reconnectWiFi(true);

  // Wait (up to 10 s) for Firebase to be ready before the first push.
  unsigned long fbStart = millis();
  while (!Firebase.ready() && millis() - fbStart < 10000) delay(200);
  Serial.println(Firebase.ready() ? "[Firebase] ready" : "[Firebase] NOT ready - check API_KEY, DATABASE_URL and rules");

  // Throw away any command left over from before this boot (e.g. an old "end_session"),
  // so the cart never ends a session because of a stale command.
  if (Firebase.ready()) {
    if (Firebase.RTDB.deleteNode(&fbdo, cartPath() + "/command")) Serial.println("[ADMIN] cleared any stale command");
  }

  Serial.println("Cart ID: " CART_ID);
  newSession("boot");
  Serial.println("Ready - tap a registered card to start shopping, scan items, tap again to pay.");
}

// Items lifted off the shelf over a minute ago and not scanned or returned:
// the shelf writes smartcart/alertSummary/list = "Name|qty|productId;..." (the node is deleted when all is clear).
// While shopping, the cart shows them as a popup on the LCD and sounds a very quiet warning.
void pollAlertSummary() {
  if (!fbReady() || state != SHOPPING || pend.active || cardLatched || millis() < nextAlertPollMs) return;
  nextAlertPollMs = millis() + (alertCount > 0 ? 4000 : 5000);
  String list = "";
  if (Firebase.RTDB.getString(&fbdo, String(ROOT) + "/alertSummary/list")) {
    list = fbdo.stringData();
  } else if (fbdo.errorReason().indexOf("not exist") < 0) {
    return;                                        // network problem: keep what is shown
  }

  // parse "Name|qty|id;Name|qty|id"
  AlertItem fresh[4];
  int n = 0;
  int pos = 0;
  while (pos < (int)list.length() && n < 4) {
    int semi = list.indexOf(';', pos);
    if (semi < 0) semi = list.length();
    String part = list.substring(pos, semi);
    pos = semi + 1;
    int b1 = part.indexOf('|');
    int b2 = part.indexOf('|', b1 + 1);
    if (b1 <= 0 || b2 < 0) continue;
    String pid = part.substring(b2 + 1);
    pid.trim();
    if (isSuppressed(pid)) continue;               // just scanned here: the shelf has not caught up yet
    fresh[n].name = part.substring(0, b1);
    fresh[n].qty = part.substring(b1 + 1, b2).toInt();
    fresh[n].id = pid;
    if (fresh[n].qty > 0) n++;
  }

  bool changed = (n != alertCount);
  bool louder = (n > alertCount);                  // a new item joined the popup
  for (int i = 0; i < n && !changed; i++) {
    if (fresh[i].id != alertItems[i].id || fresh[i].qty != alertItems[i].qty) changed = true;
    if (fresh[i].qty > alertItems[i].qty) louder = true;
  }
  if (!changed) return;

  bool wasOpen = (alertCount > 0);
  for (int i = 0; i < n; i++) alertItems[i] = fresh[i];
  alertCount = n;
  rebuildAlertText();
  Serial.println(n ? "[ALERT] popup: " + alertText : String("[ALERT] popup cleared (item back on the shelf or scanned)"));
  if (n > 0 && (louder || !wasOpen)) { PLAY_WARN(); nextAlertBeepMs = millis() + ALERT_REMIND_MS; }
  pushCartState();
  if (receiptVisible && screenReturnAt == 0) drawReceiptScreen();
}

void alertReminderTick() {                         // quiet reminder every 15 s while the popup is open
  if (alertCount > 0 && state == SHOPPING && !pend.active && millis() >= nextAlertBeepMs) {
    PLAY_WARN();
    nextAlertBeepMs = millis() + ALERT_REMIND_MS;
  }
}

void loop() {
  Blynk.run();
  buzzerTick();
  alertReminderTick();
  pollScanner();
  pollCard();
  heartbeatAndCommands();
  pollCommands();
  pollAlertSummary();
  pollButton();
  pollPendingReturn();
  if (removeMode && !pend.active && millis() > removeModeUntil) setRemoveMode(false);   // remove mode times out

  // Move the display on after a welcome / result screen has been shown for a moment
  if (screenReturnAt != 0 && millis() >= screenReturnAt) {
    screenReturnAt = 0;
    if (screenReturnMode == 1) newSession("after payment");   // full reset to idle
    else if (screenReturnMode == 2) drawReceiptScreen();      // back to the receipt
    else if (screenReturnMode == 3) drawIdleScreen();         // just the idle screen
  }

  // Idle screen animation (only while nothing else is on the display)
  if (state == IDLE && screenReturnAt == 0) idleAnimTick();

  // End the teal glow on the row that was just scanned
  if (flashIdx >= 0 && millis() > flashUntil) {
    if (receiptVisible && state == SHOPPING) drawItemRow(flashIdx, false);
    flashIdx = -1;
  }

  // Serial Monitor helpers:
  //   n  = end the session / back to idle     b  = try the next scanner speed
  //   anything else = pretend that text was scanned (the cart must be SHOPPING)
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line == "n") {
      newSession("Serial Monitor n");
    } else if (line == "b") {
      cycleScannerBaud();
    } else if (line.length() > 0) {
      handleScan(line);
    }
  }
}
