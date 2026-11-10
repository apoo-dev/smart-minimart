#include "HX711.h"
#include <math.h>
#include <stdio.h>
#include <WiFi.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>

// ============================================================
// FOUR-CELL SMART MINI MART - ESP32 + 4 x HX711
// Each cell has three product types, tracked independently.
// Cell 1 and Cell 2: 1 kg load cells
// Cell 3 and Cell 4: 5 kg load cells
// ============================================================

// Define these BEFORE any arrays that use them as their sizes.
// #define makes the sizes visible to the entire sketch during preprocessing.
#define NUM_CELLS 4
#define NUM_PRODUCTS 3

HX711 scales[NUM_CELLS];

// A cell is enabled only if its HX711 responds during startup and calibration.
bool cellAvailable[NUM_CELLS] = {false, false, false, false};

// Pin mapping for the ESP32 DevKit V1 / ESP-WROOM-32 board in your picture:
// GPIO 4  = D4       GPIO 5  = D5
// GPIO 16 = RX2      GPIO 19 = D19
// GPIO 17 = TX2      GPIO 21 = D21
// GPIO 25 = D25      GPIO 26 = D26 (Cell 4 moved to these pins)
// Keep GPIO numbers in the code. Do not connect the four DT pins together
// or the four SCK pins together; each HX711 needs its own pair of pins.
const uint8_t DOUT_PINS[NUM_CELLS] = {4, 16, 17, 25};
const uint8_t CLK_PINS[NUM_CELLS]  = {5, 19, 21, 26};

// Capacity information is for reference only; it is not used as a calibration factor.
const float CELL_CAPACITY_G[NUM_CELLS] = {1000.0, 1000.0, 5000.0, 5000.0};

// Calibration reference weights, in grams.
// The same known 100 g reference weight is used for all four cells.
const float CALIBRATION_WEIGHT_G[NUM_CELLS] = {100.0, 100.0, 100.0, 100.0};
float calibrationFactor[NUM_CELLS] = {1.0, 1.0, 1.0, 1.0};

// Actual product weights in grams for each cell, based on the supplied list.
// The order in each row is Product A, Product B and Product C.
const float PRODUCT_WEIGHT_G[NUM_CELLS][NUM_PRODUCTS] = {
  {14.0,  45.0, 100.0},  // Cell 1: ON THE GO, Popit, Iodex
  {25.0,  60.0, 114.0},  // Cell 2: Milo, Hair Oil, Marmite
  {50.0, 160.0, 250.0},  // Cell 3: Happy Cow Cheese, Imthiri Watalappan, Cream Cracker
  {72.0, 225.0, 400.0}   // Cell 4: Toblerone, Sponge Roll, Strawberry Jam
};

// Each cell has its own product names in the same order as PRODUCT_WEIGHT_G.
const char* PRODUCT_NAMES[NUM_CELLS][NUM_PRODUCTS] = {
  {"ON THE GO", "Popit", "Iodex"},
  {"Milo", "Hair Oil", "Marmite"},
  {"Happy Cow Cheese", "Imthiri Watalappan", "Cream Cracker"},
  {"Toblerone", "Sponge Roll", "Strawberry Jam"}
};
const int MAX_QUANTITY[NUM_PRODUCTS] = {3, 3, 3};

// Quantity of each product in each cell. Example: quantities[2][1] is Cell 3, Product B.
int quantities[NUM_CELLS][NUM_PRODUCTS] = {};

// Detection settings.
// Readings are filtered and a weight change must remain stable for a period
// before it is accepted. This helps reject temporary pauses and vibration.
const float MATCH_TOLERANCE_G = 4.0;
const float MIN_MATCH_MARGIN_G = 3.0;  // Reject close combinations, especially Cell 1: 3 x 14 g vs 45 g.
const float NOISE_LIMIT_G = 3.0;
const float STABILITY_BAND_G = 2.0;
const float FILTER_ALPHA = 0.25;       // EMA filter: lower = smoother, slower response.
const unsigned long REQUIRED_STABLE_TIME_MS = 650;
const int REQUIRED_STABLE_SAMPLES = 5;

float previousWeight[NUM_CELLS] = {0, 0, 0, 0};
float filteredWeight[NUM_CELLS] = {0, 0, 0, 0};
bool filterInitialized[NUM_CELLS] = {false, false, false, false};
bool changePending[NUM_CELLS] = {false, false, false, false};
float pendingAnchorWeight[NUM_CELLS] = {0, 0, 0, 0};
unsigned long pendingSince[NUM_CELLS] = {0, 0, 0, 0};
int pendingStableSamples[NUM_CELLS] = {0, 0, 0, 0};

// Latest readings shown in the real-time dashboard.
float lastMeasuredWeight[NUM_CELLS] = {0, 0, 0, 0};
bool hasLiveReading[NUM_CELLS] = {false, false, false, false};
unsigned long lastDashboardTime = 0;
const unsigned long DASHBOARD_INTERVAL_MS = 1000; // refresh once per second

// ============================================================
// NETWORK SETTINGS  (fill these in)
//   The shelf and the cart talk to each other THROUGH FIREBASE (over Wi-Fi). No ESP-NOW.
// ============================================================
#define SHELF_ID        "shelf-01"
#define WIFI_SSID       "Error_404_3G"
#define WIFI_PASS       "no hotspotsusme"
#define API_KEY         "AIzaSyDA76GWSVKqAfodI6pdakzMbCm1tHNGr9E"     // Firebase > Project settings > General > Web API Key
#define DATABASE_URL    "https://smartcart-b9ed2-default-rtdb.firebaseio.com"
#define BUZZER_PIN      27                      // buzzer (+) to GPIO 27, (-) to GND. Set -1 for none.
#define LED_PIN         2                       // on-board LED also blinks on a warning
#define UNSCANNED_MS    15000                   // 15 s after a lift with no scan/return: local WARNING (buzzer + event)
#define STAFF_ALERT_MS  60000                   // 1 minute: STAFF ALERT + warning to the shopper, until returned or scanned
#define RECHECK_MS      5000                    // while an alert is open, look again every 5 s

// ---- Buzzer (quiet on purpose) ----
// BUZZER_PASSIVE 1 = passive buzzer: tones, and the volume can be turned DOWN (WARN_VOL_PCT).
// BUZZER_PASSIVE 0 = active buzzer: it can only beep on/off, so warnings are just very short beeps.
#define BUZZER_PASSIVE  1
#define WARN_VOL_PCT    6                       // warning volume 1..100  (small = very quiet; raise it if you cannot hear it)
#define BZ_CH           7                       // LEDC channel (only used on older ESP32 cores)

// The cart identifies products by their CATALOG id (the barcode number / the key under
// smartcart/catalog, e.g. "9787223465410"). Put the SAME ids here, in the same order as
// PRODUCT_NAMES above. These are placeholders - replace all 12.
const char* PRODUCT_ID[NUM_CELLS][NUM_PRODUCTS] =  {
  {"9787223465410", "9787223465380", "9787223465397"},   // Cell 1: ON THE GO, Popit, Iodex
  {"9787223462341", "9787223469340", "9787223465366"},   // Cell 2: Milo, Hair Oil, Marmite
  {"9787223465373", "9787223465342", "9787223465427"},   // Cell 3: Happy Cow Cheese, Imthiri Watalappan, Cream Cracker
  {"9787227462347", "9787227464341", "9787223465359"}    // Cell 4: Toblerone, Sponge Roll, Strawberry Jam
};

// STOCKING mode (default at power-up): weight changes only update the stock count.
// Type 'l' + ENTER in the Serial Monitor to go LIVE; 's' to return to stocking.
bool liveMode = false;
unsigned long lastBeatAt = 0;
volatile bool fbSignedIn = false;

FirebaseData   fbdo;
FirebaseAuth   fbauth;
FirebaseConfig fbconfig;

// ---- Types: they must stay ABOVE the first function in this file ----
enum JobType : uint8_t { JOB_TAKEN, JOB_RETURNED, JOB_CHECK, JOB_STOCK, JOB_BEAT, JOB_SUMMARY, JOB_RESET };
struct FbJob { uint8_t type; char id[16]; int n; int extra; char msg[96]; };   // n = check stage; extra = cell*NUM_PRODUCTS+product (stock jobs)
QueueHandle_t fbQueue = nullptr;

#define WATCH_MAX 16
#define MAXP 8                      // lift times remembered per product
struct Watch {
  bool used;
  char id[16];
  int outstanding;            // items lifted and not yet put back or scanned
  int nPick;                  // how many lift times are remembered (<= MAXP)
  unsigned long pick[MAXP];   // WHEN each item was lifted, oldest first: every item has its own countdown
  bool warned15;              // the quiet 15 s warning has been given
  bool alert60;               // a 1 minute STAFF ALERT is open for this product
  int alertQty;               // how many items that open alert is about
  bool checking;              // a Firebase check for this product is queued
  unsigned long nextRecheck;  // when to look again while the alert is open
};
Watch watches[WATCH_MAX];
portMUX_TYPE watchMux = portMUX_INITIALIZER_UNLOCKED;

struct Note { uint16_t freq; uint16_t ms; };      // freq 0 = silence
const Note SND_WARN[]  = { {700, 60}, {0, 110}, {700, 60} };                          // soft double beep
const Note SND_ALERT[] = { {600, 70}, {0, 90}, {600, 70}, {0, 90}, {600, 70} };       // soft triple beep


// ---- Firebase jobs run on their own task so slow Wi-Fi never delays the load cells ----
bool queueJob(uint8_t type, const char* id, int n, const char* msg) {
  if (!fbQueue) return false;
  FbJob j;
  memset(&j, 0, sizeof(j));
  j.type = type;
  strncpy(j.id, id, sizeof(j.id) - 1);
  j.n = n;
  strncpy(j.msg, msg, sizeof(j.msg) - 1);
  if (xQueueSend(fbQueue, &j, 0) != pdTRUE) { Serial.println("[FB] queue full - job delayed"); return false; }
  return true;
}

// Stock level for the staff dashboard: smartcart/shelves/<SHELF_ID>/inventory/<productId>
void queueStockJob(const char* id, int qty, int cell, int product) {
  if (!fbQueue) return;
  FbJob j;
  memset(&j, 0, sizeof(j));
  j.type = JOB_STOCK;
  strncpy(j.id, id, sizeof(j.id) - 1);
  j.n = qty;
  j.extra = cell * NUM_PRODUCTS + product;
  if (xQueueSend(fbQueue, &j, 0) != pdTRUE) Serial.println("[FB] queue full - stock update dropped");
}

// Writes every product's current count (so empty ones show as "out of stock" on the dashboard).
void publishAllStock() {
  for (int c = 0; c < NUM_CELLS; c++) {
    if (!cellAvailable[c]) continue;                 // a skipped cell has no real reading
    for (int p = 0; p < NUM_PRODUCTS; p++) queueStockJob(PRODUCT_ID[c][p], quantities[c][p], c, p);
  }
  Serial.println("[FB] publishing the full inventory");
}

const char* nameForId(const char* id) {
  for (int c = 0; c < NUM_CELLS; c++)
    for (int p = 0; p < NUM_PRODUCTS; p++)
      if (strcmp(PRODUCT_ID[c][p], id) == 0) return PRODUCT_NAMES[c][p];
  return id;
}

// ---- Quiet buzzer ----
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
bool bzSounding = false;

void buzzerRaw(int freq, uint8_t volPct) {
  bzSounding = (freq > 0);
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

void buzzerTick() {                                           // call often; never blocks
  if (!bzSeq) return;
  unsigned long now = millis();
  if (bzUntil != 0 && (long)(now - bzUntil) < 0) return;
  if (bzIdx >= bzLen) { buzzerRaw(0, 0); bzSeq = nullptr; return; }
  const Note& n = bzSeq[bzIdx++];
  buzzerRaw(n.freq, bzVol);
  bzUntil = now + n.ms;
}

#define PLAY_WARN()   buzzerPlay(SND_WARN,  sizeof(SND_WARN)  / sizeof(SND_WARN[0]),  WARN_VOL_PCT)
#define PLAY_ALERT()  buzzerPlay(SND_ALERT, sizeof(SND_ALERT) / sizeof(SND_ALERT[0]), WARN_VOL_PCT)

// ---- Per-item timers ----
//  Every lifted item remembers its OWN lift time, so two items lifted at different moments count down separately.
//  15 s : the oldest item is not scanned yet -> a quiet warning beep + event
//  60 s : an item is still not scanned or returned -> STAFF ALERT (smartcart/alerts/<id> + alertSummary), shown on the
//         dashboard, the shopper app and the cart screen. It disappears when the item is put back or scanned.
// "Scanned" is read from Firebase: unscanned = taken - returned - claimed - flagged. If that count is lower than the
// items the shelf remembers, the oldest ones were scanned (first in, first out).
Watch* watchFind(const char* id, bool create) {
  for (int i = 0; i < WATCH_MAX; i++)
    if (watches[i].used && strcmp(watches[i].id, id) == 0) return &watches[i];
  if (!create) return nullptr;
  for (int i = 0; i < WATCH_MAX; i++) {
    if (!watches[i].used) {
      memset(&watches[i], 0, sizeof(Watch));
      watches[i].used = true;
      strncpy(watches[i].id, id, sizeof(watches[i].id) - 1);
      return &watches[i];
    }
  }
  return nullptr;
}

void watchReset(Watch* w) {
  taskENTER_CRITICAL(&watchMux);
  w->outstanding = 0;
  w->nPick = 0;
  w->warned15 = false;
  w->alert60 = false;
  w->alertQty = 0;
  w->checking = false;
  taskEXIT_CRITICAL(&watchMux);
}

void pickPush(Watch* w, int n) {
  taskENTER_CRITICAL(&watchMux);
  if (w->nPick == 0) w->warned15 = false;
  for (int k = 0; k < n; k++) if (w->nPick < MAXP) w->pick[w->nPick++] = millis();
  w->outstanding += n;
  taskEXIT_CRITICAL(&watchMux);
}

void pickDropOldest(Watch* w, int n) {               // n items were put back or scanned: they were the oldest ones
  taskENTER_CRITICAL(&watchMux);
  int d = (n > w->nPick) ? w->nPick : n;
  for (int i = 0; i + d < w->nPick; i++) w->pick[i] = w->pick[i + d];
  w->nPick -= d;
  w->outstanding -= n;
  if (w->outstanding < 0) w->outstanding = 0;
  if (w->nPick > 0 && (millis() - w->pick[0]) < UNSCANNED_MS) w->warned15 = false;   // the next item is younger
  taskEXIT_CRITICAL(&watchMux);
}

int pickOverdue(Watch* w, unsigned long ms) {        // items that have been off the shelf for at least ms
  int c = 0;
  unsigned long now = millis();
  taskENTER_CRITICAL(&watchMux);
  for (int i = 0; i < w->nPick; i++) if ((now - w->pick[i]) >= ms) c++;
  taskEXIT_CRITICAL(&watchMux);
  return c;
}

void watchPickup(const char* id, int n) {
  Watch* w = watchFind(id, true);
  if (!w) return;
  pickPush(w, n);
  Serial.printf("[WATCH] %s lifted (%d off the shelf) - scan it within %d s, staff alert after %d s\n",
                nameForId(id), w->outstanding, UNSCANNED_MS / 1000, STAFF_ALERT_MS / 1000);
}

void watchClear(const char* id, int n) {              // item put back on the shelf (oldest first)
  Watch* w = watchFind(id, false);
  if (!w) return;
  pickDropOldest(w, n);
  if (w->outstanding <= 0 && !w->alert60) watchReset(w);
  else if (w->alert60) w->nextRecheck = 0;            // an alert is open: re-check now so the popup clears quickly
  Serial.printf("[WATCH] %s put back - %d still off the shelf\n", nameForId(id), w->outstanding);
}

void watchLoop() {
  unsigned long now = millis();
  for (int i = 0; i < WATCH_MAX; i++) {
    Watch& w = watches[i];
    if (!w.used || w.checking) continue;
    if (w.alert60) {                                  // alert open: keep looking until it is resolved
      if ((long)(now - w.nextRecheck) >= 0 && queueJob(JOB_CHECK, w.id, 1, "")) {
        w.checking = true;
        w.nextRecheck = now + RECHECK_MS;
      }
      continue;
    }
    if (w.nPick <= 0) continue;
    unsigned long age = now - w.pick[0];              // the oldest unscanned item
    if (age >= STAFF_ALERT_MS) {
      if (queueJob(JOB_CHECK, w.id, 60, "")) {
        w.checking = true;
        Serial.printf("[WATCH] 1 minute up for %s - checking for a staff alert\n", nameForId(w.id));
      }
    } else if (!w.warned15 && age >= UNSCANNED_MS) {
      if (queueJob(JOB_CHECK, w.id, 15, "")) {
        w.checking = true;
        Serial.printf("[WATCH] 15 s up for %s - asking Firebase whether the cart scanned it\n", nameForId(w.id));
      }
    }
  }
}

void saveEvent(const char* type, const char* msg) {
  FirebaseJson e;
  e.set("cartId", SHELF_ID);
  e.set("type", type);
  e.set("message", msg);
  e.set("timestamp/.sv", "timestamp");
  if (!Firebase.RTDB.pushJSON(&fbdo, "smartcart/events", &e))
    Serial.println("[FB] event could not be saved: " + fbdo.errorReason());
}

// Fills the alert node the dashboard and the shopper app read: smartcart/alerts/<productId>
void fillAlertNode(FirebaseJson &a, const FbJob &j, Watch* w, int qty, const char* msg) {
  a.set("productId", j.id);
  a.set("name", nameForId(j.id));
  a.set("qty", qty);
  a.set("status", "active");
  a.set("message", msg);
  a.set("shelfId", SHELF_ID);
  time_t t = time(nullptr);
  if (t > 1700000000 && w && w->nPick > 0)            // when the oldest overdue item was lifted (epoch ms)
    a.set("takenAt", (double)t * 1000.0 - (double)(millis() - w->pick[0]));
}

bool runJob(const FbJob &j) {
  if (!Firebase.ready()) return false;
  FirebaseJson js;
  String counter = String("smartcart/shelfCounters/") + j.id;
  switch (j.type) {
    case JOB_TAKEN:
      js.set("taken/.sv/increment", j.n);
      if (!Firebase.RTDB.updateNode(&fbdo, counter, &js)) return false;
      Serial.printf("[FB] saved: %s taken +%d\n", nameForId(j.id), j.n);
      return true;
    case JOB_RETURNED:
      js.set("returned/.sv/increment", j.n);
      js.set("returnedAt/.sv", "timestamp");           // the cart uses this to catch a put-back that happened a moment early
      if (!Firebase.RTDB.updateNode(&fbdo, counter, &js)) return false;
      Serial.printf("[FB] saved: %s returned +%d\n", nameForId(j.id), j.n);
      return true;
    case JOB_CHECK: {
      int stage = j.n;                                // 15 = first warning, 60 = staff alert, 1 = re-check of an open alert
      int taken = 0, returned = 0, claimed = 0, flagged = 0;
      if (Firebase.RTDB.getJSON(&fbdo, counter)) {
        FirebaseJson &cj = fbdo.jsonObject();
        FirebaseJsonData cd;
        if (cj.get(cd, "taken"))    taken    = cd.intValue;
        if (cj.get(cd, "returned")) returned = cd.intValue;
        if (cj.get(cd, "claimed"))  claimed  = cd.intValue;
        if (cj.get(cd, "flagged"))  flagged  = cd.intValue;
      } else if (fbdo.errorReason().indexOf("not exist") < 0) {
        return false;                                 // network problem: try again
      }
      int unscanned = taken - returned - claimed - flagged;
      int u = (unscanned > 0) ? unscanned : 0;
      Watch* w = watchFind(j.id, false);
      Serial.printf("[CHECK %d] %s taken=%d returned=%d claimed=%d flagged=%d -> %d unscanned (shelf remembers %d)\n",
                    stage, nameForId(j.id), taken, returned, claimed, flagged, unscanned, w ? w->nPick : 0);
      if (w && w->nPick > u) pickDropOldest(w, w->nPick - u);   // the oldest ones were scanned
      String alertPath = String("smartcart/alerts/") + j.id;
      char msg[110];
      int overdue = w ? pickOverdue(w, STAFF_ALERT_MS) : 0;
      int qty = (overdue < u) ? overdue : u;

      if (stage == 15) {
        if (w && w->nPick > 0) {
          snprintf(msg, sizeof(msg), "%s lifted from the shelf but not scanned within %d s (x%d)", nameForId(j.id), UNSCANNED_MS / 1000, w->nPick);
          Serial.printf("\n!!!! WARNING: %s\n\n", msg);
          PLAY_WARN();
          saveEvent("unscanned_pickup", msg);
          w->warned15 = true;
          w->checking = false;
        } else {
          Serial.printf("[CHECK] %s was scanned in time - OK\n", nameForId(j.id));
          if (w) watchReset(w);
        }
        return true;
      }

      if (stage == 60) {
        if (w && qty > 0) {
          snprintf(msg, sizeof(msg), "%s has been off the shelf for over a minute and is not scanned (x%d)", nameForId(j.id), qty);
          Serial.printf("\n!!!! STAFF ALERT: %s\n\n", msg);
          FirebaseJson a;
          fillAlertNode(a, j, w, qty, msg);
          a.set("since/.sv", "timestamp");
          if (!Firebase.RTDB.updateNode(&fbdo, alertPath, &a)) return false;
          PLAY_ALERT();
          saveEvent("unscanned_pickup_staff", msg);
          w->alert60 = true;
          w->alertQty = qty;
          w->nextRecheck = millis() + RECHECK_MS;
          w->checking = false;
          w->warned15 = true;
          queueJob(JOB_SUMMARY, "", 0, "");
        } else {
          Serial.printf("[CHECK] %s - nothing is overdue now\n", nameForId(j.id));
          if (w) { if (w->nPick == 0) watchReset(w); else w->checking = false; }
        }
        return true;
      }

      // stage 1: an alert is open - is it resolved, bigger, or did staff dismiss it?
      if (!w) return true;
      bool alertExists = Firebase.RTDB.getString(&fbdo, alertPath + "/status");
      if (!alertExists && fbdo.errorReason().indexOf("not exist") < 0) return false;   // network problem: try again
      if (!alertExists) {                             // staff dismissed it on the dashboard
        if (u > 0) {
          FirebaseJson f;
          f.set("flagged/.sv/increment", u);           // book it as reported so it never warns again
          if (!Firebase.RTDB.updateNode(&fbdo, counter, &f)) return false;
        }
        Serial.printf("[ALERT] %s dismissed by staff\n", nameForId(j.id));
        watchReset(w);
        queueJob(JOB_SUMMARY, "", 0, "");
        return true;
      }
      if (qty <= 0) {                                 // item(s) put back or scanned: the popup goes away
        if (!Firebase.RTDB.deleteNode(&fbdo, alertPath)) return false;
        snprintf(msg, sizeof(msg), "%s is back to normal (returned or scanned)", nameForId(j.id));
        Serial.printf("[ALERT] %s\n", msg);
        saveEvent("unscanned_pickup_cleared", msg);
        w->alert60 = false;
        w->alertQty = 0;
        if (w->nPick == 0) watchReset(w);
        else { w->checking = false; if ((millis() - w->pick[0]) >= UNSCANNED_MS) w->warned15 = true; }
        queueJob(JOB_SUMMARY, "", 0, "");
      } else if (qty != w->alertQty) {                // another item went overdue, or one was resolved
        snprintf(msg, sizeof(msg), "%s has been off the shelf for over a minute and is not scanned (x%d)", nameForId(j.id), qty);
        FirebaseJson a;
        fillAlertNode(a, j, w, qty, msg);
        if (!Firebase.RTDB.updateNode(&fbdo, alertPath, &a)) return false;
        if (qty > w->alertQty) { Serial.printf("\n!!!! STAFF ALERT: %s\n\n", msg); PLAY_ALERT(); saveEvent("unscanned_pickup_staff", msg); }
        w->alertQty = qty;
        w->checking = false;
        queueJob(JOB_SUMMARY, "", 0, "");
      } else {
        w->checking = false;                          // unchanged: look again later
      }
      return true;
    }
    case JOB_SUMMARY: {
      // One small node the cart screen / app can read: smartcart/alertSummary {count, text, list}
      //   list = "Name|qty|productId;Name|qty|productId"
      int count = 0, firstQty = 0;
      char first[40] = "";
      String list = "";
      for (int i = 0; i < WATCH_MAX; i++) {
        if (watches[i].used && watches[i].alert60) {
          if (count == 0) { strncpy(first, nameForId(watches[i].id), sizeof(first) - 1); firstQty = watches[i].alertQty; }
          if (count < 4) {
            if (list.length()) list += ";";
            list += String(nameForId(watches[i].id)) + "|" + String(watches[i].alertQty) + "|" + String(watches[i].id);
          }
          count++;
        }
      }
      if (count == 0) {
        Firebase.RTDB.deleteNode(&fbdo, "smartcart/alertSummary");
        Serial.println("[ALERT] all clear");
        return true;
      }
      char txt[100];
      if (count == 1) {
        if (firstQty > 1) snprintf(txt, sizeof(txt), "Return or scan: %s x%d", first, firstQty);
        else              snprintf(txt, sizeof(txt), "Return or scan: %s", first);
      } else {
        snprintf(txt, sizeof(txt), "Return or scan: %s +%d more", first, count - 1);
      }
      FirebaseJson sj;
      sj.set("count", count);
      sj.set("text", txt);
      sj.set("list", list);
      sj.set("updatedAt/.sv", "timestamp");
      return Firebase.RTDB.setJSON(&fbdo, "smartcart/alertSummary", &sj);
    }
    case JOB_RESET: {                                   // zero this product's counters and clear its alert
      FirebaseJson z;
      z.set("taken", 0);
      z.set("returned", 0);
      z.set("claimed", 0);
      z.set("flagged", 0);
      if (!Firebase.RTDB.setJSON(&fbdo, counter, &z)) return false;
      Firebase.RTDB.deleteNode(&fbdo, String("smartcart/alerts/") + j.id);
      return true;
    }
    case JOB_STOCK: {
      int cell = j.extra / NUM_PRODUCTS;
      int prod = j.extra % NUM_PRODUCTS;
      js.set("qty", j.n);
      js.set("max", MAX_QUANTITY[prod]);
      js.set("cell", cell + 1);
      js.set("name", PRODUCT_NAMES[cell][prod]);
      js.set("unitWeight", PRODUCT_WEIGHT_G[cell][prod]);
      js.set("updatedAt/.sv", "timestamp");
      return Firebase.RTDB.updateNode(&fbdo, String("smartcart/shelves/") + SHELF_ID + "/inventory/" + j.id, &js);
    }
    case JOB_BEAT:
      js.set("online", true);
      js.set("rssi", WiFi.RSSI());
      js.set("lastSeen/.sv", "timestamp");
      return Firebase.RTDB.updateNode(&fbdo, String("smartcart/shelves/") + SHELF_ID, &js);
  }
  return true;
}

void fbTask(void*) {
  // Sign in HERE, in the background, and keep retrying: a slow or broken Firebase connection can
  // never block the weight readings or the serial commands.
  fbconfig.api_key = API_KEY;
  fbconfig.database_url = DATABASE_URL;
  fbconfig.timeout.socketConnection = 30 * 1000;
  fbconfig.timeout.serverResponse   = 30 * 1000;
  fbconfig.timeout.sslHandshake     = 120 * 1000;
  fbdo.setBSSLBufferSize(4096, 1024);
  fbdo.setResponseSize(2048);
  for (;;) {
    if (WiFi.status() == WL_CONNECTED && Firebase.signUp(&fbconfig, &fbauth, "", "")) break;
    Serial.println("[FB] sign-in not done yet (" + String(fbconfig.signer.signupError.message.c_str()) + ") - retrying in 10 s");
    vTaskDelay(pdMS_TO_TICKS(10000));
  }
  fbconfig.token_status_callback = tokenStatusCallback;
  Firebase.begin(&fbconfig, &fbauth);
  Firebase.reconnectWiFi(true);
  fbSignedIn = true;
  Serial.println("[FB] signed in - uploading queued data");

  FbJob j;
  for (;;) {
    if (xQueueReceive(fbQueue, &j, pdMS_TO_TICKS(500)) == pdTRUE) {
      bool ok = false;
      for (int t = 1; t <= 5 && !ok; t++) {
        ok = runJob(j);
        if (!ok) {
          Serial.printf("[FB] job %d for %s failed (try %d): %s\n", j.type, j.id, t, fbdo.errorReason().c_str());
          vTaskDelay(pdMS_TO_TICKS(2000));
        }
      }
      if (!ok) {
        Serial.printf("[FB] job %d for %s dropped after 5 tries\n", j.type, j.id);
        Watch* w = watchFind(j.id, false);
        if (w) w->checking = false;                  // let the check be tried again
      }
    }
  }
}

void startNetwork() {
  Serial.setTimeout(50);
  Serial.println();
  if (strcmp(WIFI_PASS, "YOUR_PASSWORD") == 0) Serial.println("[NET] !! WIFI_PASS is still the placeholder - edit it at the top of the sketch.");
  if (strcmp(API_KEY, "YOUR_WEB_API_KEY") == 0) Serial.println("[NET] !! API_KEY is still the placeholder - edit it at the top of the sketch.");
  Serial.println("[NET] connecting Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000UL) { delay(500); Serial.print("."); }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[NET] Wi-Fi FAILED - check the SSID/password (2.4 GHz only), then reset the board.");
    return;
  }
  Serial.println("[NET] Wi-Fi connected, IP " + WiFi.localIP().toString());
  WiFi.setSleep(false);
  configTime(19800, 0, "pool.ntp.org", "time.google.com");

  fbQueue = xQueueCreate(32, sizeof(FbJob));
  // Priority 0 (same as the idle task) so the CPU-heavy secure handshakes cannot starve it
  // ("Task watchdog got triggered").
  xTaskCreatePinnedToCore(fbTask, "fbTask", 20480, nullptr, 0, nullptr, 0);
  publishAllStock();
}

// Called for every stock change the load cells detect (see sendEvent below).
void onShelfEvent(int cell, int product, const char* action, int changed, int remaining) {
  const char* id = PRODUCT_ID[cell][product];
  queueStockJob(id, remaining, cell, product);     // keep the stock count in Firebase (staff inventory page)
  if (!liveMode) {
    Serial.printf("[STOCKING] %s %s x%d (not reported to the cart)\n", nameForId(id), action, changed);
    return;
  }
  Serial.printf("[LIVE] %s %s x%d -> reported to Firebase as id '%s'\n", nameForId(id), action, changed, id);
  if (strcmp(action, "REMOVED") == 0) {            // item LIFTED off the shelf
    queueJob(JOB_TAKEN, id, changed, "");
    watchPickup(id, changed);
  } else {                                         // item PUT BACK on the shelf
    queueJob(JOB_RETURNED, id, changed, "");
    watchClear(id, changed);
  }
}

void printStatus() {
  Serial.println();
  Serial.println("---------------- STATUS ----------------");
  Serial.printf("Mode     : %s\n", liveMode ? "LIVE (reporting lifts and put-backs)" : "STOCKING (not reporting to the cart)");
  Serial.printf("Wi-Fi    : %s\n", WiFi.status() == WL_CONNECTED ? "connected" : "NOT connected");
  Serial.printf("Firebase : %s, %d job(s) waiting\n", fbSignedIn ? "signed in" : "not signed in yet",
                fbQueue ? (int)uxQueueMessagesWaiting(fbQueue) : 0);
  for (int i = 0; i < WATCH_MAX; i++) {
    if (watches[i].used && (watches[i].outstanding > 0 || watches[i].alert60))
      Serial.printf("Watching : %s  off-shelf=%d  warned15=%d  STAFF ALERT=%d\n", nameForId(watches[i].id),
                    watches[i].outstanding, watches[i].warned15, watches[i].alert60);
  }
  Serial.println("----------------------------------------");
}

// Type a command and press ENTER (Serial Monitor line ending: Newline).
//   l or 1 = go LIVE   s or 0 = STOCKING mode   p = re-send inventory   status = show state
//   add N / take N = TEST without the load cells (N = product number 1-12), go LIVE first
void handleSerialCommands() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  line.toLowerCase();
  if (line.length() == 0) return;
  if (line == "l" || line == "live" || line == "1") {
    liveMode = true;
    Serial.println("[MODE] LIVE - lifts and put-backs are now reported.");
    publishAllStock();
  } else if (line == "s" || line == "stock" || line == "0") {
    liveMode = false;
    Serial.println("[MODE] STOCKING - changes only update stock (use this when refilling the shelf).");
  } else if (line == "p") {
    publishAllStock();
  } else if (line == "reset") {
    // Zero every product's counters and clear all alerts. Use it at the start of a demo, when NO cart is holding items.
    taskENTER_CRITICAL(&watchMux);
    for (int i = 0; i < WATCH_MAX; i++) memset(&watches[i], 0, sizeof(Watch));
    taskEXIT_CRITICAL(&watchMux);
    for (int c = 0; c < NUM_CELLS; c++) for (int p = 0; p < NUM_PRODUCTS; p++) queueJob(JOB_RESET, PRODUCT_ID[c][p], 0, "");
    queueJob(JOB_SUMMARY, "", 0, "");
    Serial.println("[RESET] counters and alerts are being cleared in Firebase");
  } else if (line == "status") {
    printStatus();
  } else if (line.startsWith("add ") || line.startsWith("take ")) {
    bool isAdd = line.startsWith("add ");
    int n = line.substring(isAdd ? 4 : 5).toInt();
    if (n < 1 || n > NUM_CELLS * NUM_PRODUCTS) {
      Serial.println("Use a product number from 1 to 12 (Cell 1 A,B,C = 1,2,3 ... Cell 4 A,B,C = 10,11,12).");
    } else {
      int cell = (n - 1) / NUM_PRODUCTS;
      int prod = (n - 1) % NUM_PRODUCTS;
      Serial.printf("[TEST] pretending %s of %s (no weight involved)\n", isAdd ? "a PUT-BACK" : "a LIFT", PRODUCT_NAMES[cell][prod]);
      onShelfEvent(cell, prod, isAdd ? "ADDED" : "REMOVED", 1, quantities[cell][prod]);
    }
  } else {
    Serial.println("I did not understand '" + line + "'. Commands: l or 1 (live)   s or 0 (stocking)   p (send inventory)   status   reset   add N / take N (test)");
  }
}

void handleBuzzer() {
  buzzerTick();
  static unsigned long nextReminder = 0;
  bool alertOn = false;
  for (int i = 0; i < WATCH_MAX; i++) if (watches[i].used && watches[i].alert60) alertOn = true;
  if (alertOn && millis() >= nextReminder) { nextReminder = millis() + 12000; if (nextReminder != 0) PLAY_WARN(); }   // quiet reminder every 12 s
  if (!alertOn) nextReminder = 0;
  digitalWrite(LED_PIN, (bzSounding || alertOn) ? HIGH : LOW);
}

// ------------------------------------------------------------
// Helper functions
// ------------------------------------------------------------

void waitForEnter() {
  // Wait for a fresh Enter press. Use Newline or Both NL & CR in Serial Monitor.
  while (Serial.available()) {
    Serial.read();
  }
  while (!Serial.available()) {
    delay(10);
  }
  delay(80);  // Allow CR/LF characters to arrive together.
  while (Serial.available()) {
    Serial.read();
  }
}

// Wait for a fresh Enter while showing LIVE calibrated weight in grams only.
// No raw counts are displayed to the user.
void waitForEnterWithLiveReading(int cell, bool showGrams) {
  (void)showGrams; // Kept in the signature so existing calls remain compatible.

  while (Serial.available()) {
    Serial.read();
  }

  unsigned long lastPrint = 0;
  while (!Serial.available()) {
    unsigned long now = millis();
    if (now - lastPrint >= 500UL) {
      lastPrint = now;
      if (scales[cell].is_ready()) {
        float liveGrams = scales[cell].get_units(3);
        Serial.print("[LIVE] Cell ");
        Serial.print(cell + 1);
        Serial.print(" measured weight = ");
        Serial.print(liveGrams, 1);
        Serial.println(" g");
      } else {
        Serial.print("[WAIT] Cell ");
        Serial.print(cell + 1);
        Serial.println(" HX711 is not ready; check wiring if this continues.");
      }
    }
    delay(10);
  }

  delay(80);  // Allow CR/LF characters to arrive together.
  while (Serial.available()) {
    Serial.read();
  }
}

// First-time calibration: the HX711 does not know grams until its scale factor
// has been calculated. Detect the placed 100 g reference internally (no raw
// counts are printed), calculate the factor, then show grams live so the user
// can press Enter when the display is stable near 100.0 g.
bool waitForReferenceWeightAndCalibrateExisting(int cell) {
  // More reliable first calibration:
  // 1) tare the empty plate (done in setup)
  // 2) user places the known reference weight and presses Enter
  // 3) calculate the scale factor from the change since tare
  // 4) show live GRAMS so the user can confirm the reading
  // Raw counts are kept internal and are never printed.
  const float MIN_RAW_CHANGE = 100.0;

  Serial.println();
  Serial.print("CELL ");
  Serial.print(cell + 1);
  Serial.println(" - PLACE REFERENCE WEIGHT");
  Serial.print("Place the known ");
  Serial.print(CALIBRATION_WEIGHT_G[cell], 1);
  Serial.print(" g weight on Cell ");
  Serial.print(cell + 1);
  Serial.println(" ONLY.");
  Serial.println("Keep the fixed 70 g plate attached.");
  Serial.println("After placing the weight and waiting for it to settle, press ENTER.");
  Serial.println("The ESP32 will then calibrate and display the reading in grams.");
  Serial.flush();

  // Wait for the user to place the physical reference weight.
  waitForEnter();

  if (!scales[cell].wait_ready_timeout(2500)) {
    Serial.print("[CALIBRATION ERROR] Cell ");
    Serial.print(cell + 1);
    Serial.println(" HX711 is not ready when sampling the reference weight.");
    return false;
  }

  // Since tare() was performed with the empty platform (including the fixed
  // 70 g plate), get_value() reports the change from that empty baseline.
  // This value is used internally only, not shown as raw counts.
  float referenceChange = scales[cell].get_value(20);

  if (!isfinite(referenceChange) || fabs(referenceChange) < MIN_RAW_CHANGE) {
    Serial.println();
    Serial.print("[CALIBRATION ERROR] Cell ");
    Serial.print(cell + 1);
    Serial.println(" did not show a sufficient change when the 100 g weight was placed.");
    Serial.println("The HX711 may be responding, but the load-cell signal is not changing.");
    Serial.println("Check this cell's four load-cell wires at E+, E-, A+ and A-, and check its mounting.");
    Serial.println("The code will skip this cell and continue to the next sensor.");
    return false;
  }

  calibrationFactor[cell] = referenceChange / CALIBRATION_WEIGHT_G[cell];
  scales[cell].set_scale(calibrationFactor[cell]);
  delay(150);

  float firstGrams = scales[cell].get_units(10);
  if (!isfinite(firstGrams)) {
    Serial.print("[CALIBRATION ERROR] Cell ");
    Serial.print(cell + 1);
    Serial.println(" produced an invalid gram reading. Skipping this cell.");
    return false;
  }

  Serial.println();
  Serial.print("[OK] Cell ");
  Serial.print(cell + 1);
  Serial.println(" calibration factor calculated using your reference weight.");
  Serial.println("LIVE WEIGHT IN GRAMS (keep the 100 g weight on the plate):");
  Serial.print("[LIVE] Cell ");
  Serial.print(cell + 1);
  Serial.print(" measured weight = ");
  Serial.print(firstGrams, 1);
  Serial.println(" g");
  Serial.println("The reading should be close to 100.0 g.");
  Serial.println("Wait until the gram reading is stable, then press ENTER to continue.");
  Serial.flush();

  // Keep showing only grams until the user confirms the reference is stable.
  waitForEnterWithLiveReading(cell, true);
  return true;
}


// Cell 2 only: keep its existing working calibration path unchanged. Allow
// the 100 g load to settle after Enter, then average more HX711 samples before
// calculating the factor.
bool calibrateCell2Or4(int cell) {
  Serial.println();
  Serial.print("CELL ");
  Serial.print(cell + 1);
  Serial.println(" - PLACE REFERENCE WEIGHT");
  Serial.print("Place the known ");
  Serial.print(CALIBRATION_WEIGHT_G[cell], 1);
  Serial.print(" g weight on Cell ");
  Serial.print(cell + 1);
  Serial.println(" ONLY.");
  Serial.println("Keep the fixed 70 g plate attached.");
  Serial.println("When the weight is on the plate and settled, press ENTER.");
  Serial.println("The program will wait briefly, then calculate calibration internally.");
  Serial.flush();

  waitForEnter();

  if (!scales[cell].wait_ready_timeout(3000)) {
    Serial.print("[CALIBRATION ERROR] Cell ");
    Serial.print(cell + 1);
    Serial.println(" HX711 is not ready. Skipping this cell and continuing.");
    return false;
  }

  // Let mechanical vibration settle. The existing tare offset is retained.
  delay(2500);
  if (!scales[cell].wait_ready_timeout(3000)) {
    Serial.print("[CALIBRATION ERROR] Cell ");
    Serial.print(cell + 1);
    Serial.println(" stopped responding while settling. Skipping this cell.");
    return false;
  }

  // Average several measurements. These readings are internal only; no raw
  // counts are printed. Negative changes are valid if the load-cell direction
  // is reversed, because the calculated factor carries the same sign.
  float referenceChange = scales[cell].get_value(40);
  if (!isfinite(referenceChange) || fabs(referenceChange) < 100.0) {
    delay(700);
    if (scales[cell].wait_ready_timeout(2000)) {
      referenceChange = scales[cell].get_value(60);
    }
  }

  if (!isfinite(referenceChange) || fabs(referenceChange) < 100.0) {
    Serial.println();
    Serial.print("[CALIBRATION ERROR] Cell ");
    Serial.print(cell + 1);
    Serial.println(" did not detect enough change from the 100 g weight.");
    Serial.println("Check that this load cell is wired to its own HX711 at E+, E-, A+ and A-.");
    Serial.println("Also check DT/SCK pins and that the platform can flex under the weight.");
    Serial.println("Cell 1 and Cell 3 calibration code is unchanged. Moving to the next cell.");
    return false;
  }

  calibrationFactor[cell] = referenceChange / CALIBRATION_WEIGHT_G[cell];
  scales[cell].set_scale(calibrationFactor[cell]);
  delay(250);

  float firstGrams = scales[cell].get_units(15);
  if (!isfinite(firstGrams)) {
    Serial.print("[CALIBRATION ERROR] Cell ");
    Serial.print(cell + 1);
    Serial.println(" produced an invalid gram reading. Skipping this cell.");
    return false;
  }

  Serial.println();
  Serial.print("[OK] Cell ");
  Serial.print(cell + 1);
  Serial.println(" calibration factor calculated using the 100 g reference.");
  Serial.println("LIVE WEIGHT IN GRAMS (keep the 100 g weight on the plate):");
  Serial.print("[LIVE] Cell ");
  Serial.print(cell + 1);
  Serial.print(" measured weight = ");
  Serial.print(firstGrams, 1);
  Serial.println(" g");
  Serial.println("The reading should be close to 100.0 g.");
  Serial.println("Wait until the gram reading is stable, then press ENTER.");
  Serial.flush();
  waitForEnterWithLiveReading(cell, true);
  return true;
}

// Calibration routing:
// Cell 2 (index 1) keeps its existing working calibration routine.
// Cells 1, 3, and 4 use the same existing calibration routine.
// Cell 4 has NO separate/additional calibration steps.
bool waitForReferenceWeightAndCalibrate(int cell) {
  if (cell == 1) {
    return calibrateCell2Or4(cell);
  }
  return waitForReferenceWeightAndCalibrateExisting(cell);
}

void makeProductId(int cell, int product, char* output, size_t outputSize) {
  snprintf(output, outputSize, "Cell%d_Product%c", cell + 1, 'A' + product);
}

void sendState(int cell, int product, int quantity, float weight) {
  char productId[32];
  makeProductId(cell, product, productId, sizeof(productId));

  Serial.print("STATE,");
  Serial.print(productId);
  Serial.print(",");
  Serial.print(quantity);
  Serial.print(",");
  Serial.println(weight, 1);
}

void sendEvent(int cell, int product, const char* action,
               int changed, int remaining, float shelfWeight) {
  char productId[32];
  makeProductId(cell, product, productId, sizeof(productId));

  Serial.print("EVENT,");
  Serial.print(productId);
  Serial.print(",");
  Serial.print(action);
  Serial.print(",");
  Serial.print(changed);
  Serial.print(",");
  Serial.print(remaining);
  Serial.print(",");
  Serial.println(shelfWeight, 1);

  onShelfEvent(cell, product, action, changed, remaining);   // report to Firebase
}

float expectedStockWeight(int cell) {
  float total = 0;
  for (int p = 0; p < NUM_PRODUCTS; p++) {
    total += quantities[cell][p] * PRODUCT_WEIGHT_G[cell][p];
  }
  return total;
}

void printCurrentStock(int cell, float measuredWeight) {
  Serial.println();
  Serial.println("========================================");
  Serial.print("CURRENT STOCK - CELL ");
  Serial.println(cell + 1);
  Serial.println("========================================");

  for (int p = 0; p < NUM_PRODUCTS; p++) {
    Serial.print(PRODUCT_NAMES[cell][p]);
    Serial.print(" (");
    Serial.print(PRODUCT_WEIGHT_G[cell][p], 1);
    Serial.print(" g): ");

    if (quantities[cell][p] > 0) {
      Serial.print("IN x ");
      Serial.println(quantities[cell][p]);
    } else {
      Serial.println("OUT");
    }
  }

  Serial.println("----------------------------------------");
  Serial.print("Expected product weight: ");
  Serial.print(expectedStockWeight(cell), 1);
  Serial.println(" g");
  Serial.print("Measured cell weight:    ");
  Serial.print(measuredWeight, 1);
  Serial.println(" g");
  Serial.println("========================================");
}

void printUnknownChange(int cell, const char* action,
                        float measuredChange, float closestError) {
  Serial.println();
  Serial.print("UNKNOWN ITEM CHANGE - CELL ");
  Serial.println(cell + 1);
  Serial.print("Change direction: ");
  Serial.println(action);
  Serial.print("Measured change: ");
  Serial.print(measuredChange, 1);
  Serial.println(" g");
  Serial.print("Closest match error: ");
  Serial.print(closestError, 1);
  Serial.println(" g");
  Serial.println("No stock quantity changed for this unmatched weight.");
}

// ------------------------------------------------------------
// Product weight matching
// ------------------------------------------------------------

void findBestCombination(int cell, float measuredChange,
                         const int limits[NUM_PRODUCTS],
                         int bestMatch[NUM_PRODUCTS],
                         float &bestError, float &secondBestError) {
  bestError = 999999.0;
  secondBestError = 999999.0;
  bestMatch[0] = 0;
  bestMatch[1] = 0;
  bestMatch[2] = 0;

  for (int a = 0; a <= limits[0]; a++) {
    for (int b = 0; b <= limits[1]; b++) {
      for (int c = 0; c <= limits[2]; c++) {
        if (a == 0 && b == 0 && c == 0) continue;

        float expected =
            a * PRODUCT_WEIGHT_G[cell][0] +
            b * PRODUCT_WEIGHT_G[cell][1] +
            c * PRODUCT_WEIGHT_G[cell][2];
        float error = fabs(measuredChange - expected);

        if (error < bestError) {
          secondBestError = bestError;
          bestError = error;
          bestMatch[0] = a;
          bestMatch[1] = b;
          bestMatch[2] = c;
        } else if (error < secondBestError) {
          // Includes equal-error alternatives, which must be treated as ambiguous.
          secondBestError = error;
        }
      }
    }
  }
}

bool isAmbiguousMatch(float bestError, float secondBestError) {
  return secondBestError <= MATCH_TOLERANCE_G &&
         secondBestError < 999998.0 &&
         (secondBestError - bestError) < MIN_MATCH_MARGIN_G;
}

void printAmbiguousChange(int cell, const char* action,
                          float measuredChange, float bestError,
                          float secondBestError) {
  Serial.println();
  Serial.print("AMBIGUOUS PRODUCT CHANGE - CELL ");
  Serial.println(cell + 1);
  Serial.print("Change direction: ");
  Serial.println(action);
  Serial.print("Measured change: ");
  Serial.print(measuredChange, 1);
  Serial.println(" g");
  Serial.print("Best match error: ");
  Serial.print(bestError, 1);
  Serial.println(" g");
  Serial.print("Second match error: ");
  Serial.print(secondBestError, 1);
  Serial.println(" g");
  Serial.println("More than one product combination is too close to identify safely.");
  Serial.println("No stock quantity changed. Try handling one product combination at a time.");
}

// ------------------------------------------------------------
// Product added to a cell
// ------------------------------------------------------------

void detectAdded(int cell, float measuredChange, float currentWeight) {
  int limits[NUM_PRODUCTS] = {
    MAX_QUANTITY[0] - quantities[cell][0],
    MAX_QUANTITY[1] - quantities[cell][1],
    MAX_QUANTITY[2] - quantities[cell][2]
  };
  int matched[NUM_PRODUCTS] = {0, 0, 0};
  float bestError = 999999.0;
  float secondBestError = 999999.0;

  findBestCombination(cell, measuredChange, limits, matched,
                      bestError, secondBestError);

  if (bestError > MATCH_TOLERANCE_G) {
    printUnknownChange(cell, "WEIGHT INCREASED", measuredChange, bestError);
    return;
  }

  if (isAmbiguousMatch(bestError, secondBestError)) {
    printAmbiguousChange(cell, "WEIGHT INCREASED", measuredChange,
                         bestError, secondBestError);
    return;
  }

  Serial.println();
  Serial.print("******** ITEMS ADDED - CELL ");
  Serial.print(cell + 1);
  Serial.println(" ********");

  for (int p = 0; p < NUM_PRODUCTS; p++) {
    if (matched[p] > 0) {
      quantities[cell][p] += matched[p];
      Serial.print(PRODUCT_NAMES[cell][p]);
      Serial.print(" ADDED: ");
      Serial.println(matched[p]);
      sendEvent(cell, p, "ADDED", matched[p], quantities[cell][p], currentWeight);
    }
  }

  printCurrentStock(cell, currentWeight);
}

// ------------------------------------------------------------
// Product removed from a cell
// ------------------------------------------------------------

void detectRemoved(int cell, float measuredChange, float currentWeight) {
  int limits[NUM_PRODUCTS] = {
    quantities[cell][0],
    quantities[cell][1],
    quantities[cell][2]
  };
  int matched[NUM_PRODUCTS] = {0, 0, 0};
  float bestError = 999999.0;
  float secondBestError = 999999.0;

  findBestCombination(cell, measuredChange, limits, matched,
                      bestError, secondBestError);

  if (bestError > MATCH_TOLERANCE_G) {
    printUnknownChange(cell, "WEIGHT DECREASED", measuredChange, bestError);
    return;
  }

  if (isAmbiguousMatch(bestError, secondBestError)) {
    printAmbiguousChange(cell, "WEIGHT DECREASED", measuredChange,
                         bestError, secondBestError);
    return;
  }

  Serial.println();
  Serial.print("******** ITEMS REMOVED - CELL ");
  Serial.print(cell + 1);
  Serial.println(" ********");

  for (int p = 0; p < NUM_PRODUCTS; p++) {
    if (matched[p] > 0) {
      quantities[cell][p] -= matched[p];
      Serial.print(PRODUCT_NAMES[cell][p]);
      Serial.print(" REMOVED: ");
      Serial.println(matched[p]);
      sendEvent(cell, p, "REMOVED", matched[p], quantities[cell][p], currentWeight);
    }
  }

  printCurrentStock(cell, currentWeight);
}

// ------------------------------------------------------------
// Read and process one sensor independently
// ------------------------------------------------------------

void processCell(int cell) {
  // A missing or failed cell never prevents the other cells from updating.
  if (!cellAvailable[cell]) {
    return;
  }
  if (!scales[cell].is_ready()) {
    return;
  }

  float rawWeight = scales[cell].get_units(1);
  if (!isfinite(rawWeight)) {
    return;
  }

  // Remove tiny zero-offset fluctuations before filtering.
  if (fabs(rawWeight) < 2.0) {
    rawWeight = 0.0;
  }

  // Establish the first live reading as this cell's monitoring baseline.
  if (!filterInitialized[cell]) {
    filteredWeight[cell] = rawWeight;
    previousWeight[cell] = rawWeight;
    pendingAnchorWeight[cell] = rawWeight;
    filterInitialized[cell] = true;
    lastMeasuredWeight[cell] = rawWeight;
    hasLiveReading[cell] = true;
    return;
  }

  // Exponential moving average reduces short spikes and platform vibration.
  filteredWeight[cell] += FILTER_ALPHA *
                           (rawWeight - filteredWeight[cell]);
  float currentWeight = filteredWeight[cell];

  lastMeasuredWeight[cell] = currentWeight;
  hasLiveReading[cell] = true;

  float difference = currentWeight - previousWeight[cell];

  // A small change is treated as noise. If a pending movement returns to the
  // baseline, cancel it so a take-and-return action is not recorded.
  if (fabs(difference) <= NOISE_LIMIT_G) {
    if (changePending[cell]) {
      changePending[cell] = false;
      pendingStableSamples[cell] = 0;
    }
    return;
  }

  // Start or continue checking whether the new weight has settled.
  unsigned long now = millis();
  if (!changePending[cell]) {
    changePending[cell] = true;
    pendingAnchorWeight[cell] = currentWeight;
    pendingSince[cell] = now;
    pendingStableSamples[cell] = 1;
    return;
  }

  // If the reading moves outside the stability band, restart the timer.
  // Keep the anchor fixed during each stability window to avoid accepting
  // slow movement as a stable reading.
  if (fabs(currentWeight - pendingAnchorWeight[cell]) <= STABILITY_BAND_G) {
    pendingStableSamples[cell]++;
  } else {
    pendingAnchorWeight[cell] = currentWeight;
    pendingSince[cell] = now;
    pendingStableSamples[cell] = 1;
  }

  if (pendingStableSamples[cell] < REQUIRED_STABLE_SAMPLES ||
      (now - pendingSince[cell]) < REQUIRED_STABLE_TIME_MS) {
    return;
  }

  // Only process the final settled difference from the last confirmed baseline.
  difference = currentWeight - previousWeight[cell];
  if (fabs(difference) > NOISE_LIMIT_G) {
    Serial.println();
    Serial.print("Cell ");
    Serial.print(cell + 1);
    Serial.print(" stable weight change: ");
    Serial.print(difference, 1);
    Serial.println(" g");

    if (difference > 0) {
      detectAdded(cell, difference, currentWeight);
    } else {
      detectRemoved(cell, fabs(difference), currentWeight);
    }

    // Update the baseline even for an unknown/ambiguous change. This prevents
    // the same physical movement from being reported again on the next loop.
    previousWeight[cell] = currentWeight;
  }

  changePending[cell] = false;
  pendingStableSamples[cell] = 0;
}

// ------------------------------------------------------------
// Real-time dashboard: always show all four cells together
// ------------------------------------------------------------

void printLiveDashboard() {
  Serial.println();
  Serial.println(liveMode ? "=========== LIVE WEIGHT MONITOR  [MODE: LIVE] ===========" : "======= LIVE WEIGHT MONITOR  [MODE: STOCKING - type l to go live] =======");
  Serial.println("Refresh: approximately once per second");

  for (int cell = 0; cell < NUM_CELLS; cell++) {
    Serial.print("Cell ");
    Serial.print(cell + 1);
    Serial.print(" | Capacity: ");
    Serial.print(CELL_CAPACITY_G[cell] / 1000.0, 0);
    Serial.print(" kg | ");

    if (!cellAvailable[cell]) {
      Serial.println("SKIPPED / NOT CALIBRATED");
      continue;
    }

    if (hasLiveReading[cell]) {
      Serial.print("Weight: ");
      Serial.print(lastMeasuredWeight[cell], 1);
      Serial.print(" g");
    } else {
      Serial.print("Weight: waiting for first reading");
    }

    Serial.print(" | A:");
    Serial.print(quantities[cell][0]);
    Serial.print(" B:");
    Serial.print(quantities[cell][1]);
    Serial.print(" C:");
    Serial.println(quantities[cell][2]);
  }

  Serial.println("A/B/C = current recorded stock quantities");
  Serial.println("======================================================");
}

// ------------------------------------------------------------
// Setup and individual calibration
// ------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(1500);  // Give the USB serial connection time to open.

  Serial.println();
  Serial.println("================================================");
  Serial.println("       FOUR-CELL SMART MINI MART");
  Serial.println("================================================");
  Serial.println("Cell 1 and Cell 2: 1 kg load cells");
  Serial.println("Cell 3 and Cell 4: 5 kg load cells");
  Serial.println("Each cell has a permanently attached 70 g plate.");
  Serial.println("Each responding cell will be calibrated separately with a 100 g weight.");
  Serial.flush();

  // Check each DOUT pin BEFORE calling HX711::begin(). Some HX711 library versions
  // can block inside begin() if an absent module is initialized. Missing cells are
  // skipped and the next cell is checked.
  for (int i = 0; i < NUM_CELLS; i++) {
    Serial.println();
    Serial.print("[INIT] Cell ");
    Serial.print(i + 1);
    Serial.print(" | DT GPIO ");
    Serial.print(DOUT_PINS[i]);
    Serial.print(" | SCK GPIO ");
    Serial.println(CLK_PINS[i]);
    Serial.flush();

    Serial.print("[CHECK] Checking Cell ");
    Serial.print(i + 1);
    Serial.println(" DOUT signal for up to 3 seconds...");
    Serial.flush();

    // IMPORTANT: Do NOT call scales[i].begin() until DOUT has been seen LOW.
    // Some HX711 library versions call a blocking read() from begin()/set_gain().
    // If the module is missing, that can freeze setup before the timeout check.
    // INPUT_PULLUP keeps an unconnected DOUT pin HIGH, allowing us to time out.
    pinMode(DOUT_PINS[i], INPUT_PULLUP);
    pinMode(CLK_PINS[i], OUTPUT);
    digitalWrite(CLK_PINS[i], LOW);

    bool doutReady = false;
    unsigned long checkStart = millis();
    while (millis() - checkStart < 3000UL) {
      if (digitalRead(DOUT_PINS[i]) == LOW) {
        doutReady = true;
        break;
      }
      delay(10);
    }

    if (!doutReady) {
      cellAvailable[i] = false;
      Serial.print("[SKIP] Cell ");
      Serial.print(i + 1);
      Serial.println(" DOUT stayed HIGH; no responding HX711 found. Moving to next cell.");
      Serial.println("       Check VCC, GND, DT/DOUT and SCK/CLK wiring.");
      Serial.println("       After fixing wiring, reset the ESP32 to detect and calibrate it.");
      Serial.flush();
      continue;
    }

    // DOUT is LOW, so the module is ready. It is now safe to initialize HX711.
    scales[i].begin(DOUT_PINS[i], CLK_PINS[i]);
    scales[i].set_scale(1.0);

    if (scales[i].wait_ready_timeout(1000)) {
      cellAvailable[i] = true;
      Serial.print("[OK] Cell ");
      Serial.print(i + 1);
      Serial.println(" HX711 is responding.");
    } else {
      cellAvailable[i] = false;
      Serial.print("[SKIP] Cell ");
      Serial.print(i + 1);
      Serial.println(" did not become ready after initialization. Moving to next cell.");
    }
    Serial.flush();
  }

  int respondingCount = 0;
  for (int i = 0; i < NUM_CELLS; i++) {
    if (cellAvailable[i]) respondingCount++;
  }

  Serial.println();
  Serial.print("HX711 modules responding: ");
  Serial.print(respondingCount);
  Serial.print(" of ");
  Serial.println(NUM_CELLS);
  Serial.println();
  Serial.println("CALIBRATION INSTRUCTIONS:");
  Serial.println("- Keep each cell's fixed 70 g plate attached at all times.");
  Serial.println("- Remove products from the cell currently being calibrated.");
  Serial.println("- Use the same known 100 g reference weight for each cell.");
  Serial.println("- Put the 100 g weight on ONLY the cell named in the prompt.");
  Serial.println("- Set Serial Monitor line ending to Newline or Both NL & CR.");
  Serial.println("- During calibration, the live weight is shown in grams; raw counts are hidden.");
  Serial.flush();

  // Calibrate responding sensors one by one. For each cell, first tare the
  // empty permanent plate, then measure the known 100 g reference weight.
  for (int i = 0; i < NUM_CELLS; i++) {
    if (!cellAvailable[i]) {
      continue;  // Keep going if this module did not respond at startup.
    }

    Serial.println();
    Serial.println("------------------------------------------------");
    Serial.print("CELL ");
    Serial.print(i + 1);
    Serial.println(" - STEP 1: TARE EMPTY PLATE");
    Serial.print("Keep the fixed 70 g plate attached to Cell ");
    Serial.print(i + 1);
    Serial.println(".");
    Serial.println("Remove all products from this cell.");
    Serial.println("This step sets the empty plate to 0.0 g; the fixed 70 g plate stays attached.");
    Serial.println("Press ENTER to tare this cell (zero the empty plate).");
    Serial.flush();
    waitForEnter();

    if (!scales[i].wait_ready_timeout(2000)) {
      cellAvailable[i] = false;
      Serial.print("[SKIP] Cell ");
      Serial.print(i + 1);
      Serial.println(" stopped responding before tare. Moving to next cell.");
      Serial.flush();
      continue;
    }

    scales[i].set_scale(1.0);
    scales[i].tare(15);  // Tare with the 70 g plate attached and no products.
    Serial.print("[OK] Cell ");
    Serial.print(i + 1);
    Serial.println(" tare complete.");
    Serial.println("[ZERO] Empty plate is now the zero reference (target: 0.0 g).");

    Serial.println();
    Serial.print("CELL ");
    Serial.print(i + 1);
    Serial.println(" - STEP 2: 100 g CALIBRATION");
    Serial.print("Place the known ");
    Serial.print(CALIBRATION_WEIGHT_G[i], 1);
    Serial.print(" g reference weight on Cell ");
    Serial.print(i + 1);
    Serial.println(" ONLY.");
    Serial.println("Place the weight and press Enter when it is settled.");
    Serial.println("The ESP32 then calculates calibration and displays the measured grams.");
    Serial.flush();

    if (!waitForReferenceWeightAndCalibrate(i)) {
      cellAvailable[i] = false;
      Serial.print("[CALIBRATION FAILED] Cell ");
      Serial.print(i + 1);
      Serial.println(" - no stable 100 g reference weight was detected. Moving to the next cell.");
      Serial.flush();
      continue;
    }

    float measuredReference = scales[i].get_units(8);
    Serial.print("[VERIFY] Cell ");
    Serial.print(i + 1);
    Serial.print(" calibrated reading with the reference weight: ");
    Serial.print(measuredReference, 1);
    Serial.println(" g (target: about 100.0 g).");
    Serial.println("Next, remove the 100 g reference weight; do NOT press TARE again.");

    // Verify that the reference weight was removed. Do NOT tare again here:
    // re-taring while the 100 g weight is still on the platform makes later
    // readings appear close to -100 g when that weight is removed.
    bool emptyConfirmed = false;
    while (!emptyConfirmed) {
      Serial.println();
      Serial.print("Remove the 100 g reference weight from Cell ");
      Serial.print(i + 1);
      Serial.println(". Keep the fixed 70 g plate attached.");
      Serial.println("Watch the live calibrated reading below; with the 100 g removed it should return near 0.0 g.");
      Serial.println("Press ENTER after you have removed the weight and the reading has settled.");
      Serial.flush();
      waitForEnterWithLiveReading(i, true);

      if (!scales[i].wait_ready_timeout(2000)) {
        cellAvailable[i] = false;
        Serial.print("[SKIP] Cell ");
        Serial.print(i + 1);
        Serial.println(" stopped responding after calibration. Moving to next cell.");
        Serial.flush();
        break;
      }

      float emptyCheck = scales[i].get_units(8);
      Serial.print("[CHECK] Empty-cell reading: ");
      Serial.print(emptyCheck, 1);
      Serial.println(" g");

      if (fabs(emptyCheck) <= 15.0) {
        emptyConfirmed = true;
      } else {
        Serial.println("[WARNING] The cell is not reading near zero.");
        Serial.println("Make sure the 100 g weight is removed and nothing else is on the plate.");
        Serial.println("Wait for the reading to settle, then press ENTER to check again.");
      }
    }

    if (!cellAvailable[i]) {
      continue;
    }

    // Keep the original tare offset established before calibration.
    previousWeight[i] = 0.0;
    filteredWeight[i] = 0.0;
    filterInitialized[i] = false;
    changePending[i] = false;
    pendingAnchorWeight[i] = 0.0;
    pendingSince[i] = 0;
    pendingStableSamples[i] = 0;
    lastMeasuredWeight[i] = 0.0;
    hasLiveReading[i] = false;

    Serial.print("[DONE] Cell ");
    Serial.print(i + 1);
    Serial.println(" calibrated and empty. Moving to the next cell.");
    Serial.flush();
  }

  Serial.println();
  Serial.println("================================================");
  Serial.println("CALIBRATION STAGE FINISHED");
  Serial.println("================================================");

  int calibratedCount = 0;
  for (int cell = 0; cell < NUM_CELLS; cell++) {
    Serial.print("Cell ");
    Serial.print(cell + 1);
    Serial.print(": ");
    if (cellAvailable[cell]) {
      calibratedCount++;
      Serial.println("ACTIVE / CALIBRATED");
      Serial.print("  Product weights A/B/C: ");
      Serial.print(PRODUCT_WEIGHT_G[cell][0], 1);
      Serial.print(" g / ");
      Serial.print(PRODUCT_WEIGHT_G[cell][1], 1);
      Serial.print(" g / ");
      Serial.print(PRODUCT_WEIGHT_G[cell][2], 1);
      Serial.println(" g");
      printCurrentStock(cell, 0.0);
      for (int p = 0; p < NUM_PRODUCTS; p++) {
        sendState(cell, p, quantities[cell][p], 0.0);
      }
    } else {
      Serial.println("SKIPPED - not responding or calibration failed");
    }
  }

  Serial.println();
  Serial.print("SYSTEM READY - monitoring ");
  Serial.print(calibratedCount);
  Serial.println(" calibrated cell(s).");
  Serial.println("If one cell is skipped, it will not block the other cells.");
  Serial.println("Fix its wiring and reset the ESP32 to detect and calibrate it.");
  Serial.println("Live weight dashboard refreshes every second for all four cells.");
  Serial.println("Watch for ITEMS ADDED / ITEMS REMOVED messages.");
  Serial.flush();

  buzzerInit();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  startNetwork();

  Serial.println();
  Serial.println("MODE: STOCKING. Put the products on the shelf now (they are recorded, not reported).");
  Serial.println("When the shelf is stocked, type  l  (the letter L) or 1  and press ENTER to go LIVE.");
  Serial.println("Commands (press ENTER after each): l = live, s = stocking, p = send inventory, status, add N / take N (test).");
}

// ------------------------------------------------------------
// Main loop
// ------------------------------------------------------------

void loop() {
  // Read each calibrated cell independently on every pass.
  for (int cell = 0; cell < NUM_CELLS; cell++) {
    processCell(cell);
  }

  watchLoop();
  handleSerialCommands();
  handleBuzzer();

  if (fbQueue && millis() - lastBeatAt > 15000UL) {
    lastBeatAt = millis();
    queueJob(JOB_BEAT, "", 0, "");
  }

  // Show all four cells even when no product is added or removed.
  unsigned long now = millis();
  if (now - lastDashboardTime >= DASHBOARD_INTERVAL_MS) {
    lastDashboardTime = now;
    printLiveDashboard();
  }

  delay(10);
}
