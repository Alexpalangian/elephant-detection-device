// ============================================================
// E32 LoRa messaging. Put THE SAME sketch on both boards, then change ONE number
// before uploading to each:
//
//     #define TRANSMITTER 1     -> this board is the TRANSMITTER (sends)
//     #define TRANSMITTER 0     -> this board is the RECEIVER (listens)
//
// The two boards must have DIFFERENT values. Check the "RUNNING:" line in the Serial Monitor.
// Serial Monitor: 115200 baud, line ending "Newline".
//
// TRANSMITTER: type any line and press Enter. It goes out as-is, as plain text or as a
//   data line such as   DET,1,5,850.0,120.00,0.14
//   It ignores anything it hears.
//
// TEST the link and count lost messages (transmitter only):
//   test              sends 100 numbered messages, one per second
//   test 500          sends 500
//   test 500 400      sends 500, one every 400 ms
//   test stop         ends a running test early
//   The receiver compares the message numbers and prints a RESULT line when the test ends.
//
// RECEIVER: prints what it hears. It refuses to send.
//
// OTHER (either board):  stats = receiver prints its current counts,  help = list of commands
// ============================================================
#define NODE_ID              1      // each sensor node gets its own number (0-7)

#define TRANSMITTER        0      // 1 = transmitter (sends), 0 = receiver (listens)

#define TEST_DEFAULT         100    // messages sent when you just type "test"
#define TEST_MAX             100000 // only a guard against typing a huge number by mistake
#define TEST_INTERVAL_MS     1000   // default time between test messages
#define TEST_MIN_INTERVAL_MS 250    // faster than this and the radio can't keep up

#define SWAP_UART 0         // 1 = swap the two serial wires in software instead of rewiring
#if SWAP_UART
  #define E32_RX 17
  #define E32_TX 16
#else
  #define E32_RX 16
  #define E32_TX 17
#endif
#define E32_M0 4
#define E32_M1 5

// ---- sender side ----
unsigned long seq = 0;            // number of the next test message to send
long testLeft = 0;                // test messages still to send (0 = no test running)
int endCopies = 0;                // END messages still to send after the test
long testInterval = TEST_INTERVAL_MS;
unsigned long lastSendTime = 0;   // when we last sent a test or END message

// ---- receiver side: one slot per node ID (0-7). Arrays start at zero. ----
unsigned long expectedSeq[8];     // the sequence number we expect to hear next
unsigned long gotCount[8];        // how many test messages actually arrived
unsigned long lostCount[8];       // how many were skipped (gaps in the numbers)

const char* roleName() {
  return TRANSMITTER ? "TRANSMITTER" : "RECEIVER";
}

void printHelp() {
  Serial.printf("This board is the %s.\n", roleName());
  Serial.println("Transmitter commands: test | test <count> | test <count> <interval_ms> | test stop");
  Serial.println("Anything else you type is sent as a message.");
  Serial.println("Either board: stats | help");
}

// Builds "DET,node,seq,ratio,mag25,mag50" and sends it.
// Keep it short: LoRa at 2.4 kbps takes about 0.3 s per 100 bytes.
void sendDetection(float ratio, float mag25, float mag50) {
  char msg[80];
  snprintf(msg, sizeof(msg), "DET,%d,%lu,%.1f,%.2f,%.2f", NODE_ID, seq++, ratio, mag25, mag50);
  Serial2.println(msg);
  Serial.print("sent: ");
  Serial.println(msg);
}

// Tells the receiver how many test messages were sent in total.
void sendEnd() {
  char msg[40];
  snprintf(msg, sizeof(msg), "END,%d,%lu", NODE_ID, seq);
  Serial2.println(msg);
  Serial.print("sent: ");
  Serial.println(msg);
}

// Called every loop. Sends the next test message when it is time, then the END messages.
void runTest() {
  if (testLeft > 0) {
    if (millis() - lastSendTime >= (unsigned long)testInterval) {
      lastSendTime = millis();
      sendDetection(850.0 + (seq % 5) * 20, 120.0, 0.14);   // fake numbers for the test
      testLeft--;
    }
  } else if (endCopies > 0) {
    // Send END three times, half a second apart, in case one copy is lost.
    if (millis() - lastSendTime >= 500) {
      lastSendTime = millis();
      sendEnd();
      endCopies--;
      if (endCopies == 0) Serial.println("Test finished.");
    }
  }
}

void printStats() {
  bool any = false;
  for (int i = 0; i < 8; i++) {
    if (gotCount[i] > 0 || lostCount[i] > 0) {
      Serial.printf("node %d: received %lu, lost so far %lu\n", i, gotCount[i], lostCount[i]);
      any = true;
    }
  }
  if (!any) Serial.println("No test messages received yet.");
}

void startTest(const char* args) {
  long n = 0, interval = TEST_INTERVAL_MS;
  sscanf(args, "%ld %ld", &n, &interval);          // reads up to two numbers after the word "test"
  if (n <= 0) n = TEST_DEFAULT;
  if (n > TEST_MAX) n = TEST_MAX;
  if (interval < TEST_MIN_INTERVAL_MS) interval = TEST_MIN_INTERVAL_MS;
  seq = 0;                                         // numbering starts at 0 so the receiver starts a fresh count
  testLeft = n;
  testInterval = interval;
  endCopies = 3;
  lastSendTime = millis() - interval;              // makes the first message go out right away
  Serial.printf("TEST started: sending %ld messages, one every %ld ms\n", n, interval);
}

// A line you typed: a command, or a message to send.
void handleTypedLine(const char* line) {
  // ---- commands that work on either board ----
  if (strcmp(line, "stats") == 0)      { printStats(); return; }
  if (strcmp(line, "help") == 0)       { printHelp(); return; }

  // ---- everything below transmits, so a receiver refuses ----
  if (!TRANSMITTER) {
    Serial.println("This board is a RECEIVER, so it does not send. Set TRANSMITTER to 1 and re-upload to change.");
    return;
  }

  if (strcmp(line, "test stop") == 0) {
    if (testLeft > 0) {
      testLeft = 0;                                // stop sending; END tells the receiver the total so far
      endCopies = 3;
      lastSendTime = millis();
      Serial.printf("Test stopped early after %lu messages.\n", seq);
    } else {
      Serial.println("No test is running.");
    }
  } else if (strcmp(line, "test") == 0 || strncmp(line, "test ", 5) == 0) {
    startTest(line + 4);
  } else {
    Serial2.println(line);                         // plain text or a data line, sent as-is
    Serial.print("you: ");
    Serial.println(line);
  }
}

// A line that arrived over the radio.
void handleRadioLine(const char* line) {
  if (strncmp(line, "DET,", 4) == 0) {
    int node = 0; unsigned long s = 0; float ratio = 0, m25 = 0, m50 = 0;
    if (sscanf(line, "DET,%d,%lu,%f,%f,%f", &node, &s, &ratio, &m25, &m50) == 5) {
      if (node < 0 || node > 7) {
        Serial.println("bad node number, ignored");
        return;
      }
      if (s == 0) {
        // Sequence 0 means a new test just started: begin a fresh count for this node.
        expectedSeq[node] = 0;
        gotCount[node] = 0;
        lostCount[node] = 0;
      } else if (s < expectedSeq[node]) {
        Serial.println("old or repeated message, ignored");
        return;
      }
      if (s > expectedSeq[node]) {
        lostCount[node] += s - expectedSeq[node];   // numbers we skipped = lost messages
      }
      gotCount[node]++;
      expectedSeq[node] = s + 1;
      Serial.printf("DETECTION from node %d  #%lu  ratio=%.1f  mag25=%.2f  mag50=%.2f  | got %lu, lost %lu\n",
                    node, s, ratio, m25, m50, gotCount[node], lostCount[node]);
      // This is where the gateway decides what to do next, e.g. if ratio is in range -> alert.
    } else {
      Serial.print("garbled DET message (counts as lost): ");
      Serial.println(line);
    }
  } else if (strncmp(line, "END,", 4) == 0) {
    int node = 0; unsigned long total = 0;
    if (sscanf(line, "END,%d,%lu", &node, &total) == 2 && node >= 0 && node <= 7) {
      // Compare what the sender says it sent with what we actually received.
      long lost = (long)total - (long)gotCount[node];
      float pct = (total > 0) ? (100.0 * lost / total) : 0.0;
      Serial.printf("RESULT node %d: sender sent %lu, received %lu, lost %ld (%.1f%%)\n",
                    node, total, gotCount[node], lost, pct);
    } else {
      Serial.print("garbled END message: ");
      Serial.println(line);
    }
  } else {
    Serial.print("CHAT: ");
    Serial.println(line);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // Shows which sketch and settings are on this board, so you can tell the boards apart.
  Serial.println();
  Serial.printf("RUNNING: e32_msg | NODE_ID=%d | SWAP_UART=%d | ROLE=%s\n", NODE_ID, SWAP_UART, roleName());

  // M0 = LOW and M1 = LOW is normal mode: whatever you write is transmitted.
  pinMode(E32_M0, OUTPUT);
  pinMode(E32_M1, OUTPUT);
  digitalWrite(E32_M0, LOW);
  digitalWrite(E32_M1, LOW);
  Serial2.begin(9600, SERIAL_8N1, E32_RX, E32_TX);   // 9600 = E32 factory default
  delay(1000);                                       // let the radio finish starting up
  Serial.println("Ready. Type 'help' for commands.");
}

void loop() {
  // ---- typed text: collect one full line, then handle it ----
  static char typed[100];
  static int tn = 0;
  static bool tooLong = false;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (tn > 0) {
        typed[tn] = 0;
        if (tooLong) Serial.println("(line was longer than 99 characters, the extra was cut off)");
        handleTypedLine(typed);
        tn = 0;
        tooLong = false;
      }
    } else if (tn < (int)sizeof(typed) - 1) {
      typed[tn++] = c;
    } else {
      tooLong = true;
    }
  }

  // ---- radio -> one full line at a time ----
  static char rx[100];
  static int rn = 0;
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (TRANSMITTER) continue;                     // a transmitter ignores whatever it hears
    if (c == '\n') {
      rx[rn] = 0;
      if (rn > 0) handleRadioLine(rx);
      rn = 0;
    } else if (c != '\r' && rn < (int)sizeof(rx) - 1) {
      rx[rn++] = c;
    }
  }

  // ---- keep a running test going (does nothing unless you typed "test") ----
  runTest();
}
