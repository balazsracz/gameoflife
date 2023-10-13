// This is the address to program.
static constexpr uint8_t kLo = 4;
static constexpr uint8_t kHi = 0x85;

// These two pins have an LED on them. The LED will light when you write LOW to the respective pin.
static constexpr int kLed13Pin = PF0;
static constexpr int kLed14Pin = PF1;


static constexpr uint32_t kOBHiAddress = 0x1FFFF806;
static constexpr uint32_t kOBLoAddress = 0x1FFFF804;


void print_ob() {
  uint32_t *p = (uint32_t *)0x1FFFF800;
  Serial.printf("OB: %08x %08x %08x %08x\n", p[0], p[1], p[2], p[3]);
}

bool ProgramId(uint8_t hi, uint8_t lo) {
  uint8_t *plo = (uint8_t *)kOBLoAddress;
  uint8_t chi = plo[2];
  uint8_t clo = plo[0];
  print_ob();
  if (clo != lo || chi != hi) {
    Serial.printf("Setting userdata\n");
    digitalWrite(kLed13Pin, LOW);

#if 0
    for (unsigned i = 0; i < 10; i++) {
      delay(1000);
      Serial.printf("Setting userdata %d\n", i);
      print_ob();
    }
#endif
    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();
    FLASH_OBProgramInitTypeDef d;
    memset(&d, 0, sizeof(d));
    if ((plo[1] & lo) != lo || (plo[3] & hi) != hi) {
      // need erase
      Serial.printf("need OB erase\n");
      HAL_FLASHEx_OBGetConfig(&d);
      auto st = HAL_FLASHEx_OBErase();
      if (st != HAL_OK) {
        Serial.printf("Failed OB erase: %d %x\n", st, HAL_FLASH_GetError());
        print_ob();
        return false;
      }
    }
    d.OptionType |= OPTIONBYTE_DATA;
    d.DATAAddress = kOBHiAddress;
    d.DATAData = hi;
    auto st = HAL_FLASHEx_OBProgram(&d);
    if (st != HAL_OK) {
      Serial.printf("Failed OB program: %d %x\n", st, HAL_FLASH_GetError());
    }
    d.OptionType = OPTIONBYTE_DATA;
    d.DATAAddress = kOBLoAddress;
    d.DATAData = lo;
    st = HAL_FLASHEx_OBProgram(&d);
    if (st != HAL_OK) {
      Serial.printf("Failed OB program2: %d %x\n", st, HAL_FLASH_GetError());
    }
    Serial.printf("Setting userdata DONE\n");
    print_ob();
    delay(10);
    digitalWrite(kLed14Pin, LOW);
    delay(1000);

    HAL_FLASH_OB_Launch();
    return true;
  } else {
    Serial.printf("already good\n");
    return false;
  }
}


void setup() {
  Serial.begin(115200);
  pinMode(kLed13Pin, OUTPUT);
  pinMode(kLed14Pin, OUTPUT);
  digitalWrite(kLed13Pin, HIGH);
  digitalWrite(kLed14Pin, HIGH);
  // put your setup code here, to run once:
  ProgramId(kHi, kLo);
}

uint64_t nmranet_nodeid() {
  return 0x050101010000 | ((*(uint8_t *)kOBHiAddress) << 8) | (*(uint8_t *)kOBLoAddress);
}

void loop() {
  //Serial.printf("node id: 0x0501%08x\n", uint32_t(nmranet_nodeid() & 0xfffffffful));
  print_ob();
  auto id = nmranet_nodeid();
  if ((id & 0xff) == kLo && ((id >> 8) & 0xff) == kHi) {
    digitalWrite(kLed14Pin, LOW);
    delay(250);
    digitalWrite(kLed14Pin, HIGH);
    digitalWrite(kLed13Pin, LOW);
    delay(250);
    digitalWrite(kLed13Pin, HIGH);
  }
}
