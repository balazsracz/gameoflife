
static constexpr uint32_t kOBHiAddress = 0x1FFFF806;
static constexpr uint32_t kOBLoAddress = 0x1FFFF804;


void print_ob() {
  uint32_t *p = (uint32_t *)0x1FFFF800;
  SerialUSB.printf("OB: %08x %08x %08x %08x\n", p[0], p[1], p[2], p[3]);
}

bool ProgramId(uint8_t hi, uint8_t lo) {
  uint8_t *plo = (uint8_t *)kOBLoAddress;
  uint8_t chi = plo[2];
  uint8_t clo = plo[0];
  print_ob();
  if (clo != lo || chi != hi) {
    SerialUSB.printf("Setting userdata\n");
    for (unsigned i = 0; i < 10; i++) {
      delay(1000);
      SerialUSB.printf("Setting userdata %d\n", i);
      print_ob();
    }
    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();
    FLASH_OBProgramInitTypeDef d;
    memset(&d, 0, sizeof(d));
    if ((plo[1] & lo) != lo || (plo[3] & hi) != hi) {
      // need erase
      SerialUSB.printf("need OB erase\n");
      HAL_FLASHEx_OBGetConfig(&d);
      auto st = HAL_FLASHEx_OBErase();
      if (st != HAL_OK) {
        SerialUSB.printf("Failed OB erase: %d %x\n", st, HAL_FLASH_GetError());
        print_ob();
        return false;
      }
    }
    d.OptionType |= OPTIONBYTE_DATA;
    d.DATAAddress = kOBHiAddress;
    d.DATAData = hi;
    auto st = HAL_FLASHEx_OBProgram(&d);
    if (st != HAL_OK) {
      SerialUSB.printf("Failed OB program: %d %x\n", st, HAL_FLASH_GetError());
    }
    d.OptionType = OPTIONBYTE_DATA;
    d.DATAAddress = kOBLoAddress;
    d.DATAData = lo;
    st = HAL_FLASHEx_OBProgram(&d);
    if (st != HAL_OK) {
      SerialUSB.printf("Failed OB program2: %d %x\n", st, HAL_FLASH_GetError());
    }
    SerialUSB.printf("Setting userdata DONE\n");
    print_ob();
    delay(10);
    HAL_FLASH_OB_Launch();
    return true;
  } else {
    SerialUSB.printf("already good\n");
    return false;
  }
}


void setup() {
  SerialUSB.begin(9600);
  delay(25);
  // put your setup code here, to run once:
  ProgramId(0x33, 0x02);
}

uint64_t nmranet_nodeid() {
  return 0x050101010000 | ((*(uint8_t *)kOBHiAddress) << 8) | (*(uint8_t *)kOBLoAddress);
}

void loop() {
  SerialUSB.printf("node id: 0x0501%08x\n", uint32_t(nmranet_nodeid() & 0xfffffffful));
  print_ob();
  delay(3000);
}
