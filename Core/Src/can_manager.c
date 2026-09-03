#include "can_manager.h" /* CAN frame tipleri, durum kodlari, RTOS ve STM32 tanimlari burada gelir. */

#define CAN_MANAGER_RX_QUEUE_SIZE 16U /* ISR'nin aldigi fakat task'in henuz islemedigi en fazla frame sayisi. */
#define CAN2_FILTER_BANK          14U /* F407'de CAN1 ve CAN2 ortak filtre RAM'i kullanir; CAN2 bolgesi bank 14'te baslatiliyor. */
#define CAN_MANAGER_START_TIMEOUT_MS 100U /* CAN normal moda gecemezse task'i sonsuza kadar kilitlememek icin ust sinir. */

typedef struct{
  CAN_TypeDef* can; /* Bu context'in yonettigi fiziksel bxCAN peripheral'i. */
  osThreadId_t rx_notification_task; /* RX ISR olaylarini bekleyen CAN okuma task'inin RTOS kimligi. */
  osThreadId_t tx_notification_task; /* TX ISR olaylarini bekleyen CAN yazma task'inin RTOS kimligi. */
  volatile uint32_t rx_write_sequence; /* Yalniz ISR arttirir; tamamlanmis son yazma konumunu task'a yayinlar. */
  volatile uint32_t rx_read_sequence; /* Yalniz RX task arttirir; serbest birakilan son konumu ISR'ye yayinlar. */
  volatile bool initialized; /* Manager hazir olmadan send/receive yapilmasini engelleyen yazilim durumu. */
  can_frame_t rx_queue[CAN_MANAGER_RX_QUEUE_SIZE]; /* Donanim FIFO'su ile task arasindaki yazilim tamponu. */
}can_manager_context_t;

static can_manager_context_t can2_context = {0}; /* Sadece bu dosyanin gorebildigi CAN2 calisma durumu. */

volatile can_manager_debug_t g_can2_debug = {0}; /* Live Expressions icin sayac ve ham register goruntuleri. */
volatile can_frame_t g_can2_last_tx_frame = {0}; /* En son gonderilmesi istenen frame'i debugger'da gosterir. */
const can_frame_t* volatile g_can2_last_rx_frame_ptr = NULL; /* Son peek edilen kuyruk hucresini kopyalamadan debugger'a gosterir. */

void can_manager_refresh_debug(CAN_TypeDef* can){
  if(can != CAN2) return; /* Bu surumde yalniz CAN2 context'i var; baska peripheral okunmaz. */

  g_can2_debug.last_mcr = can->MCR; /* MCR = Master Control Register: bxCAN'in init, sleep ve otomatik davranis ayarlarini tutan kontrol register'i. */
  g_can2_debug.last_msr_snapshot = can->MSR; /* MSR = Master Status Register: bxCAN'in init/sleep durumunu ve status interrupt nedenlerini gosteren register. */
  g_can2_debug.last_tsr = can->TSR; /* TSR = Transmit Status Register: uc TX mailbox'in bos, tamamlandi, basarili veya hatali olma durumlarini gosteren register. */
  g_can2_debug.last_rf0r = can->RF0R; /* RF0R = Receive FIFO 0 Register: FIFO0'da bekleyen frame sayisini, full ve overflow durumlarini gosteren register. */
  g_can2_debug.last_ier = can->IER; /* IER = Interrupt Enable Register: hangi bxCAN olaylarinin CPU'ya interrupt gonderebilecegini belirleyen register. */
  g_can2_debug.last_btr = can->BTR; /* BTR = Bit Timing Register: CAN baud rate, time segment ve synchronization jump width ayarlarini tutan register. */
  g_can2_debug.last_esr = can->ESR; /* ESR = Error Status Register: TX/RX error counter'larini, bus-off durumunu ve Last Error Code'u gosteren register. */
}

static void copy_frame_to_volatile(volatile can_frame_t* destination, const can_frame_t* source){
  destination->id = source->id; /* Debug kopyasinin ID alanini gunceller. */
  destination->dlc = source->dlc; /* Debug kopyasinin uzunlugunu gunceller. */
  destination->extended_id = source->extended_id; /* Debug kopyasinin ID tipini gunceller. */
  destination->remote_frame = source->remote_frame; /* Debug kopyasinin frame tipini gunceller. */
  destination->timestamp = source->timestamp; /* Debug kopyasinin zaman damgasini gunceller. */

  for(uint8_t index = 0U; index < 8U; index++){
    destination->data[index] = source->data[index]; /* volatile hedefe yazildigi icin derleyici bu yazmalari atamaz. */
  }
}

static void notify_rx_task(uint32_t flag){
  if(can2_context.rx_notification_task != NULL){
    (void)osThreadFlagsSet(can2_context.rx_notification_task, flag);
  }
}

static void notify_tx_task(uint32_t flag){
  if(can2_context.tx_notification_task != NULL){
    (void)osThreadFlagsSet(can2_context.tx_notification_task, flag);
  }
}

static void configure_can2_accept_all_filter(void){
  uint32_t filter_bit = 1UL << CAN2_FILTER_BANK; /* Bank 14'u filtre kontrol register'larinda secen tek bitlik maske. */

  CAN1->FMR |= CAN_FMR_FINIT; /* FMR = Filter Master Register, FINIT = Filter Initialization Mode: filtre banklarini degistirebilmek icin filtre donanimini init moduna alir. */
  CAN1->FMR = (CAN1->FMR & ~CAN_FMR_CAN2SB) |
          (CAN2_FILTER_BANK << CAN_FMR_CAN2SB_Pos) | CAN_FMR_FINIT; /* CAN2SB = CAN2 Start Bank: bank 14 ve sonrasini CAN2'ye ayirir; CAN1 ve CAN2 ortak filtre RAM'i kullanir. */

  CAN1->FA1R &= ~filter_bit; /* FA1R = Filter Activation Register: bank 14'u ayarlarken pasif yapar; yari yazilmis filtreyle frame kabul edilmesini onler. */
  CAN1->FM1R &= ~filter_bit; /* FM1R = Filter Mode Register: biti 0 yaparak Identifier Mask Mode secer; FR1 deger, FR2 hangi ID bitlerinin onemli oldugunu belirleyen maskedir. */
  CAN1->FS1R |= filter_bit; /* FS1R = Filter Scale Register: bank 14'u 32-bit olcege alir; standart/extended ID ve frame turu tek filtrede incelenebilir. */
  CAN1->FFA1R &= ~filter_bit; /* FFA1R = Filter FIFO Assignment Register: biti 0 yaparak eslesen frameleri donanim RX FIFO0'a yonlendirir. */
  CAN1->sFilterRegister[CAN2_FILTER_BANK].FR1 = 0U; /* FR1 = Filter Register 1: karsilastirilacak ID/frame desenini 0 yapar. */
  CAN1->sFilterRegister[CAN2_FILTER_BANK].FR2 = 0U; /* FR2 = Filter Register 2: maskeyi 0 yapar; hicbir bit zorunlu olmadigi icin her ID eslesir. */
  CAN1->FA1R |= filter_bit; /* FA1R = Filter Activation Register: hazirlanan bank 14'u etkinlestirir; aktif filtre olmadan bxCAN gelen frame'i FIFO'ya teslim etmez. */
  CAN1->FMR &= ~CAN_FMR_FINIT; /* FMR.FINIT temizlenir: filtre init modu biter ve filtreleme donanimi yeni ayarlarla tekrar calisir. */
}

static void decode_fifo_frame(CAN_TypeDef* can, uint8_t fifo, can_frame_t* frame){
  CAN_FIFOMailBox_TypeDef* mailbox = &can->sFIFOMailBox[fifo]; /* FIFO0 veya FIFO1'in cikis mailbox register grubunu secer. */
  uint32_t rir = mailbox->RIR; /* RIR = Receive Identifier Register: alinan frame'in ID, IDE (ID tipi) ve RTR (remote/data tipi) bitlerini tek snapshot olarak alir. */
  uint32_t rdtr = mailbox->RDTR; /* RDTR = Receive Data Length/Time Register: DLC ile payload uzunlugunu ve TIME alaninda timestamp'i tutar. */
  uint32_t rdlr = mailbox->RDLR; /* RDLR = Receive Data Low Register: alinan payload'in byte 0..3 bolumunu tutar. */
  uint32_t rdhr = mailbox->RDHR; /* RDHR = Receive Data High Register: alinan payload'in byte 4..7 bolumunu tutar. */

  frame->extended_id = (rir & CAN_RI0R_IDE) != 0U; /* IDE = Identifier Extension: 0 ise 11-bit Standard ID, 1 ise 29-bit Extended ID kullanilmistir. */
  frame->remote_frame = (rir & CAN_RI0R_RTR) != 0U; /* RTR = Remote Transmission Request: 0 data frame, 1 veri istemek icin gonderilen remote frame demektir. */
  if(frame->extended_id){
    frame->id = (rir >> 3) & 0x1FFFFFFF; /* Extended ID kullaniliyorsa RIR icindeki 29-bit ID'yi normal sayiya cevirir. */
  }
  else{
    frame->id = (rir >> 21) & 0x7FF; /* Standard ID kullaniliyorsa RIR icindeki 11-bit ID'yi normal sayiya cevirir. */
  }
  frame->dlc = (uint8_t)(rdtr & CAN_RDT0R_DLC); /* DLC = Data Length Code: RDTR'nin alt dort bitinden frame'deki gecerli payload byte sayisini alir. */
  if(frame->dlc > 8U) frame->dlc = 8U; /* Klasik CAN veri alanini asan bozuk/rezerv degeri yazilim sinirinda tutar. */
  frame->timestamp = (uint16_t)(rdtr >> 16U); /* Donanim timestamp modu kullanilirsa ust 16 biti saklar. */
  frame->data[0] = (uint8_t)(rdlr >> 0U); /* RDLR bit 7:0 -> payload byte 0. */
  frame->data[1] = (uint8_t)(rdlr >> 8U); /* RDLR bit 15:8 -> payload byte 1. */
  frame->data[2] = (uint8_t)(rdlr >> 16U); /* RDLR bit 23:16 -> payload byte 2. */
  frame->data[3] = (uint8_t)(rdlr >> 24U); /* RDLR bit 31:24 -> payload byte 3. */
  frame->data[4] = (uint8_t)(rdhr >> 0U); /* RDHR bit 7:0 -> payload byte 4. */
  frame->data[5] = (uint8_t)(rdhr >> 8U); /* RDHR bit 15:8 -> payload byte 5. */
  frame->data[6] = (uint8_t)(rdhr >> 16U); /* RDHR bit 23:16 -> payload byte 6. */
  frame->data[7] = (uint8_t)(rdhr >> 24U); /* RDHR bit 31:24 -> payload byte 7. */
}

static void drain_fifo(CAN_TypeDef* can, uint8_t fifo){
  volatile uint32_t* fifo_register;
  uint32_t pending_mask;
  uint32_t release_mask;

  if(fifo == 0){
    fifo_register = &can->RF0R; /* RF0R = Receive FIFO 0 Register: FIFO0'in bekleyen frame sayisini ve kontrol bitlerini tutar. */
    pending_mask = CAN_RF0R_FMP0; /* FMP0 = FIFO 0 Message Pending: FIFO0'da kac frame bekledigini gosteren alan. */
    release_mask = CAN_RF0R_RFOM0; /* RFOM0 = Release FIFO 0 Output Mailbox: en eski FIFO0 frame'ini serbest birakir. */
  }
  else{
    fifo_register = &can->RF1R; /* RF1R = Receive FIFO 1 Register: FIFO1'in bekleyen frame sayisini ve kontrol bitlerini tutar. */
    pending_mask = CAN_RF1R_FMP1; /* FMP1 = FIFO 1 Message Pending: FIFO1'de kac frame bekledigini gosteren alan. */
    release_mask = CAN_RF1R_RFOM1; /* RFOM1 = Release FIFO 1 Output Mailbox: en eski FIFO1 frame'ini serbest birakir. */
  }

  while((*fifo_register & pending_mask) != 0U){ /* Tek IRQ gelmeden once birden cok frame birikmis olabilir; hepsini bosaltir. */
    uint32_t write_sequence = can2_context.rx_write_sequence;
    uint32_t read_sequence = can2_context.rx_read_sequence;

    if((write_sequence - read_sequence) >= CAN_MANAGER_RX_QUEUE_SIZE){
      g_can2_debug.rx_software_overflow_count++; /* Yazilim kuyrugu doluysa yeni frame'in kaybedildigini sayar. */
      *fifo_register = release_mask; /* Dolu yazilim kuyrugu yuzunden donanim FIFO'sunun da kilitlenmesini engeller. */
      continue;
    }

    uint32_t write_index = write_sequence % CAN_MANAGER_RX_QUEUE_SIZE;
    can_frame_t* queue_frame = &can2_context.rx_queue[write_index]; /* Donanimdan gelen frame'in son kez yazilacagi bos hucreyi secer. */
    decode_fifo_frame(can, fifo, queue_frame); /* Donanim FIFO register'larini ara frame kullanmadan dogrudan bos kuyruk hucresine yazar. */

    __DMB(); /* Frame'in butun alanlari RAM'e yazilmadan yeni sequence degerinin task tarafindan gorulmesini engeller. */
    can2_context.rx_write_sequence = write_sequence + 1; /* Tek atomik 32-bit yazmayla yeni frame'i RX task'a yayinlar. */
    g_can2_debug.rx_received_count++; /* Donanim FIFO'sundan yazilim kuyruguna aktarilan frame sayisini artirir. */

    *fifo_register = release_mask; /* Okunan donanim frame'ini serbest birakir ve FIFO'daki siradaki frame'i cikisa getirir. */
  }
}

can_manager_status_t can_manager_init(CAN_TypeDef* can, osThreadId_t rx_notification_task, osThreadId_t tx_notification_task){
  if((can != CAN2) || (rx_notification_task == NULL) || (tx_notification_task == NULL)){ /* Her iki ISR hedefi de olmadan kurulum guvenli degildir. */
    g_can2_debug.last_status = can_manager_invalid_argument; /* Hatayi debugger icin kaydeder. */
    return can_manager_invalid_argument; /* Gecersiz adreslerle register/RTOS islemi yapmadan cikar. */
  }

  can2_context.can = can; /* Sonraki manager islemlerinin CAN2 ile iliskisini context'e kaydeder. */
  can2_context.rx_notification_task = rx_notification_task; /* RX ISR'nin uyandiracagi task'i kaydeder. */
  can2_context.tx_notification_task = tx_notification_task; /* TX ISR'nin uyandiracagi task'i kaydeder. */
  can2_context.rx_write_sequence = 0; /* ISR uretici sayacini sifirlar. */
  can2_context.rx_read_sequence = 0; /* RX task tuketici sayacini sifirlar. */
  g_can2_last_rx_frame_ptr = NULL;
  can2_context.initialized = false; /* Normal moda gecis tamamlanana kadar send/receive girisini kapatir. */
  g_can2_debug.ack_wait_start_tick = 0;
  g_can2_debug.ack_waiting = 0;
  g_can2_debug.ack_timeout_reported = 0;

  can->IER = 0U; /* IER = Interrupt Enable Register: butun CAN interrupt kaynaklarini gecici kapatir; yari yapilandirilmis donanim ISR calistiramaz. */
  can->MCR &= ~(CAN_MCR_SLEEP | CAN_MCR_NART | CAN_MCR_RFLM | CAN_MCR_TXFP); /* MCR = Master Control Register; SLEEP=Sleep Mode, NART=No Automatic Retransmission, RFLM=Receive FIFO Locked Mode, TXFP=Transmit FIFO Priority. Bu bitleri 0 yaparak uyanik, otomatik tekrarli, FIFO overwrite ve ID-oncelikli davranis secer. */
  can->MCR |= CAN_MCR_ABOM; /* ABOM = Automatic Bus-Off Management: cok hata sonrasi bus-off olan bxCAN'in protokoldeki bekleme tamamlaninca otomatik normale donmesini saglar. */

  configure_can2_accept_all_filter(); /* Gelen her ID'nin FIFO0'a kabul edilmesini saglar; filtresiz bxCAN frame teslim etmez. */

  while((can->RF0R & CAN_RF0R_FMP0) != 0U) can->RF0R = CAN_RF0R_RFOM0; /* RF0R = Receive FIFO0 Register; FMP0=FIFO0 Message Pending sifir olana kadar RFOM0=Release FIFO0 Output Mailbox komutuyla eski frameleri atar. */
  while((can->RF1R & CAN_RF1R_FMP1) != 0U) can->RF1R = CAN_RF1R_RFOM1; /* RF1R = Receive FIFO1 Register; FMP1 bekleyen mesaj sayisidir, RFOM1 en eski FIFO1 frame'ini serbest birakir. */

  can->TSR = CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2; /* TSR = Transmit Status Register; RQCP0/1/2 = Request Completed Mailbox 0/1/2. Bu alanlar W1C'dir: bite 1 yazmak biti set etmez, donanimdaki eski tamamlanma bayragini temizler. */
  can->MSR = CAN_MSR_ERRI | CAN_MSR_WKUI | CAN_MSR_SLAKI; /* MSR = Master Status Register; ERRI=Error Interrupt, WKUI=Wake-Up Interrupt, SLAKI=Sleep Acknowledge Interrupt. W1C ile onceki interrupt nedenlerini temizler. */
  can->ESR &= ~CAN_ESR_LEC; /* ESR = Error Status Register; LEC = Last Error Code. Alani 000 yaparak onceki bit/stuff/ACK/form/CRC hata kodunu temizler. */

  can->IER = CAN_IER_TMEIE |
          CAN_IER_FMPIE0 | CAN_IER_FOVIE0 |
          CAN_IER_FMPIE1 | CAN_IER_FOVIE1 |
          CAN_IER_EWGIE | CAN_IER_EPVIE | CAN_IER_BOFIE |
          CAN_IER_LECIE | CAN_IER_ERRIE; /* IER = Interrupt Enable Register; TMEIE=Transmit Mailbox Empty, FMPIE=FIFO Message Pending, FOVIE=FIFO Overrun, EWGIE=Error Warning, EPVIE=Error Passive, BOFIE=Bus-Off, LECIE=Last Error Code, ERRIE=genel Error interrupt izinleridir. */

  can->MCR &= ~CAN_MCR_INRQ; /* MCR.INRQ = Initialization Request: biti 0 yapmak bxCAN'e init modundan cik ve CANH/CANL hattina katil istegi verir. */

  uint32_t start_tick = osKernelGetTickCount(); /* Normal moda gecisin ne kadar surdugunu olcmek icin baslangic zamani. */
  while((can->MSR & CAN_MSR_INAK) != 0U){ /* MSR.INAK = Initialization Acknowledge: 1 iken donanim init modunda oldugunu onaylar; 0 olunca normal moda gecmis ve iletisime hazirdir. */
    if((osKernelGetTickCount() - start_tick) >= CAN_MANAGER_START_TIMEOUT_MS){ /* Donanim normal moda gecmeyi kabul etmezse sonsuz beklemeyi keser. */
      g_can2_debug.last_status = can_manager_start_timeout; /* Basarisizligin tam turunu kaydeder. */
      can_manager_refresh_debug(can); /* Hata anindaki registerlari inceleme icin dondurur. */
      return can_manager_start_timeout; /* Hazir olmayan manager'i initialized yapmadan cikar. */
    }
  }

  can2_context.initialized = true; /* Artik send/receive fonksiyonlarinin CAN2'yi kullanmasina izin verir. */
  g_can2_debug.initialized = 1U; /* Debugger'da basarili kurulumun gorunur isareti. */
  g_can2_debug.init_count++; /* Init'in kac kez basariyla tamamlandigini sayar. */
  g_can2_debug.last_status = can_manager_ok; /* Son manager isleminin basarili oldugunu kaydeder. */
  can_manager_refresh_debug(can); /* Normal moda gecildikten sonraki register durumunu kaydeder. */
  notify_tx_task(CAN_MANAGER_TX_EVENT_READY); /* Yazma task'i ancak bxCAN normal moda gectikten sonra calismaya baslar. */
  return can_manager_ok; /* Task'a manager'in kullanima hazir oldugunu bildirir. */
}

can_manager_status_t can_manager_send(CAN_TypeDef* can, const can_frame_t* frame){
  g_can2_debug.send_call_count++; /* Basarili/basarisiz fark etmeksizin kac gonderme istegi geldigini sayar. */

  if((can != CAN2) || (frame == NULL) || (frame->dlc > 8U) ||
          (!frame->extended_id && (frame->id > 0x7FFU)) ||
          (frame->extended_id && (frame->id > 0x1FFFFFFFU))){ /* Peripheral, pointer, DLC ve 11/29-bit ID sinirlarini dogrular. */
    g_can2_debug.last_status = can_manager_invalid_argument; /* Hatali uygulama istegini kaydeder. */
    return can_manager_invalid_argument; /* Gecersiz frame'i mailbox registerlarina yazmayarak belirsiz davranisi onler. */
  }

  if(!can2_context.initialized){ /* CAN init ve normal moda gecis tamamlanmis mi kontrol eder. */
    g_can2_debug.last_status = can_manager_not_initialized; /* Erken send cagrisinin neden reddedildigini kaydeder. */
    return can_manager_not_initialized; /* Init modundaki donanima TXRQ vermeyi engeller. */
  }

  uint32_t tsr = can->TSR; /* TSR = Transmit Status Register: uc TX mailbox'in bosluk ve son iletim sonucunu tek snapshot'ta alir. */
  uint8_t mailbox_index; /* Frame'in yazilacagi bos mailbox numarasini tutacak. */

  if((tsr & CAN_TSR_TME0) != 0U) mailbox_index = 0U; /* TME0 = Transmit Mailbox 0 Empty: 1 ise mailbox 0 yeni frame kabul edebilir. */
  else if((tsr & CAN_TSR_TME1) != 0U) mailbox_index = 1U; /* TME1 = Transmit Mailbox 1 Empty: mailbox 0 doluysa bos olan mailbox 1'i secer. */
  else if((tsr & CAN_TSR_TME2) != 0U) mailbox_index = 2U; /* TME2 = Transmit Mailbox 2 Empty: ilk iki mailbox doluysa mailbox 2'yi secer. */
  else{
    g_can2_debug.no_tx_mailbox_count++; /* Uc mailbox'in da mesgul oldugu ani sayar. */
    g_can2_debug.last_status = can_manager_no_tx_mailbox; /* Frame'in neden kabul edilmedigini kaydeder. */
    can_manager_refresh_debug(can); /* Mailbox dolulugunu gosteren TSR'yi saklar. */
    return can_manager_no_tx_mailbox; /* Mevcut mailbox verisinin ustune yazmak yerine caller'in sonra denemesini ister. */
  }

  CAN_TxMailBox_TypeDef* mailbox = &can->sTxMailBox[mailbox_index]; /* Secilen TX mailbox grubuna erisir: TIR=Identifier, TDTR=Data Length/Time, TDLR=Data Low, TDHR=Data High register'lari. */

  uint32_t tir;

  if(frame->extended_id){
      tir = ((frame->id & 0x1FFFFFFF) << 3) | CAN_TI0R_IDE;
  }
  else{
      tir = (frame->id & 0x7FF) << 21;
  }

  mailbox->TIR = tir; /* TIR = Transmit Identifier Register: ID, IDE ve RTR alanlarini yukler; TXRQ henuz 0 oldugu icin bxCAN hatta cikmaz. */
  mailbox->TDTR = frame->dlc & CAN_TDT0R_DLC; /* TDTR = Transmit Data Length/Time Register; DLC = Data Length Code alanina payload byte sayisini yazar. */
  mailbox->TDLR = ((uint32_t)frame->data[0] << 0U) |
          ((uint32_t)frame->data[1] << 8U) |
          ((uint32_t)frame->data[2] << 16U) |
          ((uint32_t)frame->data[3] << 24U); /* TDLR = Transmit Data Low Register: payload byte 0..3'u ilgili sekizer bitlik alanlara paketler. */
  mailbox->TDHR = ((uint32_t)frame->data[4] << 0U) |
          ((uint32_t)frame->data[5] << 8U) |
          ((uint32_t)frame->data[6] << 16U) |
          ((uint32_t)frame->data[7] << 24U); /* TDHR = Transmit Data High Register: payload byte 4..7'yi ilgili sekizer bitlik alanlara paketler. */

  copy_frame_to_volatile(&g_can2_last_tx_frame, frame); /* Hatta cikmasi istenen son frame'i debugger'da gorunur yapar. */
  g_can2_debug.tx_requested_count++; /* Donanima teslim edilen gecerli TX isteklerini sayar. */
  g_can2_debug.last_status = can_manager_ok; /* Yazilim mailbox yuklemesini basarili kabul eder; bu henuz fiziksel ACK demek degildir. */
  mailbox->TIR = tir | CAN_TI0R_TXRQ; /* TXRQ = Transmit Mailbox Request: FIZIKSEL ILETIMI BASLATAN bittir; 1 olunca bxCAN hat bosaldiginda arbitration'a katilir ve CTX pininden bitleri SN65HVD230'a yollar. */

  uint32_t request_tick = osKernelGetTickCount();
  g_can2_debug.last_tx_request_tick = request_tick;
  if(g_can2_debug.ack_waiting == 0){
    g_can2_debug.ack_waiting = 1;
    g_can2_debug.ack_timeout_reported = 0;
    g_can2_debug.ack_wait_start_tick = request_tick;
  }

  can_manager_refresh_debug(can); /* TXRQ verildikten sonraki mailbox ve CAN durumunu debug icin saklar. */
  return can_manager_ok; /* Yalnizca istek donanima verildi demektir; kesin iletim sonucu TX interrupt'inda belli olur. */
}

can_manager_status_t can_manager_check_ack_timeout(CAN_TypeDef* can, uint32_t timeout_ms){
  if((can != CAN2) || (timeout_ms == 0)) return can_manager_invalid_argument;
  if(!can2_context.initialized) return can_manager_not_initialized;
  if(g_can2_debug.ack_waiting == 0) return can_manager_ok;

  uint32_t now = osKernelGetTickCount();
  if((now - g_can2_debug.ack_wait_start_tick) < timeout_ms) return can_manager_ok;
  if(g_can2_debug.ack_timeout_reported != 0) return can_manager_ack_timeout;

  g_can2_debug.ack_timeout_reported = 1;
  g_can2_debug.ack_timeout_count++;
  g_can2_debug.last_status = can_manager_ack_timeout;
  can_manager_refresh_debug(can);
  return can_manager_ack_timeout;
}

can_manager_status_t can_manager_recover(CAN_TypeDef* can){
  /* Onceki recovery denemesi initialized=false birakmis olsa bile tekrar
   * denenebilmelidir. Context ve task hedefleri varsa manager kurulmustur. */
  if((can != CAN2) || (can2_context.can != can) ||
          (can2_context.rx_notification_task == NULL) ||
          (can2_context.tx_notification_task == NULL)){
    return can_manager_not_initialized;
  }

  osThreadId_t rx_task = can2_context.rx_notification_task;
  osThreadId_t tx_task = can2_context.tx_notification_task;
  uint32_t saved_btr = can->BTR;
  g_can2_debug.recovery_attempt_count++;

  can2_context.initialized = false;
  g_can2_debug.initialized = 0U;
  can->IER = 0;
  can->TSR = CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2;
  can->MCR |= CAN_MCR_INRQ;
  uint32_t start_tick = osKernelGetTickCount();
  while((can->MSR & CAN_MSR_INAK) == 0){
    if((osKernelGetTickCount() - start_tick) >= CAN_MANAGER_START_TIMEOUT_MS){
      /* Hata/bus-off durumundaki controller init istegine cevap vermiyorsa
       * CAN2'yi donanimsal resetle. Bit timing resetten sonra geri yuklenir. */
      RCC->APB1RSTR |= RCC_APB1RSTR_CAN2RST;
      __DSB();
      RCC->APB1RSTR &= ~RCC_APB1RSTR_CAN2RST;
      __DSB();

      can->MCR |= CAN_MCR_INRQ;
      start_tick = osKernelGetTickCount();
      while((can->MSR & CAN_MSR_INAK) == 0){
        if((osKernelGetTickCount() - start_tick) >=
                CAN_MANAGER_START_TIMEOUT_MS){
          g_can2_debug.recovery_failure_count++;
          g_can2_debug.last_status = can_manager_recovery_failed;
          can_manager_refresh_debug(can);
          return can_manager_recovery_failed;
        }
      }
      can->BTR = saved_btr;
      break;
    }
  }

  can_manager_status_t status = can_manager_init(can, rx_task, tx_task);
  if(status == can_manager_ok){
    g_can2_debug.recovery_success_count++;
    return can_manager_ok;
  }else{
    g_can2_debug.recovery_failure_count++;
    g_can2_debug.last_status = can_manager_recovery_failed;
    return can_manager_recovery_failed;
  }
}

const can_frame_t* can_manager_rx_peek(CAN_TypeDef* can){
  if((can != CAN2) || !can2_context.initialized) return NULL;

  uint32_t read_sequence = can2_context.rx_read_sequence;
  uint32_t write_sequence = can2_context.rx_write_sequence;
  if(read_sequence == write_sequence) return NULL;

  __DMB(); /* ISR'nin sequence'den once tamamladigi frame yazilarini bu noktadan sonra gorunur kilar. */
  const can_frame_t* frame = &can2_context.rx_queue[read_sequence % CAN_MANAGER_RX_QUEUE_SIZE];
  g_can2_last_rx_frame_ptr = frame; /* Live Expressions dogrudan gercek kuyruk hucresini gosterir; yani kopyasi uretilmez. */
  return frame;
}

can_manager_status_t can_manager_rx_release(CAN_TypeDef* can){
  if(can != CAN2) return can_manager_invalid_argument;
  if(!can2_context.initialized) return can_manager_not_initialized;
  if(can2_context.rx_read_sequence == can2_context.rx_write_sequence) return can_manager_rx_empty;

  __DMB(); /* Task frame'i kullanmayi bitirmeden hucrenin ISR icin bos gorunmesini engeller. */
  can2_context.rx_read_sequence++;
  g_can2_debug.rx_processed_count++;
  return can_manager_ok;
}

void can_manager_tx_irq_handler(CAN_TypeDef* can){
  if((can != CAN2) || !can2_context.initialized) return; /* Yanlis IRQ yonlendirmesini veya init oncesi olayi isleme sokmaz. */

  uint32_t tsr = can->TSR; /* TSR = Transmit Status Register: uc mailbox'in request-complete, success, arbitration-lost ve transmit-error sonucunu tek snapshot'ta alir. */
  g_can2_debug.last_tsr = tsr; /* Debugger'da ISR'yi doguran TX durumunu korur. */

  const uint32_t completion_flags[3] = {CAN_TSR_RQCP0, CAN_TSR_RQCP1, CAN_TSR_RQCP2}; /* RQCP = Request Completed: ilgili mailbox'in iletim isteginin bir sonuca ulastigini bildirir. */
  const uint32_t success_flags[3] = {CAN_TSR_TXOK0, CAN_TSR_TXOK1, CAN_TSR_TXOK2}; /* TXOK = Transmission OK: frame'in tamamlandigini ve ACK slotunda baska bir dugumden dominant ACK alindigini bildirir. */
  const uint32_t arbitration_flags[3] = {CAN_TSR_ALST0, CAN_TSR_ALST1, CAN_TSR_ALST2}; /* ALST = Arbitration Lost: ayni anda konusan daha dusuk ID'li/dominant dugume oncelik verildigini bildirir; protokol hatasi degildir. */
  const uint32_t error_flags[3] = {CAN_TSR_TERR0, CAN_TSR_TERR1, CAN_TSR_TERR2}; /* TERR = Transmission Error: ACK, bit, stuff, form veya CRC kaynakli iletim basarisizligini bildirir. */
  bool any_success = false;
  uint32_t now = osKernelGetTickCount();

  for(uint8_t index = 0U; index < 3U; index++){ /* Ayni IRQ aninda tamamlanmis olabilecek uc mailbox'i da inceler. */
    if((tsr & completion_flags[index]) != 0U){ /* Bu mailbox icin donanimin bir sonuc urettigini kontrol eder. */
      if((tsr & success_flags[index]) != 0U){
        g_can2_debug.tx_complete_count++;
        g_can2_debug.last_tx_ack_tick = now;
        g_can2_debug.last_status = can_manager_ok;
        any_success = true;
      } /* Alicidan ACK gorulmus basarili frame'i sayar. */
      if((tsr & arbitration_flags[index]) != 0U) g_can2_debug.arbitration_lost_count++; /* Hata olmayan arbitration kaybini ayri sayar. */
      if((tsr & error_flags[index]) != 0U){
        g_can2_debug.tx_error_count++;
        g_can2_debug.last_status = can_manager_tx_error;
      } /* Gercek transmit hatalarini sayar. */
      can->TSR = completion_flags[index]; /* TSR.RQCP W1C'dir (Write 1 to Clear): ilgili bite 1 yazarak donanimdaki tamamlanma bayragini 0'a temizler; temizlenmezse TX IRQ tekrar gelebilir. */
    }
  }

  uint32_t all_mailboxes_empty = CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2;
  if((can->TSR & all_mailboxes_empty) == all_mailboxes_empty){
    g_can2_debug.ack_waiting = 0;
    g_can2_debug.ack_timeout_reported = 0;
    g_can2_debug.ack_wait_start_tick = 0;
  }else if(any_success){
    g_can2_debug.ack_wait_start_tick = now;
    g_can2_debug.ack_timeout_reported = 0;
  }

  notify_tx_task(CAN_MANAGER_TX_EVENT_COMPLETE); /* TX sonucu incelenebilsin/yeni is gonderilebilsin diye yazma task'ini uyandirir. */
}

void can_manager_rx0_irq_handler(CAN_TypeDef* can){
  if((can != CAN2) || !can2_context.initialized) return; /* Gecerli ve hazir CAN2 disindaki IRQ'yu islemez. */
  g_can2_debug.rx_irq_count++; /* FIFO0 interrupt handler'inin kac kez calistigini sayar. */
  if((can->RF0R & CAN_RF0R_FOVR0) != 0U){ /* RF0R.FOVR0 = FIFO0 Overrun: yazilim yeterince hizli bosaltamadigi icin donanim FIFO0'in frame kaybettigini bildirir. */
    g_can2_debug.rx_hardware_overflow_count++; /* Donanim seviyesindeki veri kaybini gorunur yapar. */
    can->RF0R = CAN_RF0R_FOVR0; /* FOVR0 W1C'dir: 1 yazmak overflow durumunu olusturmaz, donanimdaki overflow bayragini temizler. */
  }
  drain_fifo(can, 0U); /* FIFO0'daki butun frameleri hizla yazilim ring buffer'ina tasir. */
  notify_rx_task(CAN_MANAGER_RX_EVENT_FIFO0); /* FIFO0'dan yeni frame hazir oldugunu okuma task'ina bildirir. */
}

void can_manager_rx1_irq_handler(CAN_TypeDef* can){
  if((can != CAN2) || !can2_context.initialized) return; /* Gecerli ve hazir CAN2 disindaki IRQ'yu islemez. */
  g_can2_debug.rx_irq_count++; /* FIFO1 interrupt handler'inin kac kez calistigini sayar. */
  if((can->RF1R & CAN_RF1R_FOVR1) != 0U){ /* RF1R.FOVR1 = FIFO1 Overrun: donanim FIFO1 kapasitesinin asilmasi nedeniyle frame kaybi oldugunu bildirir. */
    g_can2_debug.rx_hardware_overflow_count++; /* FIFO1 kaynakli frame kaybini sayar. */
    can->RF1R = CAN_RF1R_FOVR1; /* FOVR1 W1C'dir: bite 1 yazarak FIFO1 overflow bayragini temizler. */
  }
  drain_fifo(can, 1U); /* FIFO1'deki frameleri yazilim ring buffer'ina aktarir. */
  notify_rx_task(CAN_MANAGER_RX_EVENT_FIFO1); /* FIFO1'den yeni frame hazir oldugunu okuma task'ina bildirir. */
}

void can_manager_sce_irq_handler(CAN_TypeDef* can){
  if((can != CAN2) || !can2_context.initialized) return; /* Init olmamis veya yanlis peripheral'in status-change IRQ'sunu yok sayar. */

  uint32_t esr = can->ESR; /* ESR = Error Status Register: hata seviyelerini, TX/RX error counter'larini ve Last Error Code'u ISR basinda snapshot alir. */
  g_can2_debug.error_irq_count++; /* CAN status/error interruptlarinin toplam sayisini artirir. */
  g_can2_debug.last_esr = esr; /* Son hatanin ESR goruntusunu debugger icin saklar. */
  g_can2_debug.last_msr = can->MSR; /* MSR = Master Status Register: ERRI gibi bu status-change interrupt'ini doguran ana nedeni de saklar. */
  if((esr & CAN_ESR_EWGF) != 0U){
    g_can2_debug.warning_count++;
    g_can2_debug.last_status = can_manager_error_warning;
  } /* EWGF = Error Warning Flag: TX veya RX error counter warning esigine ulastiysa sayar. */
  if((esr & CAN_ESR_EPVF) != 0U){
    g_can2_debug.error_passive_count++;
    g_can2_debug.last_status = can_manager_error_passive;
  } /* EPVF = Error Passive Flag: dugum error-passive seviyesine gectiyse sayar; hata sinyalleme yetkisi sinirlanir. */
  if((esr & CAN_ESR_BOFF) != 0U){
    g_can2_debug.bus_off_count++;
    g_can2_debug.last_status = can_manager_bus_off;
  } /* BOFF = Bus-Off Flag: cok sayida TX hatasi nedeniyle dugum kendini fiziksel CAN trafiginden ayirdiysa sayar. */

  can->ESR &= ~CAN_ESR_LEC; /* ESR.LEC = Last Error Code: onceki bit/stuff/ACK/form/CRC hata kodunu temizler; sonraki hata ayri okunabilir. */
  can->MSR = CAN_MSR_ERRI; /* MSR.ERRI = Error Interrupt: W1C ile 1 yazarak pending hata-interrupt nedenini temizler; aksi halde IRQ yeniden gelebilir. */
  notify_rx_task(CAN_MANAGER_RX_EVENT_ERROR); /* Hata olayini okuma/task-politika tarafina tasir. */
  notify_tx_task(CAN_MANAGER_TX_EVENT_ERROR); /* Bekleyen gonderme varsa yazma task'ini da hatadan haberdar eder. */
}
