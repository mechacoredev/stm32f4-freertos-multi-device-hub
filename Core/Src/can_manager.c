#include "can_manager.h" /* CAN frame tipleri, durum kodlari, RTOS ve STM32 tanimlari burada gelir. */

#define CAN_MANAGER_RX_QUEUE_SIZE 16U /* ISR'nin aldigi fakat task'in henuz islemedigi en fazla frame sayisi. */
#define CAN2_FILTER_BANK          14U /* F407'de CAN1 ve CAN2 ortak filtre RAM'i kullanir; CAN2 bolgesi bank 14'te baslatiliyor. */
#define CAN_MANAGER_START_TIMEOUT_MS 100U /* CAN normal moda gecemezse task'i sonsuza kadar kilitlememek icin ust sinir. */

typedef struct{
  CAN_TypeDef* can; /* Bu context'in yonettigi fiziksel bxCAN peripheral'i. */
  osThreadId_t notification_task; /* ISR olaylarini bekleyen CAN task'inin RTOS kimligi. */
  volatile uint8_t rx_write_index; /* ISR'nin siradaki frame'i ring buffer'da yazacagi hucre. */
  volatile uint8_t rx_read_index; /* Task'in siradaki frame'i ring buffer'dan okuyacagi hucre. */
  volatile uint8_t rx_count; /* Kuyrukta task tarafindan henuz alinmamis frame sayisi. */
  volatile bool initialized; /* Manager hazir olmadan send/receive yapilmasini engelleyen yazilim durumu. */
  can_frame_t rx_queue[CAN_MANAGER_RX_QUEUE_SIZE]; /* Donanim FIFO'su ile task arasindaki yazilim tamponu. */
}can_manager_context_t;

static can_manager_context_t can2_context = {0}; /* Sadece bu dosyanin gorebildigi CAN2 calisma durumu. */

volatile can_manager_debug_t g_can2_debug = {0}; /* Live Expressions icin sayac ve ham register goruntuleri. */
volatile can_frame_t g_can2_last_rx_frame = {0}; /* En son alinan frame'i debugger'da kalici olarak gosterir. */
volatile can_frame_t g_can2_last_tx_frame = {0}; /* En son gonderilmesi istenen frame'i debugger'da gosterir. */

void can_manager_refresh_debug(CAN_TypeDef* can){
  if(can != CAN2) return; /* Bu surumde yalniz CAN2 context'i var; baska peripheral okunmaz. */

  g_can2_debug.last_mcr = can->MCR; /* Master Control: init/sleep ve otomatik davranis ayarlari. */
  g_can2_debug.last_msr_snapshot = can->MSR; /* Master Status: init, sleep ve hata interrupt durumlari. */
  g_can2_debug.last_tsr = can->TSR; /* Hangi TX mailbox bos, tamamlandi veya hata aldi bilgisini saklar. */
  g_can2_debug.last_rf0r = can->RF0R; /* FIFO0'da kac frame var ve overflow oldu mu bilgisini saklar. */
  g_can2_debug.last_ier = can->IER; /* Hangi CAN olaylarinin interrupt uretebildigini gosterir. */
  g_can2_debug.last_btr = can->BTR; /* Baud rate ve bit segment ayarlarinin ham goruntusu. */
  g_can2_debug.last_esr = can->ESR; /* Error counter, bus-off ve son hata turunun ham goruntusu. */
}

static void copy_frame(can_frame_t* destination, const can_frame_t* source){
  destination->id = source->id; /* Mesajin kimligini/hedeflenen anlamini kopyalar. */
  destination->dlc = source->dlc; /* Gecerli veri byte sayisini kopyalar; klasik CAN'da en fazla 8'dir. */
  destination->extended_id = source->extended_id; /* ID'nin 11 bit mi 29 bit mi oldugunu kopyalar. */
  destination->remote_frame = source->remote_frame; /* Data frame mi remote request mi oldugunu kopyalar. */
  destination->timestamp = source->timestamp; /* Donanimin yakaladigi zaman damgasini kopyalar. */

  for(uint8_t index = 0U; index < 8U; index++){
    destination->data[index] = source->data[index]; /* Butun fiziksel data register alanini kopyalar; DLC kullanilacak kismi belirler. */
  }
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

static void notify_can_task(uint32_t flag){
  if(can2_context.notification_task != NULL){ /* NULL task'a bildirim yapip RTOS hatasi olusturmayi engeller. */
    (void)osThreadFlagsSet(can2_context.notification_task, flag); /* ISR olayini bekleyen task'i uyandirir; task polling yapmaz. */
  }
}

static void configure_can2_accept_all_filter(void){
  uint32_t filter_bit = 1UL << CAN2_FILTER_BANK; /* Bank 14'u filtre kontrol register'larinda secen tek bitlik maske. */

  CAN1->FMR |= CAN_FMR_FINIT; /* Filtre RAM'ini duzenlemek icin filtreleri init moduna alir; yapilmazsa guvenli ayar yazilamaz. */
  CAN1->FMR = (CAN1->FMR & ~CAN_FMR_CAN2SB) |
          (CAN2_FILTER_BANK << CAN_FMR_CAN2SB_Pos) | CAN_FMR_FINIT; /* Bank 14 ve sonrasini CAN2'ye ayirir; CAN2 kendi ayri filtre RAM'ine sahip degildir. */

  CAN1->FA1R &= ~filter_bit; /* Ayarlarken banki kapatir; yari yazilmis filtreyle frame kabul edilmesini onler. */
  CAN1->FM1R &= ~filter_bit; /* Mask mode secer: ID bitleri FR1 ile, onem maskesi FR2 ile karsilastirilir. */
  CAN1->FS1R |= filter_bit; /* 32-bit filtre olcegini secer; standart ve extended frame alanlari birlikte incelenebilir. */
  CAN1->FFA1R &= ~filter_bit; /* Eslesen frameleri donanim FIFO0'a yonlendirir. */
  CAN1->sFilterRegister[CAN2_FILTER_BANK].FR1 = 0U; /* Beklenen ID degerini sifirlar. */
  CAN1->sFilterRegister[CAN2_FILTER_BANK].FR2 = 0U; /* Maske sifir: hicbir ID biti zorunlu degil, dolayisiyla tum frameler eslesir. */
  CAN1->FA1R |= filter_bit; /* Hazir filtre bankini aktif eder; bu olmazsa alinan frameler FIFO'ya giremez. */
  CAN1->FMR &= ~CAN_FMR_FINIT; /* Filtre init modundan cikar ve filtreleme donanimini tekrar calistirir. */
}

static void decode_fifo_frame(CAN_TypeDef* can, uint8_t fifo, can_frame_t* frame){
  CAN_FIFOMailBox_TypeDef* mailbox = &can->sFIFOMailBox[fifo]; /* FIFO0 veya FIFO1'in cikis mailbox register grubunu secer. */
  uint32_t rir = mailbox->RIR; /* ID, IDE ve RTR bitlerini tek snapshot olarak alir. */
  uint32_t rdtr = mailbox->RDTR; /* DLC ve timestamp alanlarini okur. */
  uint32_t rdlr = mailbox->RDLR; /* Data byte 0..3'u tek 32-bit okumayla alir. */
  uint32_t rdhr = mailbox->RDHR; /* Data byte 4..7'yi tek 32-bit okumayla alir. */

  frame->extended_id = (rir & CAN_RI0R_IDE) != 0U; /* IDE dominant/0 ise 11-bit, recessive/1 ise 29-bit ID kullanilmistir. */
  frame->remote_frame = (rir & CAN_RI0R_RTR) != 0U; /* RTR=1 ise veri tasimayan remote request frame'dir. */
  frame->id = frame->extended_id ? ((rir >> 3U) & 0x1FFFFFFFU) : ((rir >> 21U) & 0x7FFU); /* ID'yi donanimdaki hizali yerinden normal sayiya cevirir. */
  frame->dlc = (uint8_t)(rdtr & CAN_RDT0R_DLC); /* Alt dort bitten gecerli payload byte sayisini alir. */
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

static void store_received_frame(const can_frame_t* frame){
  if(can2_context.rx_count >= CAN_MANAGER_RX_QUEUE_SIZE){ /* Task 16 frame geride kalmissa yazilim kuyrugu doludur. */
    g_can2_debug.rx_software_overflow_count++; /* Kaybedilen frame'i sessizce gizlemek yerine sayar. */
    return; /* Dolu tamponun ustune yazip eski ve yeni frameleri bozmamak icin yeni frame'i birakir. */
  }

  copy_frame(&can2_context.rx_queue[can2_context.rx_write_index], frame); /* Frame'i ISR'den task'a aktarilacak siradaki hucreye koyar. */
  can2_context.rx_write_index = (uint8_t)((can2_context.rx_write_index + 1U) % CAN_MANAGER_RX_QUEUE_SIZE); /* Sona gelince sifira donen ring index. */
  can2_context.rx_count++; /* Task'a okunabilir bir frame daha oldugunu bildirir. */
  copy_frame_to_volatile(&g_can2_last_rx_frame, frame); /* Live Expressions'da son frame'i gorunur tutar. */
  g_can2_debug.rx_received_count++; /* Donanim FIFO'sundan yazilim kuyruguna basariyla alinan frame sayisi. */
}

static void drain_fifo(CAN_TypeDef* can, uint8_t fifo){
  volatile uint32_t* fifo_register = fifo == 0U ? &can->RF0R : &can->RF1R; /* Secilen FIFO'nun durum register adresi. */
  uint32_t pending_mask = fifo == 0U ? CAN_RF0R_FMP0 : CAN_RF1R_FMP1; /* Bekleyen frame sayisini kontrol edecek bit alani. */
  uint32_t release_mask = fifo == 0U ? CAN_RF0R_RFOM0 : CAN_RF1R_RFOM1; /* Okunan en eski mailbox'i FIFO'dan cikaran komut biti. */

  while((*fifo_register & pending_mask) != 0U){ /* Tek IRQ gelmeden once birden cok frame birikmis olabilir; hepsini bosaltir. */
    can_frame_t frame; /* Donanim register'larini rahat islenecek yazilim formatina cevirecek gecici frame. */
    decode_fifo_frame(can, fifo, &frame); /* FIFO'nun en eski frame'ini release etmeden once kopyalar. */
    store_received_frame(&frame); /* Frame'i task'in okuyacagi yazilim ring buffer'ina aktarir. */
    *fifo_register = release_mask; /* FIFO cikisini serbest birakir; yapilmazsa ayni frame basta kalir ve FIFO dolar. */
  }
}

can_manager_status_t can_manager_init(CAN_TypeDef* can, osThreadId_t notification_task){
  if((can != CAN2) || (notification_task == NULL)){ /* Desteklenmeyen peripheral veya bildirilecek task yoksa kurulum guvenli degildir. */
    g_can2_debug.last_status = can_manager_invalid_argument; /* Hatayi debugger icin kaydeder. */
    return can_manager_invalid_argument; /* Gecersiz adreslerle register/RTOS islemi yapmadan cikar. */
  }

  can2_context.can = can; /* Sonraki manager islemlerinin CAN2 ile iliskisini context'e kaydeder. */
  can2_context.notification_task = notification_task; /* ISR'nin hangi task'i uyandiracagini kaydeder. */
  can2_context.rx_write_index = 0U; /* Yazilim RX kuyrugunu bastan yazmaya hazirlar. */
  can2_context.rx_read_index = 0U; /* Yazilim RX kuyrugunu bastan okumaya hazirlar. */
  can2_context.rx_count = 0U; /* Eski calismadan kalmis frame varmis gibi davranilmasini engeller. */
  can2_context.initialized = false; /* Normal moda gecis tamamlanana kadar send/receive girisini kapatir. */

  can->IER = 0U; /* Ayarlar degisirken yari yapilandirilmis olaylarin ISR cagirmasini engeller. */
  can->MCR &= ~(CAN_MCR_SLEEP | CAN_MCR_NART | CAN_MCR_RFLM | CAN_MCR_TXFP); /* Uyanik, otomatik tekrarli, FIFO overwrite ve ID-oncelikli normal davranis secer. */
  can->MCR |= CAN_MCR_ABOM; /* Bus-off olursa bxCAN'in protokolde gereken beklemeden sonra otomatik toparlanmasini acar. */

  configure_can2_accept_all_filter(); /* Gelen her ID'nin FIFO0'a kabul edilmesini saglar; filtresiz bxCAN frame teslim etmez. */

  while((can->RF0R & CAN_RF0R_FMP0) != 0U) can->RF0R = CAN_RF0R_RFOM0; /* FIFO0'daki eski/stale frameleri tek tek serbest birakir. */
  while((can->RF1R & CAN_RF1R_FMP1) != 0U) can->RF1R = CAN_RF1R_RFOM1; /* FIFO1'i de temiz bir baslangica getirir. */

  can->TSR = CAN_TSR_RQCP0 | CAN_TSR_RQCP1 | CAN_TSR_RQCP2; /* Uc TX mailbox'in onceki tamamlanma durumlarini W1C ile temizler. */
  can->MSR = CAN_MSR_ERRI | CAN_MSR_WKUI | CAN_MSR_SLAKI; /* Eski error/wakeup/sleep interrupt nedenlerini temizler. */
  can->ESR &= ~CAN_ESR_LEC; /* Last Error Code'u sifirlar; yeni hata eski hata ile karismaz. */

  can->IER = CAN_IER_TMEIE |
          CAN_IER_FMPIE0 | CAN_IER_FOVIE0 |
          CAN_IER_FMPIE1 | CAN_IER_FOVIE1 |
          CAN_IER_EWGIE | CAN_IER_EPVIE | CAN_IER_BOFIE |
          CAN_IER_LECIE | CAN_IER_ERRIE; /* TX bitisi, RX gelisi/overflow ve CAN hata durumlarini NVIC'ye baglar; task polling yapmaz. */

  can->MCR &= ~CAN_MCR_INRQ; /* Yazilimsal olarak iletisimi baslatir: bxCAN'e init modundan normal moda cikma istegi verir. */

  uint32_t start_tick = osKernelGetTickCount(); /* Normal moda gecisin ne kadar surdugunu olcmek icin baslangic zamani. */
  while((can->MSR & CAN_MSR_INAK) != 0U){ /* INAK sifirlanana kadar donanim hala init modundadir ve hatta frame gonderemez. */
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

  uint32_t tsr = can->TSR; /* Uc transmit mailbox'in bos/dolu durumunu tek snapshot'ta alir. */
  uint8_t mailbox_index; /* Frame'in yazilacagi bos mailbox numarasini tutacak. */

  if((tsr & CAN_TSR_TME0) != 0U) mailbox_index = 0U; /* Mailbox 0 bossa ilk tercih olarak onu kullanir. */
  else if((tsr & CAN_TSR_TME1) != 0U) mailbox_index = 1U; /* Mailbox 0 doluysa bos mailbox 1'i secer. */
  else if((tsr & CAN_TSR_TME2) != 0U) mailbox_index = 2U; /* Ilk ikisi doluysa bos mailbox 2'yi secer. */
  else{
    g_can2_debug.no_tx_mailbox_count++; /* Uc mailbox'in da mesgul oldugu ani sayar. */
    g_can2_debug.last_status = can_manager_no_tx_mailbox; /* Frame'in neden kabul edilmedigini kaydeder. */
    can_manager_refresh_debug(can); /* Mailbox dolulugunu gosteren TSR'yi saklar. */
    return can_manager_no_tx_mailbox; /* Mevcut mailbox verisinin ustune yazmak yerine caller'in sonra denemesini ister. */
  }

  CAN_TxMailBox_TypeDef* mailbox = &can->sTxMailBox[mailbox_index]; /* Secilen mailbox'in TIR/TDTR/TDLR/TDHR register grubuna isaret eder. */
  uint32_t tir = frame->extended_id ? ((frame->id & 0x1FFFFFFFU) << 3U) | CAN_TI0R_IDE :
          ((frame->id & 0x7FFU) << 21U); /* 29-bit veya 11-bit ID'yi bxCAN'in TIR icinde bekledigi konuma hizalar. */
  if(frame->remote_frame) tir |= CAN_TI0R_RTR; /* Remote frame isteniyorsa RTR'yi recessive/1 yapar; data frame'de 0 kalir. */

  mailbox->TIR = tir; /* ID/IDE/RTR bilgisini TX mailbox'a yukler fakat TXRQ=0 oldugu icin henuz iletim baslamaz. */
  mailbox->TDTR = frame->dlc & CAN_TDT0R_DLC; /* Aliciya bu frame'de kac payload byte oldugunu bildirecek DLC alanini yukler. */
  mailbox->TDLR = ((uint32_t)frame->data[0] << 0U) |
          ((uint32_t)frame->data[1] << 8U) |
          ((uint32_t)frame->data[2] << 16U) |
          ((uint32_t)frame->data[3] << 24U); /* Ilk dort payload byte'ini donanimin alt data register'ina paketler. */
  mailbox->TDHR = ((uint32_t)frame->data[4] << 0U) |
          ((uint32_t)frame->data[5] << 8U) |
          ((uint32_t)frame->data[6] << 16U) |
          ((uint32_t)frame->data[7] << 24U); /* Son dort payload byte'ini donanimin ust data register'ina paketler. */

  copy_frame_to_volatile(&g_can2_last_tx_frame, frame); /* Hatta cikmasi istenen son frame'i debugger'da gorunur yapar. */
  g_can2_debug.tx_requested_count++; /* Donanima teslim edilen gecerli TX isteklerini sayar. */
  g_can2_debug.last_status = can_manager_ok; /* Yazilim mailbox yuklemesini basarili kabul eder; bu henuz fiziksel ACK demek degildir. */
  mailbox->TIR = tir | CAN_TI0R_TXRQ; /* FIZIKSEL ILETIMI BASLATAN SATIR: TX request verilir, bxCAN bus bosalinca arbitration'a katilir. */
  can_manager_refresh_debug(can); /* TXRQ verildikten sonraki mailbox ve CAN durumunu debug icin saklar. */
  return can_manager_ok; /* Yalnizca istek donanima verildi demektir; kesin iletim sonucu TX interrupt'inda belli olur. */
}

can_manager_status_t can_manager_receive(CAN_TypeDef* can, can_frame_t* frame){
  if((can != CAN2) || (frame == NULL)) return can_manager_invalid_argument; /* Gecersiz peripheral veya hedef RAM adresini reddeder. */
  if(!can2_context.initialized) return can_manager_not_initialized; /* Manager hazir degilken kuyruk okumayi engeller. */
  if(can2_context.rx_count == 0U) return can_manager_rx_empty; /* Kuyruk bossa caller'i bekletmeden 'frame yok' der. */

  uint32_t primask = __get_PRIMASK(); /* Fonksiyona girerken interruptlar zaten kapali miydi bilgisini saklar. */
  __disable_irq(); /* ISR ayni anda rx_count/index degistirip task'in yari frame okumasini engellemek icin kisa kritik bolge acar. */
  copy_frame(frame, &can2_context.rx_queue[can2_context.rx_read_index]); /* Kuyruktaki en eski frame'i caller'in RAM alanina kopyalar. */
  can2_context.rx_read_index = (uint8_t)((can2_context.rx_read_index + 1U) % CAN_MANAGER_RX_QUEUE_SIZE); /* Okuma basini siradaki hucreye ilerletir. */
  can2_context.rx_count--; /* Kuyrukta bekleyen frame sayisini bir azaltir. */
  if(primask == 0U) __enable_irq(); /* Interruptlar giriste aciksa geri acar; onceden kapaliysa yanlislikla acmaz. */

  g_can2_debug.rx_processed_count++; /* Task'in basariyla tukettigi frame sayisini sayar. */
  return can_manager_ok; /* Caller'a frame yapisinin dolduruldugunu bildirir. */
}

void can_manager_tx_irq_handler(CAN_TypeDef* can){
  if((can != CAN2) || !can2_context.initialized) return; /* Yanlis IRQ yonlendirmesini veya init oncesi olayi isleme sokmaz. */

  uint32_t tsr = can->TSR; /* Uc mailbox'in tamamlanma sonucunu ayni anda snapshot olarak alir. */
  g_can2_debug.last_tsr = tsr; /* Debugger'da ISR'yi doguran TX durumunu korur. */

  const uint32_t completion_flags[3] = {CAN_TSR_RQCP0, CAN_TSR_RQCP1, CAN_TSR_RQCP2}; /* Her mailbox icin request-complete biti. */
  const uint32_t success_flags[3] = {CAN_TSR_TXOK0, CAN_TSR_TXOK1, CAN_TSR_TXOK2}; /* Frame ACK dahil basariyla tamamlandi bitleri. */
  const uint32_t arbitration_flags[3] = {CAN_TSR_ALST0, CAN_TSR_ALST1, CAN_TSR_ALST2}; /* Daha dominant ID'ye kaybedilen arbitration bitleri. */
  const uint32_t error_flags[3] = {CAN_TSR_TERR0, CAN_TSR_TERR1, CAN_TSR_TERR2}; /* ACK/bit/stuff/form gibi nedenle basarisiz TX bitleri. */

  for(uint8_t index = 0U; index < 3U; index++){ /* Ayni IRQ aninda tamamlanmis olabilecek uc mailbox'i da inceler. */
    if((tsr & completion_flags[index]) != 0U){ /* Bu mailbox icin donanimin bir sonuc urettigini kontrol eder. */
      if((tsr & success_flags[index]) != 0U) g_can2_debug.tx_complete_count++; /* Alicidan ACK gorulmus basarili frame'i sayar. */
      if((tsr & arbitration_flags[index]) != 0U) g_can2_debug.arbitration_lost_count++; /* Hata olmayan arbitration kaybini ayri sayar. */
      if((tsr & error_flags[index]) != 0U) g_can2_debug.tx_error_count++; /* Gercek transmit hatalarini sayar. */
      can->TSR = completion_flags[index]; /* RQCP W1C bitini temizler; yapilmazsa TX IRQ nedeni aktif kalabilir. */
    }
  }

  notify_can_task(CAN_MANAGER_EVENT_TX); /* TX sonucu incelenebilsin/yeni is gonderilebilsin diye CAN task'ini uyandirir. */
}

void can_manager_rx0_irq_handler(CAN_TypeDef* can){
  if((can != CAN2) || !can2_context.initialized) return; /* Gecerli ve hazir CAN2 disindaki IRQ'yu islemez. */
  g_can2_debug.rx_irq_count++; /* FIFO0 interrupt handler'inin kac kez calistigini sayar. */
  if((can->RF0R & CAN_RF0R_FOVR0) != 0U){ /* Task/ISR gec kaldigi icin donanim FIFO0 frame kaybetti mi kontrol eder. */
    g_can2_debug.rx_hardware_overflow_count++; /* Donanim seviyesindeki veri kaybini gorunur yapar. */
    can->RF0R = CAN_RF0R_FOVR0; /* Overflow bayragini W1C ile temizler; temizlenmezse hata durumu aktif kalir. */
  }
  drain_fifo(can, 0U); /* FIFO0'daki butun frameleri hizla yazilim ring buffer'ina tasir. */
  notify_can_task(CAN_MANAGER_EVENT_RX); /* Yeni frame hazir oldugunu task'a bildirir. */
}

void can_manager_rx1_irq_handler(CAN_TypeDef* can){
  if((can != CAN2) || !can2_context.initialized) return; /* Gecerli ve hazir CAN2 disindaki IRQ'yu islemez. */
  g_can2_debug.rx_irq_count++; /* FIFO1 interrupt handler'inin kac kez calistigini sayar. */
  if((can->RF1R & CAN_RF1R_FOVR1) != 0U){ /* FIFO1 kapasitesi asilmis mi kontrol eder. */
    g_can2_debug.rx_hardware_overflow_count++; /* FIFO1 kaynakli frame kaybini sayar. */
    can->RF1R = CAN_RF1R_FOVR1; /* FIFO1 overflow bayragini W1C ile temizler. */
  }
  drain_fifo(can, 1U); /* FIFO1'deki frameleri yazilim ring buffer'ina aktarir. */
  notify_can_task(CAN_MANAGER_EVENT_RX); /* Task'in beklemesini bitirip frameleri islemesini saglar. */
}

void can_manager_sce_irq_handler(CAN_TypeDef* can){
  if((can != CAN2) || !can2_context.initialized) return; /* Init olmamis veya yanlis peripheral'in status-change IRQ'sunu yok sayar. */

  uint32_t esr = can->ESR; /* Hata durumu ve sayaclarini ISR basinda snapshot alir; sonradan degisebilirler. */
  g_can2_debug.error_irq_count++; /* CAN status/error interruptlarinin toplam sayisini artirir. */
  g_can2_debug.last_esr = esr; /* Son hatanin ESR goruntusunu debugger icin saklar. */
  g_can2_debug.last_msr = can->MSR; /* Error interrupt nedeninin MSR tarafini da saklar. */
  if((esr & CAN_ESR_EWGF) != 0U) g_can2_debug.warning_count++; /* TX/RX error counter warning esigine geldiyse sayar. */
  if((esr & CAN_ESR_EPVF) != 0U) g_can2_debug.error_passive_count++; /* Dugum error-passive olduysa sayar; hatta etkisi sinirlanmistir. */
  if((esr & CAN_ESR_BOFF) != 0U) g_can2_debug.bus_off_count++; /* Dugum cok hata nedeniyle kendini hattan ayirdiysa sayar. */

  can->ESR &= ~CAN_ESR_LEC; /* Son hata kodunu temizler; sonraki hata olayinin turu ayirt edilebilir. */
  can->MSR = CAN_MSR_ERRI; /* Error interrupt pending nedenini W1C ile temizler; aksi halde IRQ tekrar tetiklenebilir. */
  notify_can_task(CAN_MANAGER_EVENT_ERROR); /* Ayrintili politika ISR'de uzamasin diye hata olayini CAN task'ina tasir. */
}
