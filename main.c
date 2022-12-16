#include <stdio.h>
#include <string.h>

#include "periph/gpio.h"
#include "lptimer.h"
#include "thread.h"

#include "net/lora.h"
#include "net/netdev/lora.h"
#include "sx127x_internal.h"
#include "sx127x_netdev.h"
#include <time.h>
#include "include/vemac-header.h"
#include "include/vemac-crypto.h"

#ifndef VEMAC_ID
#define VEMAC_ID (0)
#endif

#define ENABLE_DEBUG    (0)
#include "debug.h"

/* For UNWD-RANGE board */
#ifndef SX127X_PARAM_PASELECT
#define SX127X_PARAM_PASELECT   (SX127X_PA_RFO)
#endif

#define SX127X_LORA_MSG_QUEUE   (16U)
#define SX127X_STACKSIZE        (2*THREAD_STACKSIZE_DEFAULT)

#define MSG_TYPE_ISR            (0x3456)
#define VEMAC_SLOTS (10)

#define DOWN_LINK                                   1

#define UP_LINK                                     0


static uint8_t LoRaMacNwkSKey[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

typedef struct {
    bool (*appdata_received_cb)(uint8_t *buf, size_t buflen);

    netdev_t *device;       /**< Pointer to the radio PHY structure */
    gpio_t pps_pin;
    
    char isr_stack[SX127X_STACKSIZE];
    kernel_pid_t isr_pid;
    
    char slot_stack[SX127X_STACKSIZE];
    kernel_pid_t slot_pid;
    
    lptimer_t slot_timer;
    
    int fcnt; /* frame counter */
    int scnt; /* slot counter */
    int time_slot; /* time slot selected for transmission */
    
    /* TODO: add VeMAC control structures (received IDs, own ID, etc...) */
    int slots[VEMAC_SLOTS]; /* used to maintain Tx set */
    bool slots_received[VEMAC_SLOTS]; /* used to maintain Nx set together with slots */
    bool received, transmitted;

    int slots_listened;

    int id;

} vemac_t;

static sx127x_params_t sx127x_params = {
    .nss_pin = SX127X_SPI_NSS,
    .spi = SX127X_SPI,

    .dio0_pin = SX127X_DIO0,
    .dio1_pin = SX127X_DIO1,
    .dio2_pin = SX127X_DIO2,
    .dio3_pin = SX127X_DIO3,
    .dio4_pin = SX127X_DIO4,
    .dio5_pin = SX127X_DIO5,
    .reset_pin = SX127X_RESET,
   
    .rfswitch_pin = SX127X_RFSWITCH,
    .rfswitch_active_level = 1,
    .paselect = SX127X_PARAM_PASELECT
};
static sx127x_t sx127x;

static vemac_t vemac;

void *isr_thread(void *arg){
    (void)arg;
    
    static msg_t _msg_q[SX127X_LORA_MSG_QUEUE];
    msg_init_queue(_msg_q, SX127X_LORA_MSG_QUEUE);

    while (1) {
        msg_t msg;
        msg_receive(&msg);
        if (msg.type == MSG_TYPE_ISR) {
            netdev_t *dev = msg.content.ptr;
            dev->driver->isr(dev);
        }
        else {
            DEBUG("[VEMAC] unexpected msg type");
        }
    }
}

/* return true in case of collision */
bool is_collision (vemac_t *vemac, vemac_header_t *header){
    bool his_id_in_nx = false;

    for (int i = 0; i < VEMAC_SLOTS; ++i) {
        if (header->slots_id[9] == vemac->slots[i] && vemac->slots_received[i]) {
            his_id_in_nx = true;
        }
    }

    if (his_id_in_nx) {
        for (int i = 0; i < VEMAC_SLOTS-1; ++i) {
            if (header->slots_id[i] == vemac->id){
                return false;
            }
        }
        
        return true;
    }

    return false;
}



void vemac_recv(vemac_t *vemac, int len, vemac_header_t *header){
    if(len == 46 && header->mHdr == 0xE0 && header->fCtrl == 0x06){
        DEBUG("[VEMAC] Frame %i, slot %i; received header: ", vemac->fcnt, vemac->scnt);
        for (int i = 0; i < VEMAC_SLOTS; ++i){
            DEBUG("%i ", header->slots_id[i]);
        }
        DEBUG("\n");
        vemac->received = true;

        // printf("Len %d\n", len);


        if (vemac->transmitted && is_collision(vemac, header)) {
            DEBUG("[VEMAC] Collision detected!");
            vemac->slots[vemac->time_slot] = 255;
            vemac->time_slot = -1;
            vemac->transmitted = false;
        } else {
            if( vemac->time_slot != -1 ){
                vemac->slots_listened = 0;
            }
        }
        
        vemac->slots[vemac->scnt] = header->slots_id[9];
        vemac->slots_received[vemac->scnt] = true;
        
        for(int i = 0; i < VEMAC_SLOTS-1; i++){
            int lcnt = (vemac->scnt + 1 + i) % VEMAC_SLOTS; /* convert to a local counter */
            if( !vemac->slots_received[lcnt] && lcnt != vemac->time_slot ){ /* do not overwrite neighbors */
                vemac->slots[lcnt] = header->slots_id[i];
            }
        }
    }
}

static void sx127x_handler(netdev_t *dev, netdev_event_t event, void *arg) {
    vemac_t *vemac = (vemac_t *)arg;
    
    if (event == NETDEV_EVENT_ISR) {
        msg_t msg;
        msg.type = MSG_TYPE_ISR;
        msg.content.ptr = dev;
        if (msg_send(&msg, vemac->isr_pid) <= 0) {
            DEBUG("[VEMAC] possibly lost interrupt.");
        }
        return;
    }
    
    switch (event) {
        case NETDEV_EVENT_RX_COMPLETE: {
            int len;
            netdev_lora_rx_info_t packet_info;
            uint8_t message[255];
            uint8_t payload[28];
    
            len = vemac->device->driver->recv(dev, NULL, 0, &packet_info);
            if (len < 0) {
                DEBUG("[VEMAC] RX: bad message, aborting\n");
                break;
            }
            
            vemac->device->driver->recv(dev, message, len, &packet_info);
            DEBUG("[VEMAC] RX: %d bytes, | RSSI: %d dBm | SNR: %d dB\n", (int)len, packet_info.rssi, (int)packet_info.snr);
            /* TODO: здесь вызываем обработчик пакета (точнее, кидаем сообщение в другой поток?) */

            vemac_header_t header;
            deserialize (message, &header);

            uint8_t *nwkSKey = LoRaMacNwkSKey;
            uint32_t mic = 0;
            uint32_t micRx = 0;

            micRx |= message[42];
            micRx |= message[43] << 8;
            micRx |= message[44] << 16;
            micRx |= message[45] << 24;

            // printf("MicRx %ld\n ", micRx);



            LoRaMacComputeMic( message, len-4, nwkSKey, &header, UP_LINK,  &mic );

            // printf("Mic %ld\n ", mic);

            LoRaMacPayloadDecrypt( payload, 28, nwkSKey,&header, UP_LINK, &message[14] );
            // printf("\n");

            // printf("Received\n:");

            // for (int i = 0; i < 28; ++i)
            // {
            //     printf("%d ", payload[i]);
            // }

            // printf("\n");
            // printf("\n");

            if (micRx == mic) vemac_recv(vemac, len, &header);
            break;
        }
        case NETDEV_EVENT_CRC_ERROR:
            DEBUG("[VEMAC] RX CRC failed\n");
            break;

        case NETDEV_EVENT_TX_COMPLETE:
            DEBUG("[VEMAC] transmission done\n");
            
            /* TODO: переключаемся в режим приема правильно */
            uint8_t state = NETOPT_STATE_IDLE;
            vemac->device->driver->set(dev, NETOPT_STATE, &state, sizeof(uint8_t));
            
            break;

        case NETDEV_EVENT_RX_TIMEOUT:
            DEBUG("[VEMAC] RX timeout\n");
            break;

        case NETDEV_EVENT_TX_TIMEOUT:
            /* this should not happen, re-init SX127X here */
            DEBUG("[VEMAC] TX timeout\n");
            vemac->device->driver->init(dev);
            break;
            
        case NETDEV_EVENT_CAD_DONE:
            DEBUG("[VEMAC] CAD done\n");
            break;
            
        case NETDEV_EVENT_CAD_DETECTED:
            DEBUG("[VEMAC] CAD detected\n");
            break;
            
        case NETDEV_EVENT_VALID_HEADER:
            DEBUG("[VEMAC] header received, switch to RX state\n");
            break;

        default:
            DEBUG("[VEMAC] received event #%d\n", (int) event);
            break;
    }
}

void vemac_slot_start(void *arg){
    vemac_t *vemac = (vemac_t *)arg;
    vemac->scnt++;
    
    /* Do not set a timer for the last slot */
    /* TODO: think about the case when GPS reception (and PPS signal) is lost */
    /* How long can we last on our own timer? */
    if(vemac->scnt < VEMAC_SLOTS-1){
        vemac->slot_timer.callback = vemac_slot_start;
        vemac->slot_timer.arg = arg;
        lptimer_set(&(vemac->slot_timer), 1000/VEMAC_SLOTS );
    }
    
    msg_t msg;
    msg_send(&msg, vemac->slot_pid);
}

void pps_handler(void *arg){
    vemac_t *vemac = (vemac_t *)arg;
    
    vemac->fcnt++;
    vemac->scnt=-1;
    lptimer_remove(&(vemac->slot_timer));
    
    vemac_slot_start(arg);
}

bool acquire_free_time_slot(vemac_t *vemac) {
    int free_slots = 0;
    for (int i = 0; i < VEMAC_SLOTS; ++i) {
        if (vemac->slots[i] == 255) {
            ++free_slots;
        }
    }
    
    if (free_slots > 0){
        free_slots = rand() % free_slots;
        
        for (int i = 0; i < VEMAC_SLOTS; ++i) {
            if (vemac->slots[i] == 255) {
                if(free_slots-- == 0){
                    vemac->time_slot = i;
                    DEBUG("[VEMAC] %i acquired time slot %i\n", vemac->id, vemac->time_slot); 
                    vemac->slots_listened = 0;
                    vemac->transmitted = false;
                    
                    return true;
                }
            }
        }
    }
    
    return false;
}

void apply_necessary_options (vemac_header_t* header, vemac_t* vemac)
{



    header->fcnt = vemac->fcnt;

    for (int i = 0; i < VEMAC_SLOTS-1; ++i){
        int lcnt = (vemac->scnt + 1 + i) % VEMAC_SLOTS; /* convert to a local counter */
        
        if(vemac->slots_received[lcnt]){
            header->slots_id[i] = vemac->slots[lcnt];
        } else {
            header->slots_id[i] = 255;
        }
    }

    header->slots_id[9] = vemac->id;
}

void do_send(vemac_t *vemac){
    int pktHeaderLen = 14;
    int LoRaMacTxPayloadLen = 28;

    uint8_t buffer[46];
    uint8_t payload[LoRaMacTxPayloadLen];

    for (int i = 0; i < 28; ++i)
    {
        payload[i] = 0;
    }

    for (int i = 0; i < 46; ++i)
    {
        buffer[i] = 0;
    }

    vemac_header_t header;

    apply_necessary_options (&header, vemac);

    serialize(&header, buffer);


    LoRaMacPayloadEncrypt( payload, LoRaMacTxPayloadLen, LoRaMacNwkSKey,  &header, UP_LINK, &buffer[14] );



    uint32_t mic = 0;

    int LoRaMacBufferPktLen = pktHeaderLen + LoRaMacTxPayloadLen;

    LoRaMacComputeMic( buffer, LoRaMacBufferPktLen, LoRaMacNwkSKey, &header, UP_LINK,  &mic );

    // printf("My Mic %ld\n", mic);

    buffer[LoRaMacBufferPktLen + 0] = mic & 0xFF;
    buffer[LoRaMacBufferPktLen + 1] = ( mic >> 8 ) & 0xFF;
    buffer[LoRaMacBufferPktLen + 2] = ( mic >> 16 ) & 0xFF;
    buffer[LoRaMacBufferPktLen + 3] = ( mic >> 24 ) & 0xFF;

    LoRaMacBufferPktLen += 4;

            iolist_t data = {
        .iol_base = buffer,
        .iol_len = 46,
    };


    // printf("\n");
    // printf("Form\n");
    //   for (int i = 0; i < 28; ++i)
    // {
    //     printf("%d ", payload[i]);
    // }

    // printf("\n");
    // printf("\n");
            
    if (vemac->device->driver->send(vemac->device, &data) < 0) {
        DEBUG("[VEMAC] cannot send, device busy");
    }
    else
    {
        vemac->transmitted = true;
        DEBUG("[VEMAC] time slot %i, sent header ", vemac->scnt);
        for (int i = 0; i < VEMAC_SLOTS+4; ++i){
            DEBUG("%i ", buffer[i]);
        }
        DEBUG("\n");
    }
}


void *slot_thread(void *arg){
    vemac_t *vemac = (vemac_t *)arg;

    while (1) {
        msg_t msg;
        msg_receive(&msg);

        DEBUG("[VEMAC] Starting time slot %i\n", vemac->scnt);
        gpio_clear(LED0_PIN);
        
        int prev_time_slot = (vemac->scnt == 0) ? VEMAC_SLOTS-1 : vemac->scnt - 1;
        if (!vemac->received) {
            vemac->slots_received[prev_time_slot] = false;
        }
        vemac->received = false;

        vemac->slots_listened++;

        if(vemac->slots_listened >= 2*VEMAC_SLOTS){
            bool res = acquire_free_time_slot(vemac);
            
            if(!res){
                DEBUG("Failed to acquire time slot\n");
            }
        }

        if (vemac->scnt == vemac->time_slot) {
            gpio_set(LED0_PIN);
            lptimer_ticks32_t ms = {1};
            lptimer_spin(ms); // a small delay
            do_send(vemac);
        } else {
            uint8_t state = NETOPT_STATE_IDLE;
            vemac->device->driver->set(vemac->device, NETOPT_STATE, &state, sizeof(uint8_t));
        }
    }
}

int vemac_init(vemac_t *vemac, netdev_t *device, gpio_t pps_pin){
    device->driver = &sx127x_driver;
    vemac->device = device;
    
    vemac->isr_pid = thread_create(vemac->isr_stack, sizeof(vemac->isr_stack), THREAD_PRIORITY_MAIN - 1,
                              THREAD_CREATE_STACKTEST, isr_thread, vemac,
                              "SX127x handler thread");

    if (vemac->isr_pid <= KERNEL_PID_UNDEF) {
        puts("ls_init: creation of SX127X ISR thread failed");
        return false;
    }
    
    vemac->slot_pid = thread_create(vemac->slot_stack, sizeof(vemac->slot_stack), THREAD_PRIORITY_MAIN - 2,
                              THREAD_CREATE_STACKTEST, slot_thread, vemac,
                              "VeMAC uplink thread");

    if (vemac->isr_pid <= KERNEL_PID_UNDEF) {
        puts("ls_init: creation of SX127X ISR thread failed");
        return false;
    }
    
    if(vemac->device->driver->init(vemac->device) < 0){
        puts("Netdev driver initialisation failed");
    }
    
    vemac->device->event_callback = sx127x_handler;
    vemac->device->event_callback_arg = vemac;
    
    /* Конфигурируем трансивер, выставляем значимые параметры */
    const netopt_enable_t enable = true;
    const netopt_enable_t disable = false;
    
    puts("[LoRa] reconfigure transceiver\n");
    /* Configure to sleep */
    uint8_t state = NETOPT_STATE_SLEEP;
    vemac->device->driver->set(vemac->device, NETOPT_STATE, &state, sizeof(uint8_t));
    
    uint16_t modem = NETDEV_TYPE_LORA;
    vemac->device->driver->set(vemac->device, NETOPT_DEVICE_TYPE, &modem, sizeof(uint16_t));

    uint8_t sf = LORA_SF7;
    vemac->device->driver->set(vemac->device, NETOPT_SPREADING_FACTOR, &sf, sizeof(uint8_t));
    uint8_t bw = LORA_BW_125_KHZ;
    vemac->device->driver->set(vemac->device, NETOPT_BANDWIDTH, &bw, sizeof(uint8_t));
    uint8_t cr = LORA_CR_4_5;
    vemac->device->driver->set(vemac->device, NETOPT_CODING_RATE, &cr, sizeof(uint8_t));
    
    uint8_t hop_period = 0;
    vemac->device->driver->set(vemac->device, NETOPT_CHANNEL_HOP_PERIOD, &hop_period, sizeof(uint8_t));
    vemac->device->driver->set(vemac->device, NETOPT_CHANNEL_HOP, &disable, sizeof(disable));
    vemac->device->driver->set(vemac->device, NETOPT_SINGLE_RECEIVE, &disable, sizeof(disable));
    vemac->device->driver->set(vemac->device, NETOPT_INTEGRITY_CHECK, &disable, sizeof(enable));
    vemac->device->driver->set(vemac->device, NETOPT_FIXED_HEADER, &disable, sizeof(disable));
    vemac->device->driver->set(vemac->device, NETOPT_IQ_INVERT, &disable, sizeof(disable));
    
    int16_t power = 7;
    vemac->device->driver->set(vemac->device, NETOPT_TX_POWER, &power, sizeof(int16_t));
    
    uint16_t preamble_len = 8;
    vemac->device->driver->set(vemac->device, NETOPT_PREAMBLE_LENGTH, &preamble_len, sizeof(uint16_t));
    
    uint32_t tx_timeout = 30000;
    vemac->device->driver->set(vemac->device, NETOPT_TX_TIMEOUT, &tx_timeout, sizeof(uint32_t));
    
    uint32_t rx_timeout = 0;
    vemac->device->driver->set(vemac->device, NETOPT_RX_TIMEOUT, &rx_timeout, sizeof(uint32_t));

    uint32_t frequency = 869000000;
    vemac->device->driver->set(vemac->device, NETOPT_CHANNEL_FREQUENCY, &frequency, sizeof(uint32_t));
    
    state = NETOPT_STATE_IDLE;
    vemac->device->driver->set(vemac->device, NETOPT_STATE, &state, sizeof(uint8_t));
    
    gpio_init_int(pps_pin, GPIO_IN_PU, GPIO_RISING, pps_handler, (void*) vemac);
    
    vemac->time_slot = -1;
    for (int i = 0; i < VEMAC_SLOTS; ++i) {
        vemac->slots[i] = 255;
        vemac->slots_received[i] = false;
    }
    
    vemac->fcnt = 0;
    vemac->scnt = 0;

    vemac->received = false;
    vemac->transmitted = false;
    
    vemac->slots_listened=0;
    
    vemac->id = VEMAC_ID;

    return 0;
}

int main(void){
    sx127x.params = sx127x_params;
    
    vemac_init(&vemac, (netdev_t*) &sx127x, UNWD_GPIO_1);
    
    return 0;
}
