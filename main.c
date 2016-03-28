#include <stdio.h>
#include <string.h>
#include <stdlib.h> //for malloc and calloc
#include <string.h>
#include "contiki.h"
#include "net/rime/rime.h"
#include "net/rime/mesh.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "dev/i2cmaster.h"  // Include IC driver
#include "dev/tmp102.h"     // Include sensor driver
#include "dev/cc2420/cc2420.h"          
#include "contiki-conf.h"
#include "net/linkaddr.h"
#include "inttypes.h"
  

/** SYSTEM VARIABLES **/
static struct mesh_conn mesh;

/** PROGRAM VARIABLES **/  
static const int TIMEOUT = 5;       // Number of counter ticks before re-trasmition -> number of listHeader structures needed to maintain the list of un-ACKed packets
static uint8_t mode = 0;            // 0=not initialized, 1=sender, 2=relayer, 3=receiver; global variable because the recv() method needs access
static struct listHeader *headers;  // The pointer to the main data structure.
 

/** PROGRAM DATA STRUCTURES **/
  /** Strcuture description **
   *  listHeader[0] ---->  listItem -> listItem -> listItem -> listItem -> listItem...
   *  listHeader[1] ---->  listItem -> listItem -> listItem -> listItem -> listItem...
   *  ...
   *  listHeader[TIMEOUT] ---->  listItem...
   */  
struct listItem {
  int16_t value;
  struct listItem *next;
};

struct listHeader {
  uint16_t id;
  int size;
  struct listItem *first;
  struct listItem *last;
};


/*-- TEMPERATURE EVENT LISTENER ---------------------------------------------*/
PROCESS(temperature, "Temperature event listener");
AUTOSTART_PROCESSES(&temperature);
/*-----------------1----------------------------------------------------------*/


/*---------------------------------------------------------------------------*/
/*                            SYSTEM CALLED FUNCTIONS                        */
/*---------------------------------------------------------------------------*/

static void sent(struct mesh_conn *c)
{
  printf("packet sent\n");
}

static void timedout(struct mesh_conn *c)
{
  printf("packet timedout\n");
}

static void recv(struct mesh_conn *c, const linkaddr_t *from, uint8_t hops)
{
  int length = packetbuf_datalen();
  printf("Data received from %d.%d: %.*s (%d)\n", from->u8[0], from->u8[1], packetbuf_datalen(), (char *)packetbuf_dataptr(), packetbuf_datalen());
  char *copy = malloc(length);
  int num = packetbuf_copyto(copy); //this copying is probably not needed and the data could be addressed directly
  printf("Copied from buffer: %d\n", num);
  //uint16_t id = ((uint16_t *)packetbuf_dataptr())[0];
  uint16_t id = (copy[1] << 8) | copy[0];
    
  /* If receive packet while in mode 1, means it is an ack package. */
  if (mode == 1){
    printf("ack received: %d\n", id);
    emptyListById(id);
  }
  
  /* If receive packet while in mode 3, means this is new data.
   *  - Get the data out of the packet and make it readable
   *  - Send an ack package with the id of the pacckage
   */
  if (mode == 3){
    linkaddr_t target;
    packetbuf_copyfrom(&id, sizeof(id));
    target.u8[0] = 1;
    target.u8[1] = 0;
    mesh_send(&mesh, &target);
    
    printf("#DATA RECEIVED\n|   id: %d\n", id);

    // Go through the package data and retrieve data.
    int i;
    for (i = 2; i < length - 1; i = i +2){
      // Define local var for extraction usage
      int16_t  tempint;
      uint16_t tempfrac;
      uint16_t absraw;
      int16_t  sign;
      char     minus = ' ';
      int16_t  raw = (copy[i+1] << 8) | copy[i];
      printf("|   received[%d]: %d\n", i, raw);
      
      
      // Extract raw data
      absraw = raw;
      
      // Checking if value below 0
      if (raw < 0) { // Perform 2C's if sensor returned negative data
        absraw = (raw ^ 0xFFFF) + 1;
        sign = -1;
      }
      
      // Get value
      tempint  = (absraw >> 8) * sign;
      tempfrac = ((absraw>>4) % 16) * 625; // Info in 1/10000 of degree
      minus = ((tempint == 0) & (sign == -1)) ? '-'  : ' ' ;
      
      // Output data
      printf ("Temperature[%c]: %c%d.%04d\n", minus, tempint, tempfrac);
      
      free(copy); 
    }// for end

  }// Mode 3 end
  
  leds_toggle(LEDS_RED);
  
}//recv end

/** SYSTEM DATA STRUCTURES **/
const static struct mesh_callbacks callbacks = {recv, sent, timedout};

/*---------------------------------------------------------------------------*/
/*                              MAIN PROCESS                                 */
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(temperature, ev, data)
{
  
  // MAIN VARIABLE DECLARATION
  static uint16_t counter = 1;
  static struct etimer et;

  // PROCESS & SENSOR INIT
  PROCESS_EXITHANDLER(mesh_close(&mesh);) 
  PROCESS_BEGIN();
  SENSORS_ACTIVATE(button_sensor);
  tmp102_init();
  mesh_open(&mesh, 132, &callbacks);
  leds_blink();

  
  // MAIN LOOP
  while (1){
    
    // THE SENDER TIMER EVENT --------------------------------
    if (mode == 1 && etimer_expired(&et)){
      if (counter == 0){
        //Once every 18 or so hours the 16bit counter will overflow and will be reset to 0 -> in that case reinitialize
        //the lists so that they dont contain old values (some unACKed packets might get lost)
        freeLists();
        initListHeaders();
      }

      etimer_reset(&et);
      int16_t raw = tmp102_read_temp_raw();  // Reading from the sensor
      
      send(raw, counter);
      printf("measured value: %d ; counter: %d\n", raw, counter);
      
      counter++;
    }

    
    // THE BUTTON EVENT --------------------------------
    if(ev == sensors_event && data == &button_sensor) {
      mode++; //dont just loop over values 1,2,3 but let it increment indefinitely as per assignment instructions
      
      // User feedback when changing mode
      leds_toggle(LEDS_GREEN);
      printf("######### MODE: %d\n", mode);
      
      // Specific mode changes
      switch (mode){
        case 1: //switching TO sender mode
          initListHeaders(); //init the lists
          etimer_set(&et, CLOCK_SECOND); //prepare the timer
          counter = 1; //reset the counter to 1
          break;
        case 2: //switching FROM sender mode TO relay mode
          freeLists(); //free the list memory
          etimer_stop(&et); //stop the counter
          break;
      }

      // Change mote address
      linkaddr_t addr;
      addr.u8[0] = mode;
      addr.u8[1] = 0;
      uint16_t shortaddr = (addr.u8[0] << 8) + addr.u8[1];
      cc2420_set_pan_addr(0xABCD, shortaddr, NULL);
      linkaddr_set_node_addr(&addr);
    }
    
    PROCESS_WAIT_EVENT();
  }// MAIN LOOP end
  
  leds_off(LEDS_ALL);
  PROCESS_END();
  
}// PROCESS_THREAD end 




/*---------------------------------------------------------------------------*/
/*                            PROGRAM FUNCTIONS                              */
/*---------------------------------------------------------------------------*/

/**
 * Send the list of unsent or un-ack values depending on the counter
 */
void send(int16_t value, uint16_t counter)
{
  int index = counter%TIMEOUT;
  printf("Working with header with index: %d\n", index);

  struct listHeader *listHeader = &headers[index]; //the listHeader contains directly the struct (not the mem. address)
  listHeader->id = counter; //set the id of the header to current counter
  addlistItemAtEnd(listHeader, value); //add the current value to the list (we dont really care if its empty or not)
  
  int16_t *values;
  int16_t valuesLength = listHeader->size + 1;
  if (listHeader->first == NULL || listHeader->size < 1) {
    printf("Head was empty");
    return NULL;
  }
  
  // define the traveling pointer
  struct listItem *next = listHeader->first;
  // assocate the poiter with memory array
  values = malloc((valuesLength)*sizeof(int16_t)); //Allocate heap memory for the return array of ints

  // Save head id to the first position
  values[0] = listHeader->id;

  // go through all the listItem values
  int j;
  for (j = 1; j <= listHeader->size; j++){
    // copy item values to the array to be send
    values[j] = next->value;
    printf("value = %d\n", next->value);
    next = next->next;            
    if (next == NULL){
      break;
    }
  }

  printf("# SENDING %d \n",counter);
  printf("|   id: %d \n",counter);
  int i;
  for (i = 0; i < valuesLength; i++){
    printf("|   data[%d]: %d\n",i, values[i]);
  }
  
  linkaddr_t target;
  int copied = packetbuf_copyfrom(values, valuesLength*sizeof(int16_t));
  printf("NUMBER COPIED: %d\n", copied);
  target.u8[0] = 3;
  target.u8[1] = 0;
  mesh_send(&mesh, &target);
  
  leds_toggle(LEDS_BLUE);
}


/**
 *  Initiate the headers of our data structure
 */
void initListHeaders()
{
  //allocate heap memory for number of listHeader objects. From now on headers[0] point to the first listHeader struct and others respectively
  headers = malloc(TIMEOUT * sizeof(struct listHeader));
  if (headers == NULL){ //if memory was not allocated -> exit
    exit(1);
    }
  
  int i;
  for (i = 0; i< TIMEOUT ;i++){ // initate all the headers
    headers[i].id = -1; //set the id to -1 indicating that the "slot" is "empty"
    headers[i].size = 0;
    headers[i].first = NULL;
    headers[i].last = NULL;
  }
  printf("#INFO: Initialized headers\n");
}

/**
 *  Free all the listItem from all ListHeaders 
 */
void freeLists()
{
  int i;
  for (i = 0; i<TIMEOUT; i++){ 
    emptyList(&headers[i]); //empty the list under the header
    free(&headers[i]); //free the header memory
  }
  printf("#INFO: Freed all lists\n");
}

/**
 *  Appends the given value to the end of the list "under" the given head.
 */
void addlistItemAtEnd(struct listHeader *head, int16_t val)
{
  // TODO make the size of the list limited, to that it doesnt grow indefinitely -> dont append it, but put it as the first item
  struct listItem *newlistItem = malloc(sizeof(struct listItem)); // create a new list item pointer
    if (newlistItem == NULL){ //if memory was not allocated -> exit
      exit(1);
    }
  newlistItem->value = val; //Assign the value to the new element
  newlistItem->next = NULL; //When appending to the end no element is next

  struct listItem *temp = head->last;
  //if (temp == NULL ){ //if last item is null = the list was empty -> make the new item first AND last
  if (head->size == 0){ 
    head->first = newlistItem;
    head->last = newlistItem;
    printf("Added first item %d to the start of the list\n", val);
  } else {
    temp->next = newlistItem;
    head->last = newlistItem;
    printf("Added item %d to the end of a list\n", val);
  }
  head->size = head->size + 1;
  printf("The size of the list with id: %d, is now: %d\n", head->id, head->size);
}

/**
 * Empties the list with the given id. Can't simply use modulo since the ACK might be received much later and some values might get lost.
 */
void emptyListById(uint16_t id)
{
  int i;
  for (i=0; i < TIMEOUT; i++){
    if (headers[i].id == id) { //if found the list whose current id is the right one
      emptyList(&headers[i]); //use the emptyList method and send the memory addres of the listHeader as a parameter
      printf("Empties list with id: %d\n", id);
      break;
    }
  }
}

/**
 * Empties the list stored "under" the given head (frees the memory)
 * param head: The listHeader to empty
 * return: void
 */
void emptyList(struct listHeader *head)
{
  struct listItem *walker;
  struct listItem *deletor;
  
  // Check if data structure is need
  if (head->size == 0 || head->first == NULL || head->last == NULL) {
    printf("Failed to empty list (it could have been already empty)\n");
    return;
  }
  
  deletor = head->first;
  walker = deletor->next;
  
  // traverse the list freeing the memory
  while (walker != NULL){ 
    walker = deletor->next;
    free(deletor);
    deletor = walker;
  }  
  free(walker);
  head->first = NULL;
  head->last = NULL;
  head->size = 0; //set the size to 0 so we know the "slot" is empty without having to traverse it
  leds_toggle(LEDS_GREEN);
  printf("Emptied a list\n");
}

/**
 * getValues generate a int16_t array and copy there all the header values.
 * param head: pointer to the head of the data structure
 * param returnData: the pointer where to set the data
 * return: void
 */
/*
int16_t getValues(struct listHeader *head, int16_t *returnData)
{
  // Check head validity
  if (head->first == NULL || head->size < 1) {
    printf("Head was empty");
    return NULL;
  }
  
  // define data value ptr traveler
  struct listItem *next = head->first;
  // assocate the poiter with memory array
  returnData = malloc((head->size + 1)*sizeof(int16_t)); //Allocate heap memory for the return array of ints

  // Save head id
  returnData[0] = head->id;

  // go through all the listItem values
  int counter;
  for (counter = 1; counter <= head->size; counter++){
    // copy item values to the array to be returned
    returnData[counter] = next->value;
    next = next->next;            
    if (next == NULL){
      break;
    }
    printf("value = %d\n", next->value);
  }
  
  return head->size;
}
*/ 