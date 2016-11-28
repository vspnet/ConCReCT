/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

#include "contiki.h"
#include "lib/random.h"
#include "sys/ctimer.h"
#include "net/uip.h"
#include "net/uip-ds6.h"
#include "net/uip-udp-packet.h"
#include "net/neighbor-info.h"
#include "net/rpl/rpl.h"

#if CONTIKI_TARGET_Z1
#include "dev/uart0.h"
#else
#include "dev/uart1.h"
#endif
//#include "net/neighbor-info.h"
//#include <uip-ds6-route.h>



#ifdef WITH_COMPOWER
#include "powertrace.h"
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "dev/button-sensor.h"
#define UDP_CLIENT_PORT 8765
#define UDP_SERVER_PORT 5678

#define UDP_EXAMPLE_ID  190

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"
#define max 20 	//numero maximo de filhos
#define min 5  //a cada 5 mensagems o no filho deve comunicarse ao menos uma vez se nao sai da lista
#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#ifndef PERIOD
#define PERIOD 60
#endif
//#define START_INTERVAL		(15 * CLOCK_SECOND)
//#define SEND_INTERVAL		(PERIOD * CLOCK_SECOND)
//#define SEND_TIME		(random_rand() % (SEND_INTERVAL))
#define MAX_PAYLOAD_LEN		30
int mute = 0; // variavel para ativar o mute
static struct uip_udp_conn *client_conn;
static uip_ipaddr_t server_ipaddr;
int PERIODA = 1; //atacando
int PERIODN = 60; //normal
int PERIODM = 240; //mutado
int atq=0;
int f = 0;
int nexthops[10];
int Lfilhos [max][2];
/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process);
extern uip_ds6_route_t uip_ds6_routing_table[UIP_DS6_ROUTE_NB];
  
/*---------------------------------------------------------------------------*/
void criaL(void){  //cria a lista de filhos
 	int i = 0;
	for(i=0;i<max;i++){
		Lfilhos[i][0] = 0;
		Lfilhos[i][1] = 0;
	}
	return ;

}

static void
tcpip_handler(void)
{
  char *str;
  if(uip_newdata()) {
    str = uip_appdata;
    str[uip_datalen()] = '\0';
    PRINTF("DATA: recv '%s'\n", str);
	if (strcmp(str,"wait")==0 && mute==0){	//eu defini essa palavra chave
		PERIODN = PERIODM;	// eu defini o tempo tbm!
		printf("SISTEMA: Estou mutado!!!!!\n");  // ao receber o wait o nó tem seu tempo de reposta prolongado!!!
		mute = 1;
	}
  }
}

int
collect_common_net_print(void)  // funçao da tabela de filhos
{
  rpl_dag_t *dag;
  int i;
	int x = 0;
  /* Let's suppose we have only one instance */
  dag = rpl_get_any_dag();
  if(dag->preferred_parent != NULL) {  // pai 
    PRINTF("NO: Pai: ");
    PRINT6ADDR(&dag->preferred_parent->addr);
    PRINTF("\n");
  }
  PRINTF("NO: Filhos:\n"); // n filhos
  for(i = 0; i < UIP_DS6_ROUTE_NB; i++) {
    if(uip_ds6_routing_table[i].isused) {
      PRINT6ADDR(&uip_ds6_routing_table[i].ipaddr);
      x++; // contador dos filhos
      PRINTF("\n");
    }
  }
  PRINTF("---\n");
 return x;
}

/*---------------------------------------------------------------------------*/
static void
send_packet(void *ptr)
{
	
  static int seq_id;
  char buf[MAX_PAYLOAD_LEN];
  seq_id++;
  f=collect_common_net_print();
  PRINTF("DATA: send to %d 'Hello %d' from: %d\n", server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1], seq_id, UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1]);
  sprintf(buf, "%d:Nf - Hello %d from:",  f, seq_id);   // na frente vai o numero de filhos!
  uip_udp_packet_sendto(client_conn, buf, strlen(buf), &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;
  
	
  PRINTF("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      PRINTF("\n");
      /* hack to make address "final" */
      if (state == ADDR_TENTATIVE) {
	uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
      }
    }
  }
    
}
/*---------------------------------------------------------------------------*/
static void
set_global_address(void)
{
  uip_ipaddr_t ipaddr;

  uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
  uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
  uip_ds6_addr_add(&ipaddr, 0, ADDR_AUTOCONF);

/* The choice of server address determines its 6LoPAN header compression.
 * (Our address will be compressed Mode 3 since it is derived from our link-local address)
 * Obviously the choice made here must also be selected in udp-server.c.
 *
 * For correct Wireshark decoding using a sniffer, add the /64 prefix to the 6LowPAN protocol preferences,
 * e.g. set Context 0 to aaaa::.  At present Wireshark copies Context/128 and then overwrites it.
 * (Setting Context 0 to aaaa::1111:2222:3333:4444 will report a 16 bit compressed address of aaaa::1111:22ff:fe33:xxxx)
 *
 * Note the IPCMV6 checksum verification depends on the correct uncompressed addresses.
 */
 
#if 0
/* Mode 1 - 64 bits inline */
   uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
#elif 1
/* Mode 2 - 16 bits inline */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0, 0x00ff, 0xfe00, 1);
#else
/* Mode 3 - derived from server link-local (MAC) address */
  uip_ip6addr(&server_ipaddr, 0xaaaa, 0, 0, 0, 0x0250, 0xc2ff, 0xfea8, 0xcd1a); //redbee-econotag
#endif
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_client_process, ev, data)
{
  static struct etimer periodic;
  static struct ctimer backoff_timer;
#if WITH_COMPOWER
  static int print = 0;
#endif

  PROCESS_BEGIN();
  SENSORS_ACTIVATE(button_sensor);
  PROCESS_PAUSE();

  set_global_address();
  criaL();
  PRINTF("UDP client process started\n");

  print_local_addresses();

  /* new connection with remote host */
  client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL); 
  if(client_conn == NULL) {
    PRINTF("No UDP connection available, exiting the process!\n");
    PROCESS_EXIT();
  }
  udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT)); 

  PRINTF("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  PRINTF(" local/remote port %u/%u\n",
	UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

#if WITH_COMPOWER
  powertrace_sniff(POWERTRACE_ON);
#endif
etimer_set(&periodic, PERIODN * CLOCK_SECOND);
  while(1) {
    PROCESS_YIELD();
	
    if(ev == tcpip_event) {	
	if(mute == 0)
	{
	 	
         tcpip_handler();
	}
	if(mute == 1){ //caso receba o wait ele é mutado
		tcpip_handler();
		PERIODN=PERIODM;
		etimer_reset(&periodic);
		etimer_set(&periodic, PERIODN * CLOCK_SECOND);
		PERIODN = PERIOD;
		mute =2; //para a funçao nao ser chamada novamente até o proximo mute

	}
	
    }
    if(etimer_expired(&periodic)) {	// aqui o tempo expira e o buffer é enviado!
      etimer_reset(&periodic);
      etimer_set(&periodic, PERIODN * CLOCK_SECOND);
	if(mute==2){ // quando ele acordar novamente, envia o awake
	      uip_udp_packet_sendto(client_conn, "awake", 5, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT)); //Avisa o Sistema que acordou
	      printf("SISTEMA: Acordei!\n");
        }
      ctimer_set(&backoff_timer, (random_rand() % (PERIODN * CLOCK_SECOND)), send_packet, NULL);  // nesta etapa calcula-se o novo tempo de envio, com o PERIODN atualizado!!!
      mute = 0; // ao fim do periodo de espera o mute é retirado
      
	#if WITH_COMPOWER
	      if (print == 0) {
		powertrace_print("#P");
	
	      }
	      if (++print == 3) {
		print = 0;
	      }
	#endif
    }
      else if (ev == sensors_event && data == &button_sensor) { // ataque inicado pelo botao!!	
         	PRINTF("SISTEMA: Iniciando Ataque\n");
		PERIODN=PERIODA;
		etimer_set(&periodic, PERIODA * CLOCK_SECOND);
		
	
      }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
