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
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/uip.h"
#include "net/rpl/rpl.h"
#include <math.h>
#include "net/netstack.h"
#include "dev/button-sensor.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define DEBUG DEBUG_PRINT
#include "net/uip-debug.h"

#define UIP_IP_BUF   ((struct uip_ip_hdr *)&uip_buf[UIP_LLH_LEN])

#define UDP_CLIENT_PORT	8765
#define UDP_SERVER_PORT	5678

#define L 20		// 20 nos no maximo na matriz =  20 filhos
#define C 4		//colunas
#define UDP_EXAMPLE_ID  190
#define TMN 1  // interassoes com o no // trafego
#define aviso 2	  // avisos sobre passar dos limites
#define penalidade 3
#define avisoM 3	// limite maximo de advertencias
#define lim 3	//maximo de hellos no Perido
#ifndef PERIOD
#define PERIOD 180 //tempo para resetar as advertencias
#endif
#define START_INTERVAL		(15 * CLOCK_SECOND)
#define SEND_INTERVAL		(PERIOD * CLOCK_SECOND)
#define SEND_TIME		(random_rand() % (SEND_INTERVAL))

static struct uip_udp_conn *server_conn;
extern uip_ds6_route_t uip_ds6_routing_table[UIP_DS6_ROUTE_NB];
int MatP [L][C];
int LisP [L][L];
int controle = 0;
int timer = 0;
extern uip_ds6_nbr_t uip_ds6_nbr_cache[];
extern uip_ds6_route_t uip_ds6_routing_table[];

//---------------------------------------------------------------------------------------

PROCESS(udp_server_process, "UDP server process");
AUTOSTART_PROCESSES(&udp_server_process);
/*---------------------------------------------------------------------------*/

//--------------------------matriz de clientes


void criaM(int Mat[L][C])  // funcao para inicializao da matriz
{
    int i = 0, j = 0;
    for(i=0; i<L; i++)
        for(j=0; j<C; j++)
            Mat[i][j] = 0;
    return ;

}

void limpaM(int Mat[L][C])  //   funcao para limpar os avisos e TMN dos nos apos o periodo de tempo acabar
{
    int i = 0;
    for(i=0; i<L; i++)
    {
        Mat[i][aviso] = 0;
        Mat[i][TMN] = 0;

    }
    return ;

}

void imprimeM (int Mat[L][C])   //impressao da matriz de filhos
{
    int i=0;
    printf("SISTEMA S: MATRIZ DO NO SINK\n");
    for(i=0; i<L; i++)
    {
        if(Mat[i][0]==0)
            return;
        printf("\tSISTEMA S: MATRIZ: No: %d, ",Mat[i][0]);
        if(Mat[i][penalidade]==1)
            printf("Situacao: Suspeito de Ataque - Mutado!\n");
        else if(Mat[i][0] == 0)
            printf("Situacao: Nao alocado!\n");
        else
            printf("Situacao: Ativo!\n");

    }
}

void DesmutaNo(int Mat[L][C], int id)  // retira a variavel de mute do no
{
    int i = 0;
    for(i=0; i<L; i++)
    {
        if(Mat[i][0] == id) 	// caso o nó não esteja alocado ainda na matriz (primeira comunicação com o sink)
        {
            Mat[i][penalidade] = 0;
        }
    }
}
float pot(int a, float b)  //operaçao de potencia
{
    int i = 0;
    double res = 1;
    for (i=0; i<b; i++)
    {
        res = res*a;
    }
    return res;
}


float Limitetrafego(int f)  //formula utilizada !
{
    float res = 0;
    float aux1 = 0, aux2 = 0, aux3 = 0;
    aux1 = pot(f,4);
    aux2 = pot(f,3);
    aux3 = pot(f,2);
    //res = (-0.00034 * aux1) + (0.0053 * aux2) - (0.2 * aux3) + 3.87;
    //res = (f) *3;
    res = 0.295+2.94*f-0.056*aux3+0.0006*aux2;


    return res;
}

int consultaM(int Mat[L][C], int id, int f)
{
    int i = 0;
    float TEN =0;

    TEN = Limitetrafego(f+1);
    for(i=0; i<L; i++)
    {
        if(Mat[i][0] == 0) 	// caso o nó não esteja alocado ainda na matriz (primeira comunicação com o sink)
        {
            Mat[i][0] = id;
            Mat[i][1] = 1;
            //printf("SISTEMA S: No %d foi adicionado!\n", id);
            return 0;
        }
        else if(Mat[i][0] == id)  //encoutrou a posiçao do no
        {
            Mat[i][TMN] ++;

            if(Mat[i][TMN]>TEN)
            {
                Mat[i][aviso] ++;
                if(Mat[i][aviso] > avisoM)  // se os numero de avisos foram ultrapassados
                {
                    printf("SISTEMA S: No %d foi penalizado! \t Advertencias: %d TMN: %d\n", id,Mat[i][aviso] ,Mat[i][TMN]);
                    Mat[i][aviso] = 0;
                    Mat[i][TMN] = 0;
                    Mat[i][penalidade] = 1;
                   // imprimeM(Mat);
                    return 1; // se excedeu o limite
                }
                else 	//se nao apenas notifica
                {
                    //printf("SISTEMA S: No %d foi notificado!  \t Advertencias: %d TMN: %d\n", id,Mat[i][aviso] ,Mat[i][TMN]);
                    Mat[i][TMN] = 0;

                    return 0;
                }
            }
            else  //Se esta tudo OK com o TEN
            {
                if(Mat[i][penalidade] != 1) //Se nao esta mutado
                {
                    //printf("SISTEMA S: No %d Esta ok! \t Advertencias: %d TMN: %d\n", id,Mat[i][aviso] ,Mat[i][TMN]);
                    Mat[i][penalidade] = 0;
                }
                return 0; // se nao excedeu o limite
            }
        }

    }
    return 0;

}



//-------------------------------------------------------------------------------------



static void
tcpip_handler(void)
{
    char *appdata;
    int cont = 0;
    int f = 1;
    char aux[2];
    aux[0] = ' ';
    aux[1] = ' ';
    int i = 0;
    rpl_dag_t *dag;

    int x = 0;
    /* Let's suppose we have only one instance */
    dag = rpl_get_any_dag();
    if(uip_newdata())
    {
        appdata = (char *)uip_appdata;
        appdata[uip_datalen()] = 0;
        if(strcmp(appdata,"awake")==0) 		//verifica se o nó ja acordou
        {
            DesmutaNo(MatP,UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1]);
        }
        PRINTF("DATA: recv '%s' from %d\n", appdata, UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1]);
        //PRINTF("\n");
        aux[0] = appdata[0];
        aux[1] = appdata[1];
        f=atoi(aux);    //transforma o texto recebido em numero (nfihlos)
	
	for(i = 0; i < UIP_DS6_NBR_NB; i++)
    	{
//		PRINTF("%d == %d\n", uip_ds6_nbr_cache[i].ipaddr.u8[sizeof(uip_ds6_nbr_cache[i].ipaddr.u8) - 1], UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1]);
       	 	if(uip_ds6_nbr_cache[i].ipaddr.u8[sizeof(uip_ds6_nbr_cache[i].ipaddr.u8) - 1] == UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1])
       	 	{
				  
       	     		 cont = consultaM(MatP,UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1], f); //verifica os filhos
        	}
    	}
       

//#if SERVER_REPLY
        if(cont == 0)  	// se o no esta ok envia apenas ok
        {
            //PRINTF("DATA: sending reply");
            //PRINTF("\n");
           // uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
           // uip_udp_packet_send(server_conn, "ok", sizeof("ok"));		//eu defini a palavra chave aqui // pode ser até o tempo de espera!!!
            //uip_create_unspecified(&server_conn->ripaddr);			// falta agora escrever a politica te tempo - isso vai virar uma condição!
        }										// é aqui que a matriz vai ser adicionada para fazer o controle

        else  	// se o no estourou o limite - envia wait
        {
            PRINTF("DATA: sending reply WAIT\n");
            //PRINTF("\n");
            uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
            uip_udp_packet_send(server_conn, "wait", sizeof("wait"));		//eu defini a palavra chave aqui // pode ser até o tempo de espera!!!
            uip_create_unspecified(&server_conn->ripaddr);			// falta agora escrever a politica te tempo - isso vai virar uma condição!
        }										// é aqui que a matriz vai ser adicionada para fazer o controle

        cont = 0;
    }
}


/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
    int i;
    uint8_t state;

    PRINTF("Server IPv6 addresses: ");
    for(i = 0; i < UIP_DS6_ADDR_NB; i++)
    {
        state = uip_ds6_if.addr_list[i].state;
        if(state == ADDR_TENTATIVE || state == ADDR_PREFERRED)
        {
            PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
            PRINTF("\n");
            /* hack to make address "final" */
            if (state == ADDR_TENTATIVE)
            {
                uip_ds6_if.addr_list[i].state = ADDR_PREFERRED;
            }
        }
    }
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(udp_server_process, ev, data)
{
    uip_ipaddr_t ipaddr;
    struct uip_ds6_addr *root_if;

    PROCESS_BEGIN();

    PROCESS_PAUSE();

    SENSORS_ACTIVATE(button_sensor);

    PRINTF("UDP server started\n");

    if(controle == 0)
    {
        criaM(MatP);

    }
    controle = 1;

#if UIP_CONF_ROUTER
    /* The choice of server address determines its 6LoPAN header compression.
     * Obviously the choice made here must also be selected in udp-client.c.
     *
     * For correct Wireshark decoding using a sniffer, add the /64 prefix to the 6LowPAN protocol preferences,
     * e.g. set Context 0 to aaaa::.  At present Wireshark copies Context/128 and then overwrites it.
     * (Setting Context 0 to aaaa::1111:2222:3333:4444 will report a 16 bit compressed address of aaaa::1111:22ff:fe33:xxxx)
     * Note Wireshark's IPCMV6 checksum verification depends on the correct uncompressed addresses.
     */

#if 0
    /* Mode 1 - 64 bits inline */
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 1);
#elif 1
    /* Mode 2 - 16 bits inline */
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0x00ff, 0xfe00, 1);
#else
    /* Mode 3 - derived from link local (MAC) address */
    uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
    uip_ds6_set_addr_iid(&ipaddr, &uip_lladdr);
#endif

    uip_ds6_addr_add(&ipaddr, 0, ADDR_MANUAL);
    root_if = uip_ds6_addr_lookup(&ipaddr);
    if(root_if != NULL)
    {
        rpl_dag_t *dag;
        dag = rpl_set_root(RPL_DEFAULT_INSTANCE,(uip_ip6addr_t *)&ipaddr);
        uip_ip6addr(&ipaddr, 0xaaaa, 0, 0, 0, 0, 0, 0, 0);
        rpl_set_prefix(dag, &ipaddr, 64);
        PRINTF("created a new RPL dag\n");
    }
    else
    {
        PRINTF("failed to create a new RPL DAG\n");
    }
#endif /* UIP_CONF_ROUTER */

    print_local_addresses();

    /* The data sink runs with a 100% duty cycle in order to ensure high
       packet reception rates. */
    NETSTACK_MAC.off(1);

    server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);


    
    if(server_conn == NULL)
    {
        PRINTF("No UDP connection available, exiting the process!\n");
        PROCESS_EXIT();
    }
    udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

    PRINTF("Created a server connection with remote address ");
    PRINT6ADDR(&server_conn->ripaddr);
    PRINTF(" local/remote port %u/%u\n", UIP_HTONS(server_conn->lport),
           UIP_HTONS(server_conn->rport));
    criaM(MatP);

    static struct etimer periodic;

    etimer_set(&periodic, PERIOD * CLOCK_SECOND);
    while(1)
    {
        PROCESS_YIELD();
        if(ev == tcpip_event)
        {
            if(etimer_expired(&periodic))
            {
                limpaM(MatP);
                //printf("SISTEMA S: Avisos Zerados!!!!\n");
                etimer_reset(&periodic);
            }
            tcpip_handler();

        }
        else if (ev == sensors_event && data == &button_sensor)     //operaçao feita ao usar o click botton
        {
            imprimeM(MatP);

        }
    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/

