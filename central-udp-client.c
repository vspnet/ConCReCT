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

extern uip_ds6_nbr_t uip_ds6_nbr_cache[];
extern uip_ds6_route_t uip_ds6_routing_table[];

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

#define PERIODMat 180
#define TMN 1  // interacoes com o no // trafego
#define aviso 2	  // avisos sobre passar dos limites
#define penalidade 3
#define avisoM 3	// limite maximo de advertencias
#define lim 3	//maximo de hellos no Perido
//#define START_INTERVAL		(15 * CLOCK_SECOND)
//#define SEND_INTERVAL		(PERIOD * CLOCK_SECOND)
//#define SEND_TIME		(random_rand() % (SEND_INTERVAL))
#define MAX_PAYLOAD_LEN		30
int mute = 0; // variavel para ativar o mute
static struct uip_udp_conn *client_conn;

static struct uip_udp_conn *server_conn;
static uip_ipaddr_t server_ipaddr;
int PERIODA = 1; //atacando
int PERIODN = 60; //normal
int PERIODM = 240; //mutado
int atq=0;
int filhosMeu = 0;
int nexthops[10];
int Lfilhos [max][2];


#define L 20		// 20 nos no maximo na matriz =  20 filhos
#define C 4		//colunas
int MatP [L][C];
int LisP [L][L];
/*---------------------------------------------------------------------------*/
PROCESS(udp_client_process, "UDP client process");
AUTOSTART_PROCESSES(&udp_client_process);
extern uip_ds6_route_t uip_ds6_routing_table[UIP_DS6_ROUTE_NB];

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
   // printf("SISTEMA C: MATRIZ DO NO SINK\n");
    for(i=0; i<L; i++)
    {
        if(Mat[i][0]==0)
            return;
        printf("\tSISTEMA C: MATRIZ: No: %d, ",Mat[i][0]);
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
    res = 0.295+2.94*f-0.056*aux3+0.0006*aux2;
    //res = (f) *3;
    //res = pow(2,2);

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
            //printf("SISTEMA C: No %d foi adicionado!\n", id);
	    filhosMeu ++;
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
                    //printf("SISTEMA C: No %d foi penalizado! \t Advertencias: %d TMN: %d\n", id,Mat[i][aviso] ,Mat[i][TMN]);
                    Mat[i][aviso] = 0;
                    Mat[i][TMN] = 0;
                    Mat[i][penalidade] = 1;
                   // imprimeM(Mat);
                    return 1; // se excedeu o limite
                }
                else 	//se nao apenas notifica
                {
                    //printf("SISTEMA C: No %d foi notificado!  \t Advertencias: %d TMN: %d\n", id,Mat[i][aviso] ,Mat[i][TMN]);
                    Mat[i][TMN] = 0;

                    return 0;
                }
            }
            else  //Se esta tudo OK com o TEN
            {
                if(Mat[i][penalidade] != 1) //Se nao esta mutado
                {
                    //printf("SISTEMA C: No %d Esta ok! \t Advertencias: %d TMN: %d\n", id,Mat[i][aviso] ,Mat[i][TMN]);
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
    char *str; 
    int cont = 0;
    int f = filhosMeu;
    char aux[2];
    aux[0] = ' ';
    aux[1] = ' ';
    char buf[MAX_PAYLOAD_LEN];
    if(uip_newdata())
    {
        //printf("mat\n");
	//imprimeM(MatP);
        str = uip_appdata;
        str[uip_datalen()] = '\0';
       // PRINTF("DATA: recv '%s' from %d\n", str, UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1]);
        if (strcmp(str,"wait")==0 && mute==0) 	//eu defini essa palavra chave
        {
            PERIODN = PERIODM;	// eu defini o tempo tbm!
            printf("SISTEMA C: Estou mutado!\n");  // ao receber o wait o nó tem seu tempo de reposta prolongado!!!
            mute = 1;
            return;
        }
        if(strcmp( str,"awake")==0) 		//verifica se o nó ja acordou
        {
            DesmutaNo(MatP,UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1]);
		return;
        }
        aux[0] =  str[0];
        aux[1] =  str[1];
        f=atoi(aux);    //transforma o texto recebido em numero (nfihlos)

        //cont = consultaM(MatP,UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1], f); //verifica os filhos

//#if SERVER_REPLY
        if(cont == 5)  	// se o no esta ok envia apenas ok
        {
            
            PRINTF(" ");
            //sprintf(buf, "wait");   // na frente vai o numero de filhos!
            //uip_udp_packet_sendto(server_conn, buf, strlen(buf), &UIP_IP_BUF->srcipaddr, UIP_HTONS(UDP_CLIENT_PORT));
        }										// é aqui que a matriz vai ser adicionada para fazer o controle

        									// é aqui que a matriz vai ser adicionada para fazer o controle

        cont = 0; //nao é filho
    
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
    if(dag->preferred_parent != NULL)    // pai
    {
        //PRINTF("NO: Pai: ");
        //PRINT6ADDR(&dag->preferred_parent->addr);
	//PRINTF("%d", dag->preferred_parent->addr.u8[sizeof(dag->preferred_parent->addr.u8) - 1]);
       // PRINTF("\n");
    }
   // PRINTF("NO: Vizinhos:\n"); // n filhos
    for(i = 0; i < UIP_DS6_NBR_NB; i++)
    {
        if(uip_ds6_nbr_cache[i].isused)
        {
            //PRINT6ADDR(&uip_ds6_nbr_cache[i].ipaddr);
	   // PRINTF("%d", uip_ds6_nbr_cache[i].ipaddr.u8[sizeof(uip_ds6_nbr_cache[i].ipaddr.u8) - 1]);
            x++; // contador dos filhos
            //PRINTF("\n");
        }
    }
    //PRINTF("---\n");
    return x;
}


/*---------------------------------------------------------------------------*/
static void
send_packet(void *ptr)
{
    int cont = 0;
    int f = filhosMeu;
    char aux[2];
    rpl_dag_t *dag;
    dag = rpl_get_any_dag();
    aux[0] = ' ';
    aux[1] = ' ';
    int i = 0;
    static int seq_id;
    char buf[MAX_PAYLOAD_LEN];
    seq_id++;
    f=filhosMeu;
    PRINTF("DATA: send to %d '%d:Nf - Hello %d'\n", server_ipaddr.u8[sizeof(server_ipaddr.u8) - 1], f, seq_id);
    
  //  cont_trafego(UIP_IP_BUF->srcipaddr.u8[sizeof(UIP_IP_BUF->srcipaddr.u8) - 1]); //controle do trafego dos seus vizinhos

 
    
    sprintf(buf, "%d:Nf - Hello %d from:",  f, seq_id);   // na frente vai o numero de filhos!

    uip_udp_packet_sendto(client_conn, buf, strlen(buf), &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT));
    uip_udp_packet_sendto(server_conn, buf, strlen(buf), &dag->preferred_parent->addr, UIP_HTONS(UDP_CLIENT_PORT));


            //uip_ipaddr_copy(&server_conn->ripaddr, &UIP_IP_BUF->srcipaddr);
            //uip_udp_packet_send(server_conn, "LOL", sizeof("LOL"));		//eu defini a palavra chave aqui // pode ser até o tempo de espera!!!
            //uip_create_unspecified(&server_conn->ripaddr);

    cont = 0;

}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
    int i;
    uint8_t state;


    
    for(i = 0; i < UIP_DS6_ADDR_NB; i++)
    {
        state = uip_ds6_if.addr_list[i].state;
        if(uip_ds6_if.addr_list[i].isused &&
                (state == ADDR_TENTATIVE || state == ADDR_PREFERRED))
        {
           // PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
           // PRINTF("\n");
            /* hack to make address "final" */
            if (state == ADDR_TENTATIVE)
            {
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
    static struct etimer periodicMAT;
    static struct ctimer backoff_timer;
#if WITH_COMPOWER
    static int print = 0;
#endif

    PROCESS_BEGIN();
    SENSORS_ACTIVATE(button_sensor);
    PROCESS_PAUSE();
    set_global_address();
    print_local_addresses();
    
   
    udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));

	
    /* new connection with remote host */
    client_conn = udp_new(NULL, UIP_HTONS(UDP_SERVER_PORT), NULL);
    server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
    if(client_conn == NULL)
    {

        PROCESS_EXIT();
    }
    udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));
    udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));

    //udp_bind(client_conn, UIP_HTONS(UDP_CLIENT_PORT));
    
   // server_conn = udp_new(NULL, UIP_HTONS(UDP_CLIENT_PORT), NULL);
    
  // udp_bind(server_conn, UIP_HTONS(UDP_SERVER_PORT));
	

    //PRINT6ADDR(&client_conn->ripaddr);
    
criaM(MatP);
#if WITH_COMPOWER
    powertrace_sniff(POWERTRACE_ON);
#endif
    etimer_set(&periodic, PERIODN * CLOCK_SECOND);
    etimer_set(&periodicMAT, PERIODMat * CLOCK_SECOND);
char *str;

    rpl_dag_t *dag;
    dag = rpl_get_any_dag();
    while(1)
    {
        PROCESS_YIELD();
	
        if(ev == tcpip_event)
        {
		
            if(mute == 0)
            {
		
                tcpip_handler();
            }
            if(mute == 1)  //caso receba o wait ele é mutado
            {
                tcpip_handler();
                PERIODN=PERIODM;
                etimer_reset(&periodic);
                etimer_set(&periodic, PERIODN * CLOCK_SECOND);
                PERIODN = PERIOD;
                mute =2; //para a funçao nao ser chamada novamente até o proximo mute

            }

        }
	if(etimer_expired(&periodicMAT))  	// aqui o tempo expira e o buffer é enviado!
        {
		limpaM(MatP);
                //printf("SISTEMA C: Avisos Zerados!\n");
                etimer_reset(&periodicMAT);
	}
        if(etimer_expired(&periodic))  	// aqui o tempo expira e o buffer é enviado!
        {
            etimer_reset(&periodic);
            etimer_set(&periodic, PERIODN * CLOCK_SECOND);
            
            if(mute==2)  // quando ele acordar novamente, envia o awake
            {
		printf("SISTEMA C: Acordei!\n");
		uip_udp_packet_sendto(server_conn, "awake", 5, &dag->preferred_parent->addr, UIP_HTONS(UDP_CLIENT_PORT));
                //uip_udp_packet_sendto(client_conn, "awake", 5, &server_ipaddr, UIP_HTONS(UDP_SERVER_PORT)); //Avisa o SISTEMA C que acordou
		
                
            }
            ctimer_set(&backoff_timer, (random_rand() % (PERIODN * CLOCK_SECOND)), send_packet, NULL);  // nesta etapa calcula-se o novo tempo de envio, com o PERIODN atualizado!!!
            mute = 0; // ao fim do periodo de espera o mute é retirado

#if WITH_COMPOWER
            if (print == 0)
            {
                powertrace_print("#P");

            }
            if (++print == 3)
            {
                print = 0;
            }
#endif
        }
	
        else if (ev == sensors_event && data == &button_sensor)   // ataque inicado pelo botao!!
        {
            PRINTF("SISTEMA C: Iniciando Ataque\n");
            PERIODN=PERIODA;
            etimer_set(&periodic, PERIODA * CLOCK_SECOND);
		//imprimeM(MatP);


        }

    }

    PROCESS_END();
}
/*---------------------------------------------------------------------------*/
