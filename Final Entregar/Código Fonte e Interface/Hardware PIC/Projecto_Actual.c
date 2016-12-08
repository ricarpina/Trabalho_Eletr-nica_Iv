/*
 * MPLAB X IDE v1.60 + XC32 v1.11
 *
 * Target: DETPIC32 board
 * Configurations:
 *      HSPLL; CPUCLK=80MHz, PBCLCK=40MHz; Watchdog timer disabled
 *      USART: 115200,8,N,1
 */
#define PIC_XTAL 20MHz //É necessária por delay.h
#define PASSO 50 //Passo na amostragem inicial (obter gráfico tensão vs posição)
#define dim 7 //Dimensão do vetor onde sefaz a correspondencia tensão-posição
#include <stdio.h> // Definer o EOF utilizado pelo gechar
#include <stdlib.h>
#include <p32xxxx.h>
#include <plib.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

#define SYSCLK 80000000L // System clock frequency, in Hz
#define PBUSCLK 40000000L // Peripheral bus clock


// Defines
#define TPS_256 7 // TCKPS code for xx pre-scaler
#define TPS_64  6
#define TPS_32  5
#define TPS_16  4
#define TPS_8   3
#define TPS_4   2
#define TPS_2   1
#define TPS_1   0

//variaveis globais
double valE,valD,arrai[dim]={};
double umax=0;
double T[2]={};
double declive=1465; //valor mínima

// Function replacements to redirect stdin/stdout to USART1
// These functions are called by printf(), scanf(), ...
void _mon_putc(char c) {
    while (U1STAbits.UTXBF); // Wait till buffer available (TX Buffer Full)
    U1TXREG = c; // Put char in Tx buffer
    return;
}

int _mon_getc(int canblock) {

    // Reset Overrun Eror Flag - if set UART does not receive any chars
    if (U1STAbits.OERR)
        U1STAbits.OERR;

    if (canblock == 0) {
        if (U1STAbits.URXDA)
            return (int) U1RXREG;
    }
    else {
        while (!U1STAbits.URXDA);
        return (int) U1RXREG;
    }
    return -1;
}

void adcconfig(){
    
    // Initialize ADC module
    // Polling mode, AN0 as input
    // Generic part
    AD1CON1bits.SSRC = 7; // Internal counter ends sampling and starts
    //conversion
    AD1CON1bits.CLRASAM = 1; //Stop conversion when 1st A/D converter interrupt
    //is generated and clears ASAM bit automatically
    AD1CON1bits.FORM = 0; // Integer 16 bit output format
    AD1CON2bits.VCFG = 0; // VR+=AVdd; VR-=AVss
    AD1CON2bits.SMPI = 0; // Number (+1) of consecutive conversions, stored in
    //ADC1BUF0...ADCBUF{SMPI}
    AD1CON3bits.ADRC = 1; // ADC uses internal RC clock
    AD1CON3bits.SAMC = 16; // Sample time is 16TAD ( TAD = 100ns)

    TRISBbits.TRISB0 = 1; // AN0 in input mode
    TRISBbits.TRISB1 = 1; // AN1 in input mode

    AD1PCFGbits.PCFG0 = 0; // AN0 as analog input
    AD1PCFGbits.PCFG1 = 0; // AN1 as analog input
   }
//
double READ_ADC(int canal)
{
    double media=0;
    int i;
     AD1CHSbits.CH0SA = canal; // para 1 é a leitura direita

    // Enable module
    AD1CON1bits.ON = 1; // Enable A/D module (This must be the ***last
    //instruction of configuration phase***)

    for (i = 0; i < 10; i++){
  
    IFS1bits.AD1IF = 0; // Reset interrupt flag
    AD1CON1bits.ASAM = 1; // Start conversion Será necessário?
    while (IFS1bits.AD1IF == 0); // Wait fo EOC
    // Convert to 0..3.3V
    media = media + (ADC1BUF0 * 3.3) / 1023;
    }
    AD1CON1bits.ON = 0; // Desabilita módulo A/D. Como é a última instrução da
    //configuração A/D deve-se desabilitar pois o canal de leitura pode mudar
    //a qualquer instante
return media / 10;
}

void pwmconfig(int T, int Ton){

    long pr_value;
// Set timer
    T2CONbits.ON = 0; // Stop timer
    IFS0bits.T2IF=0; // Reset interrupt flag
    T2CONbits.TCKPS = TPS_256; //Select pre-scaler
    T2CONbits.T32 = 0; // 16 bit timer operation
    pr_value=T/(256/(float)(PBUSCLK/1000000)); // Compute PR value
    PR2 = pr_value;
    TMR2=0;

    // Set OC1
    OC1CONbits.OCM = 6; // OCM = 0b110 : OC1 in PWM mode,
    OC1CONbits.OCTSEL=0; // Timer 2 is clock source of OCM
    OC1RS=Ton/(256/(float)(PBUSCLK/1000000)); // Compute OC1xRS value
    OC1CONbits.ON=1;     // Enable OC1

    // Start PWM generation
    T2CONbits.TON=1; // Start the timer

}

void PID_timer(int tempo){

    long pr_value;
     // Set timer
    //period_us=200000/2; // Toggles every half-period - period in microseconds
    pr_value = tempo/(256/(float)(PBUSCLK/1000000)); // Compute PR value
    T4CONbits.ON = 0; // Stop timer
    T4CONbits.TCKPS = TPS_256; //Select pre-scaler
    T4CONbits.T32 = 0; // 16 bit timer operation
    PR4=pr_value;
    TMR4=0;
    T4CONbits.TON=1; // Start the timer
}

int pos_calc(){
    int j=0;
     //sensor esquerda do eixo negativo. à direita do eixo positivo
    double adc, m,b;
        valE=READ_ADC(0);
        valD=READ_ADC(1);

        adc=valD-valE;
        if (adc<arrai[0]){
            return 0;
        }else if(adc>arrai[dim-1]){
            return 300;
        }else{
    while((arrai[j]>=adc)||(arrai[j+1]<=adc)&&j<(dim-1)){
        j++;
    }
}

     m=(arrai[j]-arrai[j-1])/(PASSO); //50 é o passo na calha
     b=arrai[j-1]-(m*(j-1)*PASSO);
     int pos =(int) ((adc-b)/m);

     if (pos >300){
         printf("Colocar bola entre as posições -150mm e 150mm\n");
         return 300;
     }else if(pos <0){
         printf("Colocar bola entre as posições -150mm e 150mm\n");
         return 0;
     }
     return pos;
}



int main(int argc, char** argv) {
      // Performance optimization (flash access time, enable instruct and data
    //cache,... and PBClock setup
    SYSTEMConfigPerformance(SYSCLK);
    mOSCSetPBDIV(OSC_PB_DIV_2); // This is necessary since
    //SYSTEMConfigPerformance defaults FPBDIV to DIV_1

    // Remove input/output buffering
    setbuf(stdin, NULL); //no input buffer (for scanf)
    setbuf(stdout, NULL); //no output buffer (for printf)

    
  adcconfig();

    short int opcao=1, k=0;
    int posd=0, posa=0; //posd é a posição destino posa é a posição a alcançar
   int ii=0;
   short int i=0;
  char str[5], c='0';
 

    //Variâveis do Controlo PID------------------------------------------------
     double ek=0,erros=0,pre_ek=0; //Erros
     double kp=4;//ganho proporcional
     double Ti=100000000; //Constante integrativa
     double Ts=10000; //Timer associado ao PID , em us
     double Td=800000; //Constante derivativa
    // double ki=0.9;
     double u=0, u_ant=0; //Resultado do PID

     double ki=kp*Ts/Ti;
     double kd=kp*Td/Ts;
  
     double P=0,D=0,I=0;
     
     
    
     //Cálculos d Controlo PID
//-----------------------------------------------------------------------------
        printf("SET"); //Sinal para informar PC que o PIC foi reiniciado
             //Antes de definir a posição de destino, traça-se o gráfico
   
   PID_timer(Ts); //Ajustar tempo
    while (1) {
while(IFS0bits.T4IF==0);
        IFS0bits.T4IF=0;

        int result=1;
        int resultp=1;
        int resultd=1;

        switch(opcao){
            case 1:
             str[0]=_mon_getc(0);
            result = strncmp(&str[0], "+", 1);
            if (result==0){
            valE=READ_ADC(0);
            valD=READ_ADC(1);
            arrai[ii]=valD-valE;
            ii++;
            
            if (ii==dim){ // 7 posicoes. Quando alcança 7 as posições de 0
                //a 6 estão preenchidas
                opcao=2;
           
            }
            }
            break;
            case 2:
        str[0]=_mon_getc(0);
        str[1]=_mon_getc(0);
        str[2]=_mon_getc(0);
        str[3]=_mon_getc(0);
                str[4]=_mon_getc(0);

                result = strncmp(&str[0], "p", 1);
            resultp = strncmp(&str[0], "P", 1);
            resultd = strncmp(&str[0], "D", 1);

             char st[4];

             if(result==0 || resultp==0 || resultd==0){
        st[0]=str[1];
        st[1]=str[2];
        st[2]=str[3];
        st[3]=str[4];
             }

             if (result==0){
                posd=atoi(st);
             }else if(resultp==0){
                 kp=(double)atoi(st);
             }else if(resultd==0){
                 Td=(double)atoi(st);
             }
             posa=pos_calc()-150;
          printf("Posicao estimada: %d\n", posa);
                        
    ek=((double)posd-(double)posa);
    P = kp * ek;
    D= (kd*(ek-pre_ek));
    erros +=ek; //erros=erros+ek[0];
    pre_ek=ek;
    I=ki*erros;
    u=P+D+I;
     u=(u/10);
     u=(int)u;
    if (u>75){
      u=75;
    }
    if (u<-75){
      u=-75;
        }
     if(u!=u_ant)
    pwmconfig(10159,1465-(int)u);

     u_ant=u;
      printf("u: %f\n", u);

      break;
        }

    }
  return(EXIT_SUCCESS);

}
