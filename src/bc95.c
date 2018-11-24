#include "bc95.h"
#include "main.h"
extern rx_buf[MAX_STRBUF_SIZE];
char strx;
char extstrx;


void ClearRxBuffer(void)
{
    unsigned char i;

    for(i = 0; i < MAX_STRBUF_SIZE; i++) {
        rx_buf[i] = 0;
    }
}
// TODO:set the maximum delay for while() to break
void BC95_Init(void) {
    TransmitString("AT\r\n");
    strx=strstr((const char*)RxBuffer,(const char*)"OK");// Return OK
    while(strx==NULL)
        {
            Clear_Buffer();
            //TransmitString("AT\r\n");
            printf("AT\r\n");
            DelayClock(300);
            strx=strstr((const char*)rx_buf,(const char*)"OK");
        }
    printf("AT+CMEE=1\r\n"); //Acquire the error code
    DelayClock(300);
    strx=strstr((const char*)RxBuffer,(const char*)"OK");
    Clear_Buffer();

    printf("AT+NBAND?\r\n");// Acquire the frequency
    DelayClock(300);
    strx=strstr((const char*)RxBuffer,(const char*)"+NBAND:5");
    Clear_Buffer();

    while(strx==NULL)
    {
            Clear_Buffer();
            printf("AT+NBAND?\r\n");// Acquire the frequency
            DelayClock(300);
            strx=strstr((const char*)RxBuffer,(const char*)"+NBAND:5");
    }

    printf("AT+CIMI\r\n");// Acquire CIMI
    DelayClock(300);
    strx=strstr((const char*)RxBuffer,(const char*)"46011");//¡¤Return 46011
    Clear_Buffer();
    while(strx==NULL)
    {
            Clear_Buffer();
            printf("AT+CIMI\r\n");// Acquire CIMI
            DelayClock(300);
            strx=strstr((const char*)RxBuffer,(const char*)"46011");// Return 46011
    }

    printf("AT+CGATT=1\r\n");// Confirm the card is available or not
    DelayClock(300);
    strx=strstr((const char*)RxBuffer,(const char*)"OK");//¡¤Return OK
    Clear_Buffer();
    while(strx==NULL)
    {
            Clear_Buffer();
            printf("AT+CGATT=1\r\n");
            DelayClock(300);
            strx=strstr((const char*)RxBuffer,(const char*)"OK");
    }
    printf("AT+CGATT?\r\n");//Activiate the network
    DelayClock(300);
    strx=strstr((const char*)RxBuffer,(const char*)"+CGATT:1");// Return 1
    Clear_Buffer();
    while(strx==NULL)
    {
            Clear_Buffer();
            printf("AT+CGATT?\r\n");
            DelayClock(300);
            strx=strstr((const char*)RxBuffer,(const char*)"+CGATT:1");
    }

    printf("AT+CSQ\r\n");//Acquire CSQ
        DelayClock(300);
        strx=strstr((const char*)RxBuffer,(const char*)"+CSQ");//¡¤Return CSQ
        if(strx)
            {
                BC95_Status.CSQ=(strx[5]-0x30)*10+(strx[6]-0x30);
                if(BC95_Status.CSQ==99)// Fail to connect
                {
                    while(1)
                    {
                        // TODO
                        // Insert the code to indicate the error during the process of initializing BC95
                        DelayClock(300);
                    }
                }
            }
            while(strx==NULL)
            {
                    Clear_Buffer();
                    printf("AT+CSQ\r\n");//
                    DelayClock(300);
                    strx=strstr((const char*)RxBuffer,(const char*)"+CSQ");//
                if(strx)
                {
                    BC95_Status.CSQ=(strx[5]-0x30)*10+(strx[6]-0x30);//
                    if(BC95_Status.CSQ==99)//
                    {
                        while(1)
                        {
                            // TODO
                            // Insert the code to indicate the error during the process of initializing BC95
                            DelayClock(300);
                        }
                    }
                }
            }
            Clear_Buffer();

    printf("AT+CEREG?\r\n"); // Registration status
    DelayClock(300);
    strx=strstr((const char*)RxBuffer,(const char*)"+CEREG:0,1");// Return the status of registration
    extstrx=strstr((const char*)RxBuffer,(const char*)"+CEREG:1,1");
    Clear_Buffer();
    while(strx==NULL&&extstrx==NULL)
    {
        Clear_Buffer();
        printf("AT+CEREG?\r\n");
        DelayClock(300);
        strx=strstr((const char*)RxBuffer,(const char*)"+CEREG:0,1");
        extstrx=strstr((const char*)RxBuffer,(const char*)"+CEREG:1,1");
    }

    printf("AT+CEREG=1\r\n");
    DelayClock(300);
    strx=strstr((const char*)RxBuffer,(const char*)"OK");
    Clear_Buffer();
    while(strx==NULL&&extstrx==NULL)
    {
        Clear_Buffer();
        printf("AT+CEREG=1\r\n");//
        DelayClock(300);
        strx=strstr((const char*)RxBuffer,(const char*)"OK");//
    }

}
