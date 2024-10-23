// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

int alarmEnabled = FALSE;
int alarmCount = 0;

// Alarm function handler
void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

volatile int STOP = FALSE;

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        return -1;
    }

    layer = connectionParameters; //Criamos uma variável layer para guardar o valor do connectionParameters
    (void)signal(SIGALRM, alarmHandler); //Quando o sinal SIGALRM for gerado (por exemplo, após o uso de alarm()), a função alarmHandler será chamada.

    // (Semelhante ao writenoncanonical.c)

    //Abrimos a porta de série para leitura e escrita. 

    int fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);

    if (fd < 0){
        perror(connectionParameters.serialPort);
        exit(-1);
    }

    //Estruturas usadas para configurar parâmetros da porta de série.

    struct termios oldtio; //oldtio armazena a configuração atual da porta de série.
    struct termios newtio; //newtio armazena a nova configuração da porta de série.

    //Guardar as definições da porta de série.

    if(tcgetattr(fd, &oldtio) == -1){
        perror("tcgetattr");
        exit(-1);
    }

    //Limpar a struct para armazenar as novas configurações da porta de série.

    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR; //Ignora erros de paridade ao receber dados.
    newtio.c_oflag = 0; 

    newtio.c_lflag = 0; //Configura a comunicação em modo noncanonical
    newtio.c_cc[VTIME] = 0; 
    newtio.c_cc[VMIN] = 0;  //A função read só dá return depois de serem recebidos pelo menos 0 caracteres.

    tcflush(fd, TCIOFLUSH);

    //Definir novas configurações da porta de série:

    if(tcsetattr(fd, TCSANOW, &newtio) == -1){
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termio structure set\n");

    sleep(1); // O sleep espera até que todos os bytes tenham sido escritos

    STOP = FALSE; //STOP é usado como uma flag para depois terminar o loop em baixo
    int bytes = 0; //Variável para armazenar os bytes enviados ou recebidos

    if (connectionParameters.role == LlTx) //Verifica se é transmissor
    {
        while (!STOP && alarmCount < connectionParameters.nRetransmissions)
        {
            if (!alarmEnabled)
            {
                bytes = sendBuffer(A_T, C_SET); //SET para iniciar a comunicação e armazenar o nr de bytes enviados na variavel bytes
                printf("%d bytes written\n", bytes);

                alarm(connectionParameters.timeout);
                alarmEnabled = TRUE; 
                state = START;
            }
            stateMachine(A_T, C_UA, 0, 0); //A state machine aguarda a resposta UA do recetor
        }
        alarm(0); // Caso o transmissor tenha recebido a resposta UA, desativa o alarme

        if (alarmCount >= layer.nRetransmissions) //Se o nr de trasmissoes atingiu o limite (alarmCount), o transmissor nao conseguiu estabelecer ligação por isso: Erro TimeOut
        {
            printf("ERRO TIME OUT\n");
            return -1;
        }

        printf("LLOPEN OK\n");
    }

    else if (connectionParameters.role == LlRx) //Aqui vai ser o recetor a aguardar o SET do transmissor
    {
        while (!STOP) //Espera pelo comando SET para entrar no loop
        {
            stateMachine(A_T, C_SET, 0, 0); 
        }

        bytes = sendBuffer(A_T, C_UA); //Quando recetor recebe SET envia de volta a resposta UA para o transmissor para avisar que está tudo ok na transmissao
        printf("RESPONSE TO LLOPEN TRANSMITTER. %d bytes written\n", bytes);
    }

    return 0;
}


////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}
