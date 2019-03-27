
// Código GPRS(GSM SIM800L) retirado do video do youtube : Brincando Com Ideias ---> https://www.youtube.com/watch?v=GbVXixOUUPM&t=813s

#include <SoftwareSerial.h> // biblioteca padrao do Arduino

SoftwareSerial serialGSM(10, 11); // RX, TX

//funcoes
#define senhaGsm "1234"
#define pinBotaoCall 12
#define numeroCall "xxxxxxxxxxx" //numero a ser definido

//variaveis
bool temSMS = false;
String telefoneSMS;
String dataHoraSMS;
String mensagemSMS;
String comandoGSM = "";
String ultimoGSM = "";
bool callStatus = false;

//funcoes
void leGSM();
void enviaSMS(String telefone, String mensagem);
void fazLigacao(String telefone);
void configuraGSM();

//inicia a comunicacao serial
void setup() {

  Serial.begin(9600);
  serialGSM.begin(9600); 

  pinMode(pinBotaoCall, INPUT_PULLUP);

  Serial.println("Sketch Iniciado!");
  configuraGSM(); //funcao para receber sms
}

void loop() {
  leGSM(); //execucao constante 

  if (comandoGSM != "") {
      Serial.println(comandoGSM);
      ultimoGSM = comandoGSM;
      comandoGSM = "";
  }

  if (temSMS) { // verdadeira se recebermos sms

     Serial.println("Chegou Mensagem!!");
     Serial.println();
    
     Serial.print("Remetente: ");  
     Serial.println(telefoneSMS);
     Serial.println();
    
     Serial.print("Data/Hora: ");  
     Serial.println(dataHoraSMS);
     Serial.println();
    
     Serial.println("Mensagem:");  
     Serial.println(mensagemSMS);
     Serial.println();
      
     mensagemSMS.trim();      
     if ( mensagemSMS == senhaGsm ) { // compara as senhas
        Serial.println("Enviando SMS de Resposta.");  
         enviaSMS(telefoneSMS, "SMS Recebido e Senha OK!"); // envia o sms de volta para o telefone que enviou a msg ,o texto colocado aqui será enviado de volta
     }
     temSMS = false;
  }

  if (!digitalRead(pinBotaoCall) && !callStatus) {
     Serial.println("Afetuando Ligacao..."); 
     fazLigacao(numeroCall);
     callStatus = true;
  }

  if (ultimoGSM.indexOf("+COLP:") > -1) {
     Serial.println("LIGACAO EM ANDAMENTO");
     ultimoGSM = "";                
  }
       
  if (ultimoGSM.indexOf("NO CARRIER") > -1) {
     Serial.println("LIGACAO TERMINADA");
     ultimoGSM = "";
     callStatus = false;
  }
       
  if (ultimoGSM.indexOf("BUSY") > -1) {
     Serial.println("LINHA/NUMERO OCUPADO");
     ultimoGSM = "";
     callStatus = false;
  }

  if (ultimoGSM.indexOf("NO DIALTONE") > -1) {
     Serial.println("SEM LINHA");
     ultimoGSM = "";
     callStatus = false;
  }
       
  if (ultimoGSM.indexOf("NO ANSWER") > -1) {
     Serial.println("NAO ATENDE");
     ultimoGSM = "";
     callStatus = false;
  }
  
}

void leGSM()
{
  static String textoRec = "";
  static unsigned long delay1 = 0;
  static int count=0;  
  static unsigned char buffer[64];

  if (serialGSM.available()) {            
 
     while(serialGSM.available()) {         
   
        buffer[count++] = serialGSM.read();     
        if(count == 64)break;
     }

     textoRec += (char*)buffer;
     delay1   = millis();
     
     for (int i=0; i<count; i++) {
         buffer[i]=NULL;
     } 
     count = 0;                       
  }


  if ( ((millis() - delay1) > 100) && textoRec != "" ) {

     if ( textoRec.substring(2,7) == "+CMT:" ) {
        temSMS = true;
     }

     if (temSMS) {
            
        telefoneSMS = "";
        dataHoraSMS = "";
        mensagemSMS = "";

        byte linha = 0;  
        byte aspas = 0;
        for (int nL=1; nL < textoRec.length(); nL++) {

            if (textoRec.charAt(nL) == '"') {
               aspas++;
               continue;
            }                        
          
            if ( (linha == 1) && (aspas == 1) ) {
               telefoneSMS += textoRec.charAt(nL);
            }

            if ( (linha == 1) && (aspas == 5) ) {
               dataHoraSMS += textoRec.charAt(nL);
            }

            if ( linha == 2 ) {
               mensagemSMS += textoRec.charAt(nL);
            }

            if (textoRec.substring(nL - 1, nL + 1) == "\r\n") {
               linha++;
            }
        }
     } else {
       comandoGSM = textoRec;
     }
     
     textoRec = "";  
  }     
}


void enviaSMS(String telefone, String mensagem) {
  serialGSM.print("AT+CMGS=\"" + telefone + "\"\n");
  serialGSM.print(mensagem + "\n");
  serialGSM.print((char)26); 
}

void fazLigacao(String telefone) {
  serialGSM.println("ATH0\n");
  serialGSM.print((char)26); 
  serialGSM.println("ATD " + telefone + ";\n");
  serialGSM.print((char)26); 
}


void configuraGSM() {
   serialGSM.print("AT+CMGF=1\n;AT+CNMI=2,2,0,0,0\n;ATX4\n;AT+COLP=1\n"); 
}
