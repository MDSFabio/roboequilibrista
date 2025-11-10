const int OUTA = 3;
const int OUTB = 9;
const int OUTC = 10;
const int OUTD = 11;

void init_motores(){
  pinMode(OUTA, OUTPUT);
  pinMode(OUTB, OUTPUT);
  pinMode(OUTC, OUTPUT);
  pinMode(OUTD, OUTPUT);
  
}

void PMWControleMotores(double comando){
  if(comando > 0){
    //ANDAR PARA FRENTE
    analogWrite(OUTA, abs(comando));//CONTROLANDO MOTOR DA ESQUERDA PARA FRENTE
    analogWrite(OUTB, 0);//CONTROLANDO MOTOR  DA ESQUERDA PARA TRAS
    analogWrite(OUTC, abs(comando));//CONTROLANDO MOTOR  DA DIREITA PARA FRENMTE
    analogWrite(OUTD, 0);//CONTROLANDO MOTOR  DA DIREITA PARA TRAS
  }else{
    //ANDAR PARA TRÁS
    analogWrite(OUTA, 0);//CONTROLANDO MOTOR DA ESQUERDA PARA FRENTE
    analogWrite(OUTB, abs(comando));//CONTROLANDO MOTOR  DA ESQUERDA PARA TRAS
    analogWrite(OUTC, 0);//CONTROLANDO MOTOR  DA DIREITA PARA FRENMTE
    analogWrite(OUTD, abs(comando));//CONTROLANDO MOTOR  DA DIREITA PARA TRAS
      
  }
}


