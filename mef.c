/*
 * mef.c

A implementar cuando tengamos el modulo bluetooth andando, las entradas se van a leer por ahi para actualizar la mef
e ir pasando entre estados

 *     
 */



typedef enum {
	SPEC,
	FIR,
	NORM,
} state;

void f_norm(void);
void f_spec (void);
void f_fir(void);
void MEF_reinicio (void);
void (*MEF[])(void) = { f_norm, f_spec, f_fir };

state estado;
state estado_ant;
unsigned char entrada;


void Init_MEF(void) {
		
	MEF_reinicio();
	estado = SPEC;
}

void UpdateMEF(unsigned char tecla) {
		
	//entrada = tecla; para cuando hagamos andar el modulo bluetooth
	(*MEF[estado])();
}

void f_norm(void) {
	
	if (entrada == '0'){
		
		estado = SPEC;
		estado_ant = NORM;
	
	}
}


void f_spec (){
	

	if (entrada == '#'){

		estado = NORM;
		estado_ant = SPEC;
	}
	else{
		if (entrada == '*'){
			//mostrarR=0;
			MEF_reinicio();
			estado = FIR;
			estado_ant = SPEC;
		}
	}
	
}

void f_fir(){
	
	if (entrada == '#'){

			//	mostrarR=1;
			//	mostrarT=0;
			//	mostrarV=0;

				estado = NORM;
				estado_ant = FIR;
			}
			else{
				if (entrada == '*'){
					
					//mostrarR=0;

					MEF_reinicio();
					estado = SPEC;
					estado_ant = FIR;
				}
			}
}



void MEF_reinicio(){
	
	
}

