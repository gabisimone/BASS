/*
 * mef.c



 *     
 */



typedef enum {
	SPEC,
	SPEC_CONFIG,
	FIR,
	FIR_CONFIG,
	NORM,
} state;

void f_norm(void);
void f_spec_config(void);
void f_fir_config(void);
void f_spec (void);
void f_fir(void);
void MEF_reinicio (void);
void (*MEF[])(void) = { f_fir_config,f_spec_config, f_norm, f_spec, f_fir };

state estado;
state estado_ant;
unsigned char entrada;


void Init_MEF(void) {
		
	MEF_reinicio();
	estado = SPEC;
}

void UpdateMEF(unsigned char tecla) {
		
	//TODO: arreglar para recibir tecla
	(*MEF[estado])();
}

void f_norm(void) {
	
	if (entrada == '0'){
		
		estado = SPEC;
		estado_ant = NORM;
	
	}
}

void f_spec_config(void){
//TODO: llamar funciones ajustarEntradas, variables leídas vía bluetooth
}

void f_fir_config(void){
//TODO: llamar funciones referentes al ajuste de los filtros, vía bluetooth
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
	
	estado=SPEC;
	estado_ant=SPEC;
	UpdateMEF();
}

