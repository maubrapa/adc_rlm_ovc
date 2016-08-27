/* ********************************************************************
 		UNISINOS - Universidade do Vale do Rio dos Sinos   
 Sistema de Apoio ao Motorista baseados em Visao Computacional
 Coordenador - Cláudio Rosito Jung
 Bolsista CNPq - Leandro Lorenzett Dihl

 Projeto CNPQ - Processo 506350/2004-5			Maio/2005
**********************************************************************/

/*##########################  Includes  #############################*/

#include "CLane.h"
#include "opencv2/opencv.hpp"
#include "ostream"

/********************************************************************
CONSTRUTORES
********************************************************************/

CLane::CLane()
{
	this->largMin = 15;
	this->largMax = 315;
	this->altMin = 120;
	this->altMax = 215;
	
	this->altTotal = altMax - altMin;
	this->largTotal = largMax - largMin;

	matrix_Main = Matrix<float>(this->largMax - this->largMin, this->altMax - this->altMin);//Matriz principal para armazenar os dados da imagem
	matrixWeight = Matrix<float>(largMax - largMin,altMax - altMin);
	matrix_Weight = Matrix<float>(this->largMax - this->largMin,this->altMax - this->altMin);//Cópia da Matriz principal para armazenar os 
	matrix_Orientation = Matrix<float>(this->largMax - this->largMin,this->altMax - this->altMin);
	matrix_TmpX = Matrix<float>(this->largMax - this->largMin,this->altMax - this->altMin);
	matrix_TmpY = Matrix<float>(this->largMax - this->largMin,this->altMax - this->altMin);

	compMax = 15;	//recebe o comprimento máximo utilizado no lbroi;
	compMin = 5;	//recebe o comprimento mínimo utilizado no lbroi;
	iXm = 40;       //recebe o valor do XM
	iXv = 0;
	constXM = LINESKY;
	acummXV =0;
	matrixWeight = Matrix<float>(largMax - largMin,altMax - altMin);	//define o tamanho da matriz de pesos
	contFrame = 15;
	trainingFrames = 15;
	this->iBoundary = 0;
	this->iMiddlef = 0;
	iControllerOD = 0; //variavel que controla a detecção da obstaculos
//	float fMi = 0.0f; //varia´veis para a comparação dos valores q podem ser possiveis obstáculos
//	float fTi = 0.0f;

	pesos[0] = 0.0765f; pesos[1] = 0.1334f; pesos[2] = 0.1861f;
    pesos[3] = 0.2080f; pesos[4] = 0.1861f; pesos[5] = 0.1334f;
	pesos[6] = 0.0765f; pesos[7] = 0.1271f;
	
	discreteGaussianWeights[0] = 0.2075f; discreteGaussianWeights[1] = 0.2062f;
	discreteGaussianWeights[2] = 0.2024f; discreteGaussianWeights[3] = 0.1962f;
	discreteGaussianWeights[4] = 0.1878f;

	tamMatrixAuxEsq = altTotal-LINESKY; //Define o tamanho da matriz auxiliar
	tamMatrixAuxDir = altTotal-LINESKY;
	tamMatrixMain = altTotal-LINESKY;
	matrixMainEsq = new MatrixMain[tamMatrixMain];
	matrixMainDir = new MatrixMain[tamMatrixMain];
	stateSystem = INITIAL_STATE;
	int x,y, x1 = this->largMax-this->largMin, y1 = this->altMax-this->altMin;
	for (y=0;y<y1;y++){
		for (x=0;x<x1;x++){
			matrix_Main[x][y] = 0.0f;
			matrix_Weight[x][y] = 0.0f;
			matrix_Orientation[x][y] = 0.0f;
			matrix_TmpX[x][y] = 0.0f;
			matrix_TmpY[x][y] = 0.0f;
		}
	}
}

CLane::CLane(int lgMin, int lgMax, int alMin, int alMax, int xm,int xv, int training, float cpMax, float cpMin)
{	
	this->largMin = lgMin;	//recebe a largura mínima que será informado pelo usuário na interface
	this->largMax = lgMax;	//recebe a largura máxima que será informado pelo usuário na interface
	this->altMin = alMin;	//recebe a altura mínima que será informado pelo usuário na interface
	this->altMax = alMax;	//recebe a altura máxima que será informado pelo usuário na interface		
	this->altTotal = altMax - altMin;
	this->largTotal = largMax - largMin;

	this->compMax = cpMax;	//recebe o comprimento máximo utilizado no lbroi;
	this->compMin = cpMin;	//recebe o comprimento mínimo utilizado no lbroi;
	this->iXm = xm;			//recebe o valor do XM
	this->iXv = 0;
	this->acummXV = 0;
	this->constXM = LINESKY;
	this->contFrame = training;
	this->trainingFrames = training;
	this->quadro = 0;
	this->iBoundary = 0;
	this->iMiddlef = 0;
	iControllerOD = 0; //variavel que controla a detecção da obstaculos
//	float fMi = 0.0f; //varia´veis para a comparação dos valores q podem ser possiveis obstáculos
//	float fTi = 0.0f;
	
	matrix_Main = Matrix<float>(this->largMax - this->largMin, this->altMax - this->altMin);//Matriz principal para armazenar os dados da imagem
	matrixWeight = Matrix<float>(largMax - largMin,altMax - altMin);
	matrix_Weight = Matrix<float>(this->largMax - this->largMin,this->altMax - this->altMin);//Cópia da Matriz principal para armazenar os 
	matrix_Orientation = Matrix<float>(this->largMax - this->largMin,this->altMax - this->altMin);
	matrix_TmpX = Matrix<float>(this->largMax - this->largMin,this->altMax - this->altMin);
	matrix_TmpY = Matrix<float>(this->largMax - this->largMin,this->altMax - this->altMin);

	tamMatrixAuxEsq = altTotal-LINESKY; //Define o tamanho da matriz auxiliar
	tamMatrixAuxDir = altTotal-LINESKY;
	tamMatrixMain = altTotal-LINESKY;
	matrixMainEsq = new MatrixMain[tamMatrixMain];
	matrixMainDir = new MatrixMain[tamMatrixMain];
	stateSystem = INITIAL_STATE;
	
	pesos[0] = 0.0765f; pesos[1] = 0.1334f; pesos[2] = 0.1861f;
    pesos[3] = 0.2080f; pesos[4] = 0.1861f; pesos[5] = 0.1334f;
	pesos[6] = 0.0765f; pesos[7] = 0.1271f;

	discreteGaussianWeights[0] = 0.2075f; discreteGaussianWeights[1] = 0.2062f;
	discreteGaussianWeights[2] = 0.2024f; discreteGaussianWeights[3] = 0.1962f;
	discreteGaussianWeights[4] = 0.1878f;

	int x,y, x1 = this->largMax-this->largMin, y1 = this->altMax-this->altMin;
	for (y=0;y<y1;y++){
		for (x=0;x<x1;x++){
			matrix_Main[x][y] = 0.0f;
			matrix_Weight[x][y] = 0.0f;
			matrix_Orientation[x][y] = 0.0f;
			matrix_TmpX[x][y] = 0.0f;
			matrix_TmpY[x][y] = 0.0f;
		}
	}
};

/*********************************************************************
Retorna o estado do sistema
*********************************************************************/
/*CString CLane::getStateAlgorithm()
{
	return stateAlgorithm;
}*/


/*####################################FUNÇÕES GERAIS##################################
	
	Empregadas nos dois processos principais do sistema: a detecção inicial 
	e a detecção no prosseguimento
	Todas elas iniciam com gd_ : geral detection.

######################################FUNÇÕES GERAIS##################################*/

IplImage* CLane::gd_Run(IplImage* imagem)
{
float xx;
	IplImage* imagemAtual = NULL;
	id_fill_MatrixMain(imagem);
	switch (stateSystem) {
    case INITIAL_STATE:
        stateSystem = TRAINING_STATE;
        imagemAtual = this->id_initialLane();
        break;
    case TRAINING_STATE:
		if(contFrame > 0){//se no completou o estipulado no treinamento
			contFrame--;	//controla a qtde de Frames para o calculo do XV
			imagemAtual = this->fd_followingIndependent();
		}
		else { //completou o treinamento
			if (contFrame == 0){
				this->fd_divXV(); //realiza a diviso do somatorio dos XV
				contFrame--;
			}
			stateSystem = ON_ROAD_STATE;
			imagemAtual = fd_followingDependent();//realiza no modo dependente
		}
        break;
    case ON_ROAD_STATE:
        imagemAtual = fd_followingDependent();//realiza no modo dependente
		if(fd_lane_Departure_Detection() != 0){
			//stateAlgorithm = "OFF ROAD STATE";
			stateSystem = OFF_ROAD_STATE;
//            cout << "stateSystem = OFF_ROAD_STATE" << endl;
		}
        break;
    case OFF_ROAD_STATE:
        imagemAtual = fd_followingDependent();//realiza no modo dependente
		xx = fd_standard_Deviation();
		if( xx < 0.06f){
			if(((this->fd_get_Angles()) > 90) ||((this->fd_get_Angles()) < -90)) {
				stateSystem = SWITCH_ROAD_STATE;
			}
			else{
				stateSystem = ON_ROAD_STATE;
			}	
			stateSystem = SWITCH_ROAD_STATE;
		}
        break;
    case SWITCH_ROAD_STATE:
        iBoundary = fd_lane_Departure_Detection();
		fd_get_New_Boundary();
		stateSystem = ON_ROAD_STATE;
		imagemAtual = fd_followingDependent();//realiza no modo dependente
        break;
	case ERROR_STATE:
        iBoundary = fd_lane_Departure_Detection();
		fd_get_New_Boundary();
        break;
	}
	quadro++;
	return imagemAtual;
}

void CLane::setLargMin(int lgMin){
	this->largMin = lgMin;
}
void CLane::setLargMax(int lgMax){
	this->largMax = lgMax;
}
void CLane::setAltMin(int alMin){
	this->altMin = alMin;
}
void CLane::setAltMax(int alMax){
	this->altMax = alMax;
}
void CLane::setCompMax(float cpMax){
	this->compMax = cpMax;
}
void CLane::setCompMin(float cpMin){
	this->compMin = cpMin;
}
void CLane::setXM(int xm){
	this->iXm = xm;			
}
float CLane::getXV(){
	return (float)this->iXv;			
}

int CLane::getXM() {
    return (int) this->iXm;
}

void CLane::imprimeXM(FILE* file){
	if(vFilteredLateralOffSet.size()>1)
		fprintf(file,"\n%i %f %f %f %i %f %i",quadro,lateralOffset,vLateralOffSet[vLateralOffSet.size()-1],vFilteredLateralOffSet[vFilteredLateralOffSet.size()-1],fd_lane_Departure_Detection(),fd_standard_Deviation(),fd_get_Angles());
}
/*********************************************************************
	Função para resolver o modelo dos limites da estrada, determinando os
valores para preencher os vetores auxiliares da esquerda e da direita 
*********************************************************************/

void CLane::gd_solveEquationLBM(float matrixABD[3], MatrixMain matrixAux[])
{
	for(int x = 0; x < tamMatrixMain; x++)
	{
		matrixAux[x].posI= 0.0f;
		matrixAux[x].posJ= 0.0f; 
	}
	float a=matrixABD[0], b=matrixABD[1], d=matrixABD[2], tempf;
	for(int i = 0; i < tamMatrixMain; i++)
	{		
		matrixAux[i].posI = (float)(i+LINESKY);
		if(matrixAux[i].posI > constXM)
		{
			tempf = a + b * matrixAux[i].posI;
			matrixAux[i].posJ = (float)tempf;
		}
		else
		{
			tempf = ((2*a+constXM*(b-d))/2+d*matrixAux[i].posI+((b-d)/(2*constXM))*(matrixAux[i].posI*matrixAux[i].posI));
			matrixAux[i].posJ = (float)tempf;
		}
	}
}

/*********************************************************************
Preenche a matrix principal inicialmente com os novos valores da imagem
*********************************************************************/
void CLane::gd_paintImage()
{
	for (int y=0;y < tamMatrixMain  ;y++){
		if (((int)matrixMainDir[y].posJ < largMax - largMin-2) && ((int)matrixMainDir[y].posJ > 2)){
			
			//printf("| %d %d %d %d %d\n", y, matrixMainDir[y].posI, matrixMainDir[y].posJ, matrixMainEsq[y].posI, matrixMainEsq[y].posJ );
			/*cout << hex
				<< "| " << y
				<< " " << matrixMainDir[y].posI
				<< " " << matrixMainDir[y].posJ
				<< " " << matrixMainEsq[y].posI
				<< " " << matrixMainEsq[y].posJ << "\n";*/
			for (int l=matrixMainEsq[y].posJ;l<matrixMainDir[y].posJ;l++){
				if ((int)matrixMainEsq[y].posJ >= largMax - largMin-2 ){

				}
				else {
					if ((int)matrixMainEsq[y].posJ <= 2) {
					}
					// DENTRO DESSE 'ELSE' ABAIXO É PINTADA A PISTA MAIS A BORDA DIREITA DELA.
					else {
						matrix_Main[l][((int)matrixMainDir[y].posI)]=0;
					}
				}
			}
			// matrix_Main[((int)matrixMainDir[y].posJ)][((int)matrixMainDir[y].posI)]=0;			
			// matrix_Main[((int)matrixMainDir[y].posJ)-1][((int)matrixMainDir[y].posI)]=0;
			// matrix_Main[((int)matrixMainDir[y].posJ)-2][((int)matrixMainDir[y].posI)]=0;	
			
		}
		/*
		else if ((int)matrixMainDir[y].posJ >= largMax - largMin-2) {
			matrix_Main[(largMax - largMin - 1)][((int)matrixMainDir[y].posI)]=0;
			matrix_Main[(largMax - largMin - 2)][((int)matrixMainDir[y].posI)]=0;
			matrix_Main[(largMax - largMin - 3)][((int)matrixMainDir[y].posI)]=0;
		}
		else if ((int)matrixMainDir[y].posJ <= 2){		
			// matrix_Main[1][1]=0;
		    matrix_Main[1][((int)matrixMainDir[y].posI)]=0;
			matrix_Main[2][((int)matrixMainDir[y].posI)]=0;
			matrix_Main[3][((int)matrixMainDir[y].posI)]=0;
		}
		*/

		
		// DENTRO DESSE 'IF' ABAIXO É PINTADA A BORDA ESQUERDA DA PISTA.
		if (((int)matrixMainEsq[y].posJ > 2) && ((int)matrixMainEsq[y].posJ < largMax - largMin-2)){		
			 matrix_Main[((int)matrixMainEsq[y].posJ)][((int)matrixMainEsq[y].posI)]=0;
			 matrix_Main[((int)matrixMainEsq[y].posJ)+1][((int)matrixMainEsq[y].posI)]=0;
			 matrix_Main[((int)matrixMainEsq[y].posJ)+2][((int)matrixMainEsq[y].posI)]=0;
		}
		
		/*
		else if ((int)matrixMainEsq[y].posJ >= largMax - largMin-2 ){
			matrix_Main[(largMax - largMin - 1)][((int)matrixMainEsq[y].posI)]=0;
			matrix_Main[(largMax - largMin - 2)][((int)matrixMainEsq[y].posI)]=0;
			matrix_Main[(largMax - largMin - 3)][((int)matrixMainEsq[y].posI)]=0;
		
		}
		else if ((int)matrixMainEsq[y].posJ <= 2){
			matrix_Main[1][((int)matrixMainEsq[y].posI)]=0;
			matrix_Main[2][((int)matrixMainEsq[y].posI)]=0;
			matrix_Main[3][((int)matrixMainEsq[y].posI)]=0;
		}
		*/
	}
	
	/*
	for (int z = 0; z < matrixObstacles.size(); z++){
		matrix_Main[matrixObstacles[z].posJ][matrixObstacles[z].posI]=0;
	} 
	*/
}

IplImage* CLane::gd_matrixToImg(Matrix<float>* mat){
	int i,j;
	for ( j=0 ; j<largTotal ; j++ ){
		for ( i=0 ; i<altTotal ; i++ ){
			image_Main->imageData[(image_Main->height-i-altMin)*(image_Main->width)+j+largMin] = mat->GetAt( j, i );
		}
	}
	return image_Main;
}


/*###########################FUNÇÕES da DETECÇÃO INICIAL##############################
	
	Empregadas no processo de detecção inicial das bordas da pista do sistema.
	Todas elas iniciam com id_ : inicial detection.

#############################FUNÇÕES da DETECÇÃO INICIAL##############################*/

IplImage* CLane::id_initialLane(){
	id_cleanVariaveis();
	id_gradient_Function();
	id_sum_Magnitudes();
	id_filter_Angles();
	id_get_Peaks();
	id_get_Pairs();
	id_applying_HT();
	id_rate_ABD(matrixABDEsq,matrixABDDir);
	this->gd_solveEquationLBM(matrixABDEsq,matrixMainEsq);
	this->gd_solveEquationLBM(matrixABDDir,matrixMainDir);
	this->gd_paintImage();
	return this->gd_matrixToImg(&matrix_Main);
}

/*Método para realizar a detecção inicial recebendo uma image da camera ou de um AVI*/
IplImage* CLane::id_initialLane(IplImage* image){
	id_cleanVariaveis();
	id_fill_MatrixMain(image);
	id_gradient_Function();
	id_sum_Magnitudes();
	id_filter_Angles();
	id_get_Peaks();
	id_get_Pairs();
	id_applying_HT();
	id_rate_ABD(matrixABDEsq,matrixABDDir);
	this->gd_solveEquationLBM(matrixABDEsq,matrixMainEsq);
	this->gd_solveEquationLBM(matrixABDDir,matrixMainDir);
	this->gd_paintImage();
	return this->gd_matrixToImg(&matrix_Main);
}

/*Preenche a matrix_Main principal inicialmente com os novos valores da imagem*/
void CLane::id_fill_MatrixMain(Matrix<float> matrix)
{	
	int x,y, x1,y1;

	for (y=0;y<altTotal;y++){
		for (x=0;x<largTotal;x++){
			x1 = x+largMin;
			y1 = y+altMin;
			matrix_Main[x][y] = matrix[x1][y1];
			matrix_Weight[x][y] = 0.0f;
			matrix_Orientation[x][y] = 0.0f;
			matrix_TmpX[x][y] = 0.0f;
			matrix_TmpY[x][y] = 0.0f;
		}
	}
}

/*LEITURA DA IMAGEM*/

void CLane::id_fill_MatrixMain(IplImage* ipl_image)
{
    assert(ipl_image);
	IplImage* gray = cvCreateImage( cvSize( ipl_image->width , ipl_image->height ), IPL_DEPTH_8U ,1);

    cvCvtColor(ipl_image, gray, CV_BGR2GRAY);
    
	if(ipl_image->origin == gray->origin) {		//version 1 of opencv flips the image?!
		cvFlip(gray, gray, 0);
	}   
    
	image_Main = gray;
	for (int y=0;y<altTotal;y++){
		for (int x=0;x<largTotal;x++){
		// FOI MUDADO ABAIXO (unsigned char)
			matrix_Main[x][y] = (unsigned char) gray->imageData[(gray->width)*(gray->height-y-altMin)+x+largMin];
		}
	}
}

void CLane::id_fill_MatrixMain()
{
	int x,y;
	for (y=0;y<this->altMax-this->altMin;y++){
		for (x=0;x<this->largMax-this->largMin;x++){
			matrix_Weight[x][y] = 0.0f;
			matrix_Orientation[x][y] = 0.0f;
			matrix_TmpX[x][y] = 0.0f;
			matrix_TmpY[x][y] = 0.0f;
		}
	}
}
void CLane::id_cleanVariaveis()
{		
		for(int y=0;y<3;y++){
			matrixABDEsq[y] = 0.0f;
			matrixABDDir[y] = 0.0f;
		}
		il_sum_Mag.clear();
		il_sum_MagEnd.clear();
		peak_Mag.clear();
		pos_Peak.clear();
		neg_Peak.clear();
		pos_Peak.resize(90);
		neg_Peak.resize(90);
		matrixAEsqWeight.clear();//vetores para armazenar os pesos
		matrixADirWeight.clear();
		rhoEsq.clear();
		rhoDir.clear();
		rhoEsqEnd.clear();
		rhoDirEnd.clear();
		histoEsq.clear();
		histoDir.clear();
}
/*********************************************************************
   Funcao para remover estruturas indesejadas devido a interferências,
   ruídos ou outras estruturas que aparecem na estrada.

   Recebe a media dos pesos e o tamanho da matriz aux e Retorna void.
*********************************************************************/
void CLane::id_gradient_Function()
{
	for (int i = 1; i < altTotal-1; i++)
	{
 		for(int j = 1; j < largTotal-1; j++) 
		{
			matrix_Weight[j][i] = id_rate_Sobel(j,i);
//			cout << "matrix_Weight[" << j << "][" <<i<< "] = " << matrix_Weight[j][i] << "\n";
		}
	}	
	id_rate_Orientation();
}
float CLane::id_rate_Sobel(int x, int y)
{
	float tmpX = 0.0f, tmpY = 0.0f;
	float tmp;
	tmpX=(float)(-matrix_Main[x-1][y-1]-2*matrix_Main[x][y-1]-matrix_Main[x+1][y-1]+
				matrix_Main[x-1][y+1]+2*matrix_Main[x][y+1]+matrix_Main[x+1][y+1]);
    tmpY=(float)(-matrix_Main[x-1][y-1]+matrix_Main[x+1][y-1]-2*matrix_Main[x-1][y]+
				2*matrix_Main[x+1][y]-matrix_Main[x-1][y+1]+matrix_Main[x+1][y+1]);

	matrix_TmpX[x][y] = tmpX;
	matrix_TmpY[x][y] = tmpY;
	return tmp =float(abs(tmpX) + abs(tmpY));
}
void CLane::id_rate_Orientation(){
	double dX = 0.0f, dY = 0.0f;
	for (int y = 1; y < altTotal-1; y++)
	{
 		for(int x = 1; x < largTotal-1; x++) 
		{
			dX = 0.0f;
			dY = 0.0f;
			dX=(double)((2*matrix_TmpX[x-1][y-1]*matrix_TmpY[x-1][y-1])+(2*matrix_TmpX[x][y-1]*matrix_TmpY[x][y-1])+(2*matrix_TmpX[x+1][y-1]*matrix_TmpY[x+1][y-1])+(2*matrix_TmpX[x-1][y]*matrix_TmpY[x-1][y])+(2*matrix_TmpX[x][y]*matrix_TmpY[x][y])+(2*matrix_TmpX[x+1][y]*matrix_TmpY[x+1][y])+
				(2*matrix_TmpX[x-1][y+1]*matrix_TmpY[x-1][y+1])+(2*matrix_TmpX[x][y+1]*matrix_TmpY[x][y+1])+(2*matrix_TmpX[x+1][y+1]*matrix_TmpY[x+1][y+1]));
			dY=(double)((matrix_TmpY[x-1][y-1]*matrix_TmpY[x-1][y-1] - matrix_TmpX[x-1][y-1]*matrix_TmpX[x-1][y-1])+(matrix_TmpY[x+1][y-1]*matrix_TmpY[x+1][y-1]-matrix_TmpX[x+1][y-1]*matrix_TmpX[x+1][y-1])+(matrix_TmpY[x-1][y]*matrix_TmpY[x-1][y]-matrix_TmpX[x-1][y]*matrix_TmpX[x-1][y])+
				(matrix_TmpY[x][y-1]*matrix_TmpY[x][y+1]-matrix_TmpX[x][y-1]*matrix_TmpX[x][y-1])+(matrix_TmpY[x][y]*matrix_TmpY[x][y]-matrix_TmpX[x][y]*matrix_TmpX[x][y])+(matrix_TmpY[x][y+1]*matrix_TmpY[x][y+1]-matrix_TmpX[x][y+1]*matrix_TmpX[x][y+1])+
				(matrix_TmpY[x+1][y]*matrix_TmpY[x+1][y]-matrix_TmpX[x+1][y]*matrix_TmpX[x+1][y])+(matrix_TmpY[x-1][y+1]*matrix_TmpY[x-1][y+1]-matrix_TmpX[x-1][y+1]*matrix_TmpX[x-1][y+1])+(matrix_TmpY[x+1][y+1]*matrix_TmpY[x+1][y+1]-matrix_TmpX[x+1][y+1]*matrix_TmpX[x+1][y+1]));
			matrix_Orientation[x][y] = ((atan(dX/dY+0.000001f))/2 * 180/PI) ;
		}
	}
}

void CLane::id_sum_Magnitudes(){
	int tamanhoVetor = 180/SUBINTERVAL, indice = 0; //armazena o tamanho q serah o vetor das Magnitudes
	il_sum_Mag.resize(tamanhoVetor);				//redimensiona o vetor
	
	for(unsigned int x = 0; x < il_sum_Mag.size(); x++){		//
		il_sum_Mag[x] = 0.0f;
	}
	for (int i = 1; i < altTotal-1; i++)		//
	{
 		for(int j = 1; j < largTotal-1; j++) 
		{
			if((matrix_Orientation[j][i] >=-90) && (matrix_Orientation[j][i] <= 90)){
				indice = 0;
				indice = int((matrix_Orientation[j][i]+90)/SUBINTERVAL);
				float temp = 0.0f;
				temp = il_sum_Mag[indice] + matrix_Weight[j][i];
				il_sum_Mag[indice] = temp;
			}
		}
	}
}
/*Percorre o vetor */
void CLane::id_filter_Angles()	//filtragem dos pesos dos angulos
{
	float temp;
	il_sum_MagEnd.resize(il_sum_Mag.size()); 
	for(unsigned int x = 3; x < il_sum_Mag.size()-4; x++){
		temp = 0.0f;
		for(int y = -3; y < 4; y++){
			temp += il_sum_Mag[y+x] * pesos[y+3];			
		}
		il_sum_MagEnd[x] = temp;		
	}
}

void CLane::id_get_Peaks()
{
//	int x = 0;
	Peak *vectorPeak;//armazena temporiamente os picos 
	vectorPeak = new Peak[180];
	float dezMajorMag = 0.0f;
	float majorMag =  il_sum_MagEnd[0];
	for(unsigned int x = 0; x < il_sum_MagEnd.size(); x++)
	{
		vectorPeak[x].setAngle(-90+(x*SUBINTERVAL));
		vectorPeak[x].setMagnitude(il_sum_MagEnd[x]);
		if(majorMag < il_sum_MagEnd[x]){
			majorMag = il_sum_MagEnd[x];										//pega o maior peso
		}
	}	
	dezMajorMag = majorMag/10.0f;						        //calcula 10% do maior peso		
	for (unsigned x = 1; x < il_sum_MagEnd.size()-1; x++)
	{
		if ((il_sum_MagEnd[x] > il_sum_MagEnd[x-1]) && (il_sum_MagEnd[x] > il_sum_MagEnd[x+1])) //testa se eh pico
			if(il_sum_MagEnd[x] > dezMajorMag)								//testa se eh maior q 10% do maior
				peak_Mag.push_back(vectorPeak[x]);					
	}

//	for (x = 0; x < peak_Mag.size(); x++)
//	{
//		int temp = peak_Mag[x].getAngle();
//	}
	delete[] vectorPeak;


}
void CLane::id_get_Pairs(){
	pos_Peak.resize(90);
	neg_Peak.resize(90);
	int m = 0, n = 0;
	for(unsigned int l = 0; l < peak_Mag.size(); l++) //separa os angulos em pares e impares
	{
		if (peak_Mag[l].getAngle() > 0){			//se eh maior que zero armazena em pos_Peak vector
			pos_Peak[m].setAngle(peak_Mag[l].getAngle());
			pos_Peak[m++].setMagnitude (peak_Mag[l].getMagnitude());
		}
		else if (peak_Mag[l].getAngle() < 0){		//se eh menor que zero armazena em neg_Peak vector
			neg_Peak[n].setAngle(peak_Mag[l].getAngle());
			neg_Peak[n++].setMagnitude(peak_Mag[l].getMagnitude());
		}
	}
	pos_Peak.resize(m); //set o novo tamanho dos vetores pos_Peak e neg_Peak
	neg_Peak.resize(n);
	peak_Mag.clear(); //limpa o vetor maior

	if(pos_Peak.size()>1)
	{
		for(unsigned int m = 0; m < neg_Peak.size(); m++) //percorre o vetor negativo para comparar com o positivo
		{
			for(unsigned int n = 0; n < pos_Peak.size()-1; n++)
			{
//				int negMA = neg_Peak[m].getAngle();
//				int posNA = pos_Peak[n].getAngle();
//				int posN1A = pos_Peak[n+1].getAngle();
				// se a diferença do ang neg e pos eh menor que a dif do neg e pos em uma posiçao a frente
				// armazena esses angulos senão armazena os seguintes
				if(abs(neg_Peak[m].getAngle() + pos_Peak[n].getAngle()) < 2) 
				{
					peak_Mag.push_back(neg_Peak[m]);
					peak_Mag.push_back(pos_Peak[n]);
				}
				else
				{
					peak_Mag.push_back(neg_Peak[m]);
					peak_Mag.push_back(pos_Peak[n+1]);
				}
			}
		}
	}
	else
	{
		for(unsigned int m = 0; m < neg_Peak.size(); m++) //percorre o vetor negativo para comparar com o positivo
		{
			peak_Mag.push_back(neg_Peak[m]);
			peak_Mag.push_back(pos_Peak[0]);
		}
	}
	m = 0;
	while(peak_Mag.size() > 2)
	{
		if ((peak_Mag[0].getAngle() + peak_Mag[1].getAngle()) == (peak_Mag[2].getAngle() + peak_Mag[3].getAngle()))
		{
			peak_Mag.erase (peak_Mag.begin()+2);
			peak_Mag.erase (peak_Mag.begin()+2);
		}
		else{
			if (abs(peak_Mag[0].getAngle() + peak_Mag[1].getAngle()) > abs(peak_Mag[2].getAngle() + peak_Mag[3].getAngle()))
			{
				peak_Mag.erase (peak_Mag.begin());
				peak_Mag.erase (peak_Mag.begin());
			}
			else
			{
				peak_Mag.erase (peak_Mag.begin()+2);
				peak_Mag.erase (peak_Mag.begin()+2);
			}
		}
	}
	pos_Peak.clear();
	neg_Peak.clear();
}

void CLane::id_applying_HT()
{

	float maxHistoEsq, maxHistoDir;	//armazenaram o valores máximos dos histogramas da esquerda e da direita
	alphaDireita = -peak_Mag[1].getAngle();//armazena o angulo da direita
	alphaEsquerda = -peak_Mag[0].getAngle();//armazena o angulo da esquerda
//	int x = 0;
//	int y = 0;
	float maxEsq = 0.0f, maxDir = 0.0f, minEsq = 0.0f, minDir = 0.0f;
	float maxWeightEsq = 0.0f, maxWeightDir = 0.0f;
	//percorre a matriz dos angulos para pegar o maior peso;
	for (int y = 0; y < altTotal; y++){
		for (int x = 0; x < largTotal; x++){
			//verifica as orientações que correspondem aos angulos encontrados
			if ((abs(matrix_Orientation[x][y] + alphaDireita) < DELTA_ALPHA)&&(x > (largTotal/2)))
			{
				//armazena o maior peso dos angulos correspondente da direita
				if (maxWeightDir < matrix_Weight[x][y])
					maxWeightDir  = matrix_Weight[x][y];
	
			}
			if ((abs(matrix_Orientation[x][y] + alphaEsquerda) < DELTA_ALPHA)&&(x < (largTotal/2)))
			{
				//armazena o maior peso dos angulos correspondente da esquerda
				if (maxWeightEsq < matrix_Weight[x][y])
					maxWeightEsq  = matrix_Weight[x][y];				
			}			
		}		
	}
	//calcula os dez por cento dos maiores pesos
	float dezPCWeightDir = maxWeightDir/10;
	float dezPCWeightEsq = maxWeightEsq/10;
	
	//vetores q armazenaram os rhos
	rhoDir.clear();
	rhoEsq.clear();
	//percorre a matriz dos angulos correspondentes e calcula os pesos e os rhos
	for (int y = 1; y < altTotal; y++){
		for (int x = 1; x<largTotal; x++){
			//se eh o angulo correto da direita
			if ((abs(matrix_Orientation[x][y] + alphaDireita) < DELTA_ALPHA)&&(x > (largTotal/2)))
			{
				//se eh maior que os dez por cento do maior peso encontrado entre os angulos
				if (matrix_Weight[x][y] > dezPCWeightDir){
					//armazena em um vetor o peso correspondente do angulo
					matrixADirWeight.push_back(matrix_Weight[x][y]);
					//calcula o rho e armazena em um vetor
					rhoDir.push_back ((float)y*cos((float)alphaDireita/180.0f*PI)+x*sin((float)alphaDireita/180.0f*PI));
				}				
			}
			//se eh o angulo correto da esquerda
			if ((abs(matrix_Orientation[x][y] + alphaEsquerda) < DELTA_ALPHA)&&(x < (largTotal/2)))
			{
				//se eh maior que os dez por cento do maior peso encontrado entre os angulos
				if (matrix_Weight[x][y] > dezPCWeightEsq){
					//armazena em um vetor o peso correspondente do angulo
					matrixAEsqWeight.push_back(matrix_Weight[x][y]);
					//calcula o rho e armazena em um vetor
					rhoEsq.push_back((float) y*cos((float)alphaEsquerda/180.0f*PI)+x*sin((float)alphaEsquerda/180.0f*PI));
				}
			}			
		}		
	}
	/*O código a seguir realiza uma filtragem nos valores obtidos nos Rhos*/
	float temp = 0.0f;
	rhoEsqEnd.resize(rhoEsq.size());
	for(unsigned int x = 3; x < rhoEsq.size()-4; x++){
		temp = 0.0f;
		for(int y = -3; y < 4; y++){
			temp += rhoEsq[y+x] * pesos[y+3];			
		}
		rhoEsqEnd[x] = temp;		
	}
	temp = 0.0f;
	rhoDirEnd.resize(rhoDir.size()); 
	for(unsigned int x = 3; x < rhoDir.size()-4; x++){
		temp = 0.0f;
		for(int y = -3; y < 4; y++){
			temp += rhoDir[y+x] * pesos[y+3];
		}
		rhoDirEnd[x] = temp;		
	}

	// variáveis para armazenar o menor e o maior rho da esquerda e da direita
	minEsq = rhoEsqEnd[0]; maxEsq = rhoEsqEnd[0]; minDir = rhoDirEnd[0]; maxDir = rhoDirEnd[0];
	//verifica qual o menor e maior rho da esquerda
	for (unsigned int x=0;x<rhoEsqEnd.size();x++){
		if(rhoEsqEnd[x] < minEsq){
			minEsq = rhoEsqEnd[x]; 
		}
		if(rhoEsqEnd[x] > maxEsq){
			maxEsq = rhoEsqEnd[x]; 
		}
	}
	int indice = 0;
	float var_Esq = float(maxEsq - minEsq);

	histoEsq.clear(); 
	histoEsq.resize(int(var_Esq)+1);
	for (unsigned int x=0;x<histoEsq.size();x++){
		histoEsq[x] = 0.0f; 
	}
	for (unsigned int x=0;x<rhoEsqEnd.size();x++){
		indice = int(rhoEsqEnd[x] - minEsq);
		histoEsq[indice] += matrixAEsqWeight[x];
	}
	maxHistoEsq = histoEsq[0];
	indiceMaxEsq = 0;
	for (unsigned int x=0;x<histoEsq.size();x++){
		if(maxHistoEsq < histoEsq[x]){
			maxHistoEsq = histoEsq[x];
			indiceMaxEsq = x;
		}
	}
	indiceMaxEsq+= minEsq;

	for (unsigned int x=0;x<rhoDirEnd.size();x++){
		if(rhoDirEnd[x] < minDir){
			minDir = rhoDirEnd[x]; 
		}
		if(rhoDirEnd[x] > maxDir){
			maxDir = rhoDirEnd[x]; 
		}
	}

	indiceMaxEsq += minEsq;
	float var_Dir = float(maxDir - minDir);
	indice = 0;
	histoDir.clear();
	histoDir.resize(int(var_Dir)+1);
	for (unsigned int x=0;x<histoDir.size();x++){
		histoDir[x] = 0.0f; 
	}
	for (unsigned int x=0;x<rhoDirEnd.size();x++){
		indice = int(rhoDirEnd[x] - minDir);
		histoDir[indice] += matrixADirWeight[x];
	}
	maxHistoDir = histoDir[0];
	indiceMaxDir = 0;
	for (unsigned int x=0;x<histoDir.size();x++){
		if(maxHistoDir < histoDir[x]){
			maxHistoDir = histoDir[x];
			indiceMaxDir = x;
		}
	}
	indiceMaxDir += minDir;
}
void CLane::id_rate_ABD(float matrix_ABDEsq[3],float matrix_ABDDir[3])
{
	matrix_ABDEsq[0] = float(indiceMaxEsq)/sin((float(alphaEsquerda)/180.f*PI));
	matrix_ABDEsq[1] = -1/tan(alphaEsquerda/180.0f*PI);
	matrix_ABDEsq[2] = matrix_ABDEsq[1];
	matrix_ABDDir[0] = float(indiceMaxDir)/sin((float(alphaDireita)/180.0f*PI));
	matrix_ABDDir[1] = -1/tan(alphaDireita/180.0f*PI);
	matrix_ABDDir[2] = matrix_ABDDir[1];
}



/*###########################MÉTODOS do ESTADO DE TREINAMENTO##################################
	
	Empregadas durante o estado de treinamento do sistema. Este treinamento visa ao sistema
	determinar o melhor valor para XV, variavel que determina o ponto de encontro da bordas
	das pistas

#############################MÉTODOS do ESTADO DE TREINAMENTO##############################*/

/*
Metodo que vai acumulando o valor de iXv durante os quadros estipulados pelo usuário
para depois realizar divisão e obter uma media.
*/

void CLane::fd_acummXV(float matrixABDl[3],float matrixABDr[3])
{
	iXv = (matrixABDl[0] - matrixABDr[0]) / (matrixABDr[1] - matrixABDl[1]);
	acummXV += (matrixABDl[0] - matrixABDr[0]) / (matrixABDr[1] -matrixABDl[1]);
}

/*
Função para dividir o valor acumulado durante o treinamento do XV pela quantidade de Frames lidos.
É realizado para determinar a média de valores encontrados durante a fase de treinamento.
*/

void CLane::fd_divXV()
{
	iXv = (int) acummXV/trainingFrames;
}

/*############## MÉTODOS de DETECÇÃO das BORDAS no PROSSEGUIMENTO da PISTA ######################
	
	Empregadas no processo de detecção no prosseguimento das bordas da pista do sistema.
	Todas elas iniciam com fd_ : following detection.
	Abaixo estão os métodos genéricos, utilizados nos dois modelos, tanto no modelo dependente
	como no modelo independente.

################################################################################################*/

/*********************************************************************
   Funcao para remover estruturas indesejadas devido a interferências,
   ruídos ou outras estruturas que aparecem na estrada.

   Recebe a media dos pesos e o tamanho da matriz aux e Retorna void.
*********************************************************************/
void CLane::fd_thresholderEdge(float mediaEsq, float mediaDir, int *tamEsq, int *tamDir)
{
	/*armazenarão temporariamente os valores maiores q a média encontrada pelo metodo
	fd_meanMagnitude, tanto do lado esquerdo da pista como direito*/
	matrixAuxTempEsq = new MatrixAux[*tamEsq];
	matrixAuxTempDir = new MatrixAux[*tamDir];
	int i=0,i2, j, lbroi, jEsq , jDir,  indiceEsq=0, indiceDir=0, contEsq=0, contDir=0; //indMain=0;
	float fSobel =0.0f;
	jEsq = matrixMainEsq[i].posJ ; jDir = matrixMainDir[i].posJ;
	for (i2 = LINESKY-1; i < tamMatrixMain; i++,i2++)//percorre a matriz principal e 
		//verifica se os valores saum iguais ou maiores q a media
		//se são coloca os valores e os indices da posição da matriz em uma matriz auxiliar
		//através da chamada da função fillMatrixAux.
	{
		lbroi = (compMax - compMin)/((float)altMax - (float)altMin) * (float)(i2) + compMin;//calcula a distancia do LBROI		
 		for(j = 0; j <= lbroi; ++j) 
		{
			if((jEsq-j-1 > 1 )&& (jEsq-j-1 < largMax-largMin - 1)){//verifica se naum sai da matriz
				fSobel = matrixWeight[jEsq-j-1][i2];
				if( fSobel >= mediaEsq){//verifica se eh ruido se não é ruido 
					//chama a função para armazenar seus valores na matriz aux.
					fillMatrixAux(matrixAuxTempEsq, indiceEsq,(float)i2,(float)(jEsq-j-1), fSobel);		
					contEsq++;
			}}
			if((jDir+j < largMax-largMin - 1) && (jDir+j > 1)){
				fSobel = matrixWeight[jDir+j][i2];
				if(fSobel >= mediaDir){
					fillMatrixAux(matrixAuxTempDir, indiceDir,(float)i2,(float)(jDir+j), fSobel);
					contDir++;					
			}}
			if((jEsq+j > 1)&&(jEsq+j < largMax-largMin - 1)){
			fSobel = matrixWeight[jEsq+j][i2];
				if(fSobel >= mediaEsq){
					fillMatrixAux(matrixAuxTempEsq, indiceEsq, (float)i2,(float)(jEsq+j),fSobel);
					contEsq++;
			}}
			if((jDir-j-1<largMax - largMin-1) && (jDir-j-1 > 1)){
			fSobel = matrixWeight[jDir-j-1][i2];
				if(fSobel >= mediaDir){
					fillMatrixAux(matrixAuxTempDir, indiceDir,(float)i2,(float)(jDir-j-1), fSobel);
					contDir++;		
			}}
		}
		jEsq = matrixMainEsq[i].posJ ; jDir = matrixMainDir[i].posJ;
	}
	matrixAEsq.resize(contEsq);//redimensiona o vetor da esquerda para o novo tamanho 
	matrixADir.resize(contDir);//redimensiona o vetor da direita para o novo tamanho 
	matrixAEsq.clear();//limpa o vetor
	matrixADir.clear();
	for (i = 0; i < contEsq; i++){// percorre o vetor final com o novo tamanho
		matrixAEsq.push_back(matrixAuxTempEsq[i]);
	}
	for (i = 0; i < contDir; i++){// percorre o vetor final com o novo tamanho
		matrixADir.push_back(matrixAuxTempDir[i]);
	}
	delete[] matrixAuxTempEsq;
	delete[] matrixAuxTempDir;
	*tamEsq = contEsq;
	*tamDir = contDir;
}

/*************************************************************************************************
   Funcao para retornar o valor da média dos pesos de cada lado da estrada
   Possui dois for para percorrer a matriz principal calculando através da função "fd_sobel", os
   pesos para a matriz "matrixWeight" e ao mesmo tempo fazendo o somatório e um contador para o 
   cálculo da média.

   Recebe um ponteiro para int para armazenar o tamanho do vetor auxiliar
   Retorna um float com o valor da media
*************************************************************************************************/

void CLane::fd_meanMagnitude(int *tamEsq, int *tamDir, float *mediaEsq, float *mediaDir){
	int i,i2, j,  jEsq , jDir, contEsq=0, contDir=0,lbroi;
	float mediaE = 0.0f, mediaD = 0.0f;
	jEsq = matrixMainEsq[0].posJ ; jDir = matrixMainDir[0].posJ;
	for (i = 0, i2 = LINESKY-1; i < tamMatrixMain; i++,i2++)
	{
	lbroi = (int)((compMax - compMin)/((float)altMax - (float)altMin) * (float)i2 + compMin);//calcula a distancia do LBROI
		for(j = 0; j <= lbroi; ++j) 
		{
			if((jEsq-j-1 > 1 )&& (jEsq-j-1 < largMax-largMin - 1)){
				matrixWeight[jEsq-j-1][i2] = fd_sobel(jEsq-j-1,i2);
				mediaE += matrixWeight[jEsq-j-1][i2]; contEsq++;
			}
			if((jDir+j < largMax-largMin - 1) && (jDir+j > 1)){
				matrixWeight[jDir+j][i2] = fd_sobel(jDir+j,i2);
				mediaD += matrixWeight[jDir+j][i2]; contDir++;
			}
			if((jEsq+j > 1)&&(jEsq+j < largMax-largMin - 1)){
				matrixWeight[jEsq+j][i2] = fd_sobel(jEsq+j,i2);
				mediaE += matrixWeight[jEsq+j][i2]; contEsq++;
			}
			if((jDir-j-1<largMax - largMin-1) && (jDir-j-1 > 1)){
				matrixWeight[jDir-j-1][i2] = fd_sobel(jDir-j-1,i2);
				mediaD += matrixWeight[jDir-j-1][i2]; contDir++;
			}
		}
		jEsq = matrixMainEsq[i].posJ ; jDir = matrixMainDir[i].posJ;
	}
	*tamEsq = contEsq;//atualiza o tamanho com o cont Esq
	*tamDir = contDir;//atualiza o tamanho com o cont Dir
	*mediaEsq = (mediaE/contEsq);
	*mediaDir = (mediaD/contDir);
}

float CLane::fd_sobel(int x, int y)
{
	int tmpX = 0, tmpY = 0;
	float tmp;
	tmpX=-matrix_Main[x-1][y-1]-2*matrix_Main[x][y-1]-matrix_Main[x+1][y-1]+
				matrix_Main[x-1][y+1]+2*matrix_Main[x][y+1]+matrix_Main[x+1][y+1];
    tmpY=-matrix_Main[x-1][y-1]+matrix_Main[x+1][y-1]-2*matrix_Main[x-1][y]+
				2*matrix_Main[x+1][y]-matrix_Main[x-1][y+1]+matrix_Main[x+1][y+1];
	return tmp = abs(tmpX) + abs(tmpY);		
}

/*função que calcula o valor de XM conforme a estrada, se for uma reta o XM diminui de valor se
for uma curva o xm aumenta de valor*/
void CLane::fd_solveXM()
{
	double temp;
	if(matrixABC[2] < 0)
		temp = matrixABC[2]*-1;
	else
		temp = matrixABC[2];
	if (temp > 0.014){
		iXm = min(XM_MAX,(float) iXm+1);
	}
	else if(temp < 0.009){
		iXm = max(XM_MIN,(float) iXm-1);
	}
}


/*###### MÉTODOS da DETECÇÃO das BORDAS no PROSSEGUIMENTO da PISTA (MODELO INDEPENDENTE)##########
	
	Empregadas no processo de detecção no prosseguimento das bordas da pista do sistema usando
	modo independente. Todas elas iniciam com fd_ : following detection.

################################################################################################*/


/*Metodo que realiza o processo de detecção de bordas no prosseguimento da pista utilizando o 
processo independente para cada borda de pista. Este processo é utilizado somente nos primeiros 
quadros determinado pelo usuário para verificar o valor a ser preenchido na variável iXv
O Método recebe uma matriz de float com os valores da imagem, realiza a detecção e retorna uma nova
matriz da imagem.
*/

IplImage* CLane::fd_followingIndependent()
{
	fd_cleanVariaveisIndependent();
	this->fd_meanMagnitude(&tamMatrixAuxEsq, &tamMatrixAuxDir, &mediaEsq, &mediaDir);
	this->fd_thresholderEdge(mediaEsq, mediaDir, &tamMatrixAuxEsq, &tamMatrixAuxDir);
	this->fd_fillMatrixIndependent();
	this->fd_solveLinear(matrixB1Esq,matrixL1Esq, matrixABDEsq);
	this->fd_solveLinear(matrixB1Dir,matrixL1Dir, matrixABDDir);
	this->fd_solveEquationIndep(matrixABDEsq,matrixMainEsq);
	this->fd_solveEquationIndep(matrixABDDir,matrixMainDir);
	this->fd_acummXV(matrixABDEsq,matrixABDDir);
	this->gd_paintImage();
	return this->gd_matrixToImg(&matrix_Main);	
}

/*
Método para zerar as variáveis utilizadas no processo de detecção de bordas independente, 
ou seja, utilizando duas matrizes, uma para cada borda.
*/

void CLane::fd_cleanVariaveisIndependent()
{
	for(int x = 0; x < 3; x++){
		for(int y = 0; y < 3; y++){
			matrixB1Esq[x][y] = 0.0f;
			matrixB2Esq[x][y] = 0.0f;
			matrixB1Dir[x][y] = 0.0f;
			matrixB2Dir[x][y] = 0.0f;
		}
		matrixL1Esq[x] = 0.0f;
		matrixL1Dir[x] = 0.0f;
		matrixL2Esq[x] = 0.0f;
		matrixL2Dir[x] = 0.0f;
		matrixABDEsq[x] = 0.0f;
		matrixABDDir[x] = 0.0f;
	}
	matrixAEsq.clear();
	matrixADir.clear();
}


/*********************************************************************
	Função para resolver o modelo dos limites da estrada, determinando os
valores para preencher os vetores auxiliares da esquerda e da direita 
*********************************************************************/

void CLane::fd_solveEquationIndep(float matrixABD[3], MatrixMain matrixAux[])
{
	for(int x = 0; x < tamMatrixMain; x++)
	{
		matrixAux[x].posI= 0.0f;
		matrixAux[x].posJ= 0.0f; 
	}
	float a=matrixABD[0], b=matrixABD[1], tempf; //d=matrixABD[2]
	for(int i = 0; i < tamMatrixMain; i++)
	{		
		matrixAux[i].posI = (float)(i+LINESKY);
		tempf = a + b * matrixAux[i].posI;
		matrixAux[i].posJ = (float)tempf;
	}
}
 /*********************************************************************
   Funcao para resolver o sistema Linear para a matriz 3x3;

   Recebe a MatrizB, o vetor L e um vetor para armazenar o valor de
   ABD; 
   Retorna void.
*********************************************************************/
void CLane::fd_solveLinear(float matrixB[3][3], float matrixL[3], float matrixABD[3])
{
	float a = matrixB[0][0],b=matrixB[0][1],c=matrixB[1][1];
	float c1 = matrixL[0], c2=matrixL[1];
	float x=0, y= 0, z=0, base = 0;	
	base = (b*b)-a*c;
	x = -c*c1+b*c2;
	y = b*c1-a*c2;
	z = y;
	matrixABD[0] = x/base;
	matrixABD[1] = y/base;
	matrixABD[2] = z/base;
}

/*********************************************************************
   Funcao para preencher as matrizes B1 e B2 percorrendo a matriz auxiliar

   Recebe void 
   Retorna void
*********************************************************************/
void CLane::fd_fillMatrixIndependent(){
//	int i = 0;
	for (unsigned int i = 0; i < matrixAEsq.size(); i++)
	{
		updateMatrixB1(apMatrixB1Esq, matrixB1Esq,matrixAEsq[i].weight, matrixAEsq[i].posI);
		updateMatrixL1(apMatrixL1Esq, matrixL1Esq,matrixAEsq[i].weight, matrixAEsq[i].posJ, matrixAEsq[i].posI);
	}
	for (unsigned int i = 0; i < matrixADir.size(); i++)
	{
		updateMatrixB1(apMatrixB1Dir, matrixB1Dir,matrixADir[i].weight, matrixADir[i].posI);
		updateMatrixL1(apMatrixL1Dir, matrixL1Dir,matrixADir[i].weight, matrixADir[i].posJ, matrixADir[i].posI);
	}
	matrixAEsq.clear();//limpa os vetores
	matrixADir.clear(); 

}

/*####### MÉTODOS da DETECÇÃO das BORDAS no PROSSEGUIMENTO DA PISTA (MODELO DEPENDENTE)##########
	
	Empregadas no processo de detecção no prosseguimento das bordas da pista do sistema usando
	modo dependente. Todas elas iniciam com fd_ : following detection.

################################################################################################*/

/*Metodo que realiza o processo de detecção de bordas no prosseguimento da pista utilizando o 
processo dependente através de uma matriz 4x4 para o calculo da detecção das bordas de pista.
Este processo é utilizado durante toda a detecção após a parte inicial de treinamento para  
quadros determinado pelo usuário para verificar o valor a ser preenchido na variável iXv
Ele recebe um objeto CVisGrayByteImage imagem, realiza a detecção e retorna um novo
CVisGrayByteImage da imagem.
*/

IplImage* CLane::fd_followingDependent()
{	
	fd_cleanVariaveisDependent();
	this->fd_meanMagnitude(&tamMatrixAuxEsq, &tamMatrixAuxDir, &mediaEsq, &mediaDir);
	this->fd_thresholderEdge(mediaEsq, mediaDir, &tamMatrixAuxEsq, &tamMatrixAuxDir);
	this->fd_fillMatrixDependent();
	this->fd_solveLinear(matrixABC);
	this->fd_solveEquation(matrixABC,matrixMainEsq, matrixMainDir);
	this->fd_solveXM();
	this->fd_solveOffSetLateral();
//	this->fd_obstacles_Detection();
	this->gd_paintImage();
	return this->gd_matrixToImg(&matrix_Main);
}
/*
   Funcao para resolver o sistema Linear
   Recebe a Matriz para armazenar os valores resultantes
   Retorna void.
***********************************************************************************************/

void CLane::fd_solveLinear(double matrixArBrCrBl[4])
{
	double a = matrixTotal[0][0],b=matrixTotal[0][1],c=matrixTotal[0][2];
	double d = matrixTotal[0][3],e=matrixTotal[1][1],f=matrixTotal[1][2];
	double g = matrixTotal[1][3],h=matrixTotal[2][2],i=matrixTotal[2][3];
	double j = matrixTotal[3][3];
 
	double m0 = vetorTotal[0], m1 = vetorTotal[1], m2 = vetorTotal[2], m3 = vetorTotal[3];

	double x = 0, y = 0, z = 0, w = 0;	

	x = (f*f*j*m0 + d*f*i*m1 - b*i*i*m1 - c*f*j*m1 + b*h*j*m1 - b*f*j*m2 + g*g*(h*m0 - c*m2) - d*f*f*m3 + b*f*i*m3 + e*(i*i*m0 - h*j*m0 + 
      c*j*m2 + d*h*m3 - i*(d*m2 + c*m3)) + g*((-d)*h*m1 + c*i*m1 + b*i*m2 - b*h*m3 + f*(-2*i*m0 + d*m2 + c*m3)))/
   (a*g*g*h + d*d*(-f*f + e*h) - 2*a*f*g*i - b*b*i*i + a*e*i*i + 2*d*(c*f*g - b*g*h - c*e*i + b*f*i) +  a*f*f*j + b*b*h*j - a*e*h*j + c*c*(-g*g + e*j) + 
    2*b*c*(g*i - f*j));

	y = ((-b)*i*i*m0 + b*h*j*m0 +    a*i*i*m1 - a*h*j*m1 - a*g*i*m2 + a*f*j*m2 + d*d*(h*m1 - f*m2) + a*g*h*m3 - a*f*i*m3 + 
    c*c*(j*m1 - g*m3) + d*((-g)*h*m0 + f*i*m0 - 2*c*i*m1 + c*g*m2 + b*i*m2 + c*f*m3 - b*h*m3) + c*(g*i*m0 - f*j*m0 - b*j*m2 + b*i*m3))/
   (a*g*g*h + d*d*(-f*f + e*h) - 2*a*f*g*i - b*b*i*i + a*e*i*i + 2*d*(c*f*g - b*g*h - c*e*i + b*f*i) + a*f*f*j + b*b*h*j - a*e*h*j + c*c*(-g*g + e*j) + 
    2*b*c*(g*i - f*j));

	z = (b*g*i*m0 - b*f*j*m0 - a*g*i*m1 + a*f*j*m1 + a*g*g*m2 + b*b*j*m2 -  a*e*j*m2 + d*d*((-f)*m1 + e*m2) - a*f*g*m3 - 
    b*b*i*m3 + a*e*i*m3 + d*(f*g*m0 - e*i*m0 + c*g*m1 + b*i*m1 - 2*b*g*m2 - c*e*m3 + b*f*m3) + c*((-g*g)*m0 + e*j*m0 - b*j*m1 + b*g*m3))/
   (a*g*g*h + d*d*(-f*f + e*h) - 2*a*f*g*i - b*b*i*i + a*e*i*i + 2*d*(c*f*g - b*g*h - c*e*i + b*f*i) + a*f*f*j + b*b*h*j - a*e*h*j + c*c*(-g*g + e*j) + 
    2*b*c*(g*i - f*j));

	w = ((-b)*g*h*m0 + b*f*i*m0 + a*g*h*m1 - a*f*i*m1 - a*f*g*m2 - b*b*i*m2 + a*e*i*m2 + d*((-f*f)*m0 + e*h*m0 + c*f*m1 - 
      b*h*m1 - c*e*m2 + b*f*m2) + a*f*f*m3 + b*b*h*m3 - a*e*h*m3 + c*c*((-g)*m1 + e*m3) + c*(f*g*m0 - e*i*m0 + b*i*m1 + b*g*m2 - 2*b*f*m3))/
   (a*g*g*h + d*d*(-f*f + e*h) - 2*a*f*g*i - b*b*i*i + a*e*i*i + 2*d*(c*f*g - b*g*h - c*e*i + b*f*i) + a*f*f*j + b*b*h*j - a*e*h*j + c*c*(-g*g + e*j) + 
    2*b*c*(g*i - f*j));
	  
	matrixArBrCrBl[0] = x;
	matrixArBrCrBl[1] = y;
	matrixArBrCrBl[2] = z;
	matrixArBrCrBl[3] = w;
}

 /*********************************************************************************************
	
	  Função para resolver o modelo dos limites da estrada, determinando os valores para 
	preencher os vetores auxiliares da esquerda e da direita.
	Recebe uma matriz 4x4 e as duas matrizes auxiliares onde serão armazenados os valores 
	calculados

***********************************************************************************************/

void CLane::fd_solveEquation(double matrixArBrCrBl[4], MatrixMain matrixAuxEsq[], MatrixMain matrixAuxDir[])
{
	//percorre as duas matrizes onde serão armazenados os valores calculados e zera as posições
	for(int x = 0; x < tamMatrixMain; x++)
	{
		matrixAuxEsq[x].posI= 0.0f;
		matrixAuxEsq[x].posJ= 0.0f;
		matrixAuxDir[x].posI= 0.0f;
		matrixAuxDir[x].posJ= 0.0f; 
	}
	//armazena os valores da matriz 4x4 em variáveis que serão utilizadas para o calculo 
	float aR = matrixArBrCrBl[0], bR = matrixArBrCrBl[1], cR = matrixArBrCrBl[2], bL = matrixArBrCrBl[3];
	float cL = cR;
	float aL = (iXv - iXm) * (bR-bL) + aR;
	for(int i = 0; i < tamMatrixMain; i++)
	{
		matrixAuxEsq[i].posI = (float)(i+LINESKY);
		matrixAuxDir[i].posI = (float)(i+LINESKY);		
		if(matrixAuxEsq[i].posI > iXm)
		{
			matrixAuxEsq[i].posJ = aL + (bL * (matrixAuxEsq[i].posI-iXm));
			matrixAuxDir[i].posJ = aR + (bR * (matrixAuxDir[i].posI-iXm));
		}
		else
		{
			matrixAuxEsq[i].posJ = aL + (bL * (matrixAuxEsq[i].posI-iXm)) + (cL *(matrixAuxEsq[i].posI-iXm)*(matrixAuxEsq[i].posI-iXm));
			matrixAuxDir[i].posJ = aR + (bR * (matrixAuxDir[i].posI-iXm)) + (cR *(matrixAuxDir[i].posI-iXm)*(matrixAuxDir[i].posI-iXm));
		}
	}
}

/*
	Método para zerar as variáveis utilizadas no processo de detecção de bordas dependente, 
	ou seja, utilizando somente uma matriz.
*/
void CLane::fd_cleanVariaveisDependent()
{
	for(int x = 0; x < 4; x++){
		for(int y = 0; y < 4; y++){
			matrixTotal[x][y] = 0.0f;
			matrixFarLeft[x][y] = 0.0f;
			matrixNearLeft[x][y] = 0.0f;
			matrixFarRight[x][y] = 0.0f;
			matrixNearRight[x][y] = 0.0f;
		}
		vetorTotal[x] = 0.0f;
		vetorNLeft[x] = 0.0f;
		vetorFLeft[x] = 0.0f;
		vetorFRight[x] = 0.0f;
		vetorNRight[x] = 0.0f;
		matrixABDEsq[x] = 0.0f;
		matrixABDDir[x] = 0.0f;
		matrixABC[x] = 0.0f;
	}
	matrixAEsq.clear();
	matrixADir.clear();
}

/*
	Funcao para preencher a Matriz 4x4 e o Vetor[4].
	Recebe void 
	Retorna void
*/
void CLane::fd_fillMatrixDependent(){

/*
	Percorre toda a matriz que contem as posições onde estam os pesos da imagem
	do lado esquerda da pista, para cada posição, acumula os pesos na matriz 4x4 e 
	no Vetor[4];
*/
//	int i = 0;
	for (unsigned int i = 0; i < matrixAEsq.size(); i++)
	{
		if (matrixAEsq[i].posI <= iXm)
		{
			updateMatrixFarLeft(matrixAEsq[i].weight, matrixAEsq[i].posI);
			updateVetorFarLeft(matrixAEsq[i].weight, matrixAEsq[i].posJ, matrixAEsq[i].posI);
		}
		else
		{
			updateMatrixNearLeft(matrixAEsq[i].weight, matrixAEsq[i].posI);
			updateVetorNearLeft(matrixAEsq[i].weight, matrixAEsq[i].posJ, matrixAEsq[i].posI);
		}
	}
	for (unsigned int i = 0; i < matrixADir.size(); i++)
	{
		if (matrixADir[i].posI <= iXm)
		{
			updateMatrixFarRight(matrixADir[i].weight, matrixADir[i].posI);
			updateVetorFarRight(matrixADir[i].weight, matrixADir[i].posJ, matrixADir[i].posI);
		}
		else
		{			
			updateMatrixNearRight(matrixADir[i].weight, matrixADir[i].posI);
			updateVetorNearRight(matrixADir[i].weight, matrixADir[i].posJ, matrixADir[i].posI);
		}
	}
	matrixAEsq.clear();
	matrixADir.clear();
	fd_multMatrices();	//realiza a multiplicação com as matrizes
	fd_sumMatrices();	//realiza a soma das quatro matrizes
	fd_sumVectors();	//realiza a soma dos quatro vetores
}
// multiplica valores a algumas posições das matrizes
void  CLane::fd_multMatrices()
{
	matrixNearLeft[0][1] *= (iXv-iXm);
	matrixNearLeft[1][0] *= (iXv-iXm);
	matrixNearLeft[1][1] *= ((iXv-iXm)*(iXv-iXm));
	matrixNearLeft[1][3] *= (iXv-iXm);
	matrixNearLeft[3][1] *= (iXv-iXm);
	matrixFarLeft[0][1] *= (iXv-iXm);
	matrixFarLeft[1][0] *= (iXv-iXm);
	matrixFarLeft[1][1] *= ((iXv-iXm)*(iXv-iXm));
	matrixFarLeft[1][2] *= (iXv-iXm);
	matrixFarLeft[1][3] *= (iXv-iXm);
	matrixFarLeft[2][1] *= (iXv-iXm);
	matrixFarLeft[3][1] *= (iXv-iXm);
	vetorNLeft[1] *= (iXv-iXm);
	vetorFLeft[1] *= (iXv-iXm);
}

//soma as quatro matrizes
void  CLane::fd_sumMatrices()
{
	for(int x = 0; x < 4; x++){
		for(int y = 0; y < 4; y++){
			matrixTotal[x][y] =  matrixNearRight[x][y] + matrixNearLeft[x][y] + matrixFarRight[x][y] + matrixFarLeft[x][y];
		}
	}
}
//soma os quatro vetores
void CLane::fd_sumVectors()
{
	for(int x = 0; x < 4; x++){
		vetorTotal[x] =  vetorNLeft[x] + vetorNRight[x] + vetorFLeft[x] + vetorFRight[x];
	}
}

/*############################ MÉTODOS para a DETECÇÃO da SAÍDA da PISTA########################
	
	Empregadas no processo de detecção da saída da pista do sistema. 

################################################################################################*/
/*
	Método que calcula e armazena em um vetor o valor do lateral offset
*/
void CLane::fd_solveOffSetLateral()
{
	float A = (matrixABC[3] + matrixABC[1])/2;
	float DA = (matrixABC[3] - matrixABC[1]);
	lateralOffset = A/DA;
	vLateralOffSet.push_back(lateralOffset);
	if(vLateralOffSet.size()>4){
		fd_filter_LateralOffset();
	}
}
/*
	Método que realiza a filtragem dos pesos encontrados e armazenados no vetor do lateral offset.
	Os valores da filtragem são armazenados em um vetor.
*/
void CLane::fd_filter_LateralOffset()
{
	float temp = 0.0f;
	for(int x = 0; x < 5; x++){
			temp += vLateralOffSet[vLateralOffSet.size() - 1 - x] * discreteGaussianWeights[x];			
	}
	vFilteredLateralOffSet.push_back(temp);
}
/*
	Método que realiza a detecção da saida de pista verificando se o veiculo não está no centro da pista.
	Ele realiza dois testes simultaneos. Um que verifica se a distancia é maior que 1/4 da pista
	confirmando a troca de pista e o segundo que verifica se a variação do lateralOffSet é constante, ou seja,
	o veiculo realmente estah trocando de pista.

*/
int CLane::fd_lane_Departure_Detection()	//detecção da saida de pista
{
//	float temp = vLateralOffSet[vLateralOffSet.size()-1] ;
	if(((vLateralOffSet[vLateralOffSet.size()-1]) > 0.25f && (fd_distance_Increasing()))||(vLateralOffSet[vLateralOffSet.size()-1]>0.47)){
		return 1;
	}
	else if (((vLateralOffSet[vLateralOffSet.size()-1])< -0.25f && (fd_distance_Decreasing()))||(vLateralOffSet[vLateralOffSet.size()-1]>0.47)){
		return -1;
	}
	else
		return 0;
}
/*
	Este método verifica se estah havendo variação no aumento da distancia do centro da pista
	Se o valor estiver aumentando indica que o veiculo estah se deslocando do centro para a direita
	da pista.
*/
bool CLane::fd_distance_Increasing()	//aumento
{
	int cont = 0;
	for(int x = 0; x < 4; x++){
		if ((vFilteredLateralOffSet[vFilteredLateralOffSet.size()-x-1] - vFilteredLateralOffSet[vFilteredLateralOffSet.size()-x-2]) > 0)
		{cont++;}		
	}
	if (cont>3)
		return true;
	else
		return false;
}
/*
	Este método verifica se estah havendo variação na diminuição da distancia do centro da pista
	Se o valor estiver diminuindo indica que o veiculo estah se deslocando do centro para a direita
	da pista.
*/
bool CLane::fd_distance_Decreasing()	//filtragem dos pesos dos angulos
{
	int cont = 0;
	for(int x = 0; x < 4; x++){
		if ((vFilteredLateralOffSet[vFilteredLateralOffSet.size()-x-1] - vFilteredLateralOffSet[vFilteredLateralOffSet.size()-x-2]) < 0)
		{cont++;}
	}
	if (cont>3)
		return true;
	else
		return false;
}
/*
	Metodo para calcular o desvio padrão de um conjunto de valores do vetor do devios laterais filtrados
	A variação do desvio padrão indica se o veiculo trocou de pista.
*/
float CLane::fd_standard_Deviation()
{
	int x = 0;
	int amostra = 30; //quantidade de valores que serão utilizados
	float media = 0.0f; //armazenará a media dos valores
	for(x = 0; x < amostra; x++){//calcula a média
		media += vFilteredLateralOffSet[vFilteredLateralOffSet.size()-1-x];
	}
	media /= amostra;
	float standard_Deviation = 0.0f;
	//calcula o desvio padrão
	for(x = 0; x < amostra; x++){
		standard_Deviation += (vFilteredLateralOffSet[vFilteredLateralOffSet.size()-1-x] - media) * (vFilteredLateralOffSet[vFilteredLateralOffSet.size()-1-x] - media);
	}
	standard_Deviation = sqrt(standard_Deviation/amostra);
	return standard_Deviation;
}


float CLane::fd_obstacles_Detection()
{
	int tamBlocks = int(tamMatrixMain/BLOCKS_OD);//Define o tamanho dos blocos que se calculará a mediana
	int indexX = 0;

	for(int k = 0; k < BLOCKS_OD; k++){
		contBlocks[k] = 0;
		vMedian[k].clear();
	}
	
	float mediana1 = 0.0f;// armazena a mediana
	if (iControllerOD < TRAINNING_OD){
		for(int k = 0; k < BLOCKS_OD; k++){//percorre todos os blocos que a pista foi dividida
			for(int x = 0; x < tamBlocks; x ++){ //percorre o "interior" dos blocos para calcular a mediana
				indexX = x + (k*tamBlocks); //define qual o indice que do eixo x do interior de cada bloco
				for(int y = matrixMainEsq[indexX].posJ; y < matrixMainDir[indexX].posJ; y++)//percorre por dentro das faixas
				{
					if(y > 0 && y < largTotal)//verifica se estah dentro da imagem
					{
						vMedian[k].push_back(matrix_Main[y][matrixMainDir[indexX].posI]);//armazena o valor do pixel no vetor
						contBlocks[k]++;//controla a quantidade de pixels para o posterior calculo da mediana;
					}
				}
			}
		}
		iControllerOD++;
	}
	if(iControllerOD == TRAINNING_OD){
		for(int k = 0; k < BLOCKS_OD; k++){
			fd_quicksort(vMedian[k], 0, int( vMedian[k].size() - 1 ) );// coloca o vetor em ordem crescente para determinar a mediana

			int indiceMedio = contBlocks[k]/2;// armazena o indice central
			if(contBlocks[k]%2 == 0){// se o vetor é par soma-se as duas posições centrais e calcula a media.
				aMedians[k] = (vMedian[k][indiceMedio] + vMedian[k][indiceMedio+1])/2;
			}
			else{			// se o vetor é impar armazena-se diretamente o valor da posição central
				aMedians[k] = vMedian[k][indiceMedio];
			}
			float standard_Deviation = 0.0f;
			//calcula o desvio padrão			
			for(int x = 0; x < tamBlocks; x ++){
				indexX = x + (k*tamBlocks); //define qual o indice que do eixo x do interior de cada bloco
				for(int y = matrixMainEsq[indexX].posJ; y < matrixMainDir[indexX].posJ; y++){
					if(y > 0 && y < largTotal){
						standard_Deviation += (matrix_Main[y][matrixMainDir[indexX].posI] - aMedians[k]) * (matrix_Main[y][matrixMainDir[indexX].posI] - aMedians[k]);
					}
				}
			}
			aStandartDev[k] = sqrt(standard_Deviation/(contBlocks[k] -1));
			iControllerOD++;
		}
		
	}
	if (iControllerOD >TRAINNING_OD){
		matrixObstacles.clear();
		for(int k = 0; k < BLOCKS_OD; k++){
			fMi = aMedians[k];
			fTi = 2 * aStandartDev[k];
			for(int x = 0; x < tamBlocks; x ++){
				indexX = x + (k*tamBlocks);
				for(int y = matrixMainEsq[indexX].posJ - (matrixMainDir[indexX].posJ-matrixMainEsq[indexX].posJ) ; y < matrixMainDir[indexX].posJ ; y++){
					if(y > 0 && y < largTotal){
//						float teste = abs(matrix_Main[y][int(matrixMainDir[indexX].posI)] - fMi);
//						float teste2 = fMi - aMedians[k];
						if ((abs(matrix_Main[y][int(matrixMainDir[indexX].posI)] - fMi)) > (fTi)){
							MatrixMain tempMxMain;
							tempMxMain.posJ = y;
							tempMxMain.posI = int(matrixMainDir[indexX].posI);
							matrixObstacles.push_back(tempMxMain);			
						}
					}
				}				
			}
		}
	}
	return mediana1;
}


/* ######## QUICK SORT ######### */
void CLane::fd_quicksort(vector <float> & arr, int start, int end){
int pivot, starth, endh; // store pivot # keep start & end in memory for split
starth = start;
endh = end;
pivot = arr[start];

while(start < end)
{
 while((arr[end] >= pivot) && (start < end))
  end--;
    if (start != end)
    {
      arr[start] = arr[end];
      start++;
    }
  while ((arr[start] <= pivot) && (start < end))
      start++;
    if (start != end)
    {
      arr[end] = arr[start];
      end--;
    }
}
arr[start] = pivot;
pivot = start;
start = starth;
end = endh;
if(start < pivot)
	fd_quicksort(arr, start, pivot-1);
if(end > pivot)
	fd_quicksort(arr, pivot+1, end);
}



/* 
	Método que compara os angulos das bordas detecdas durante o prosseguimento na pista.
	A soma desses angulos indica se o veiculo estah trocando de pista ou não.
*/
int CLane::fd_get_Angles()
{
	int sum_Angles = 0;
//	int temp1 = atan(matrixABC[1])*180/PI;
//	int temp2 = atan(matrixABC[3])*180/PI;
	sum_Angles = atan(matrixABC[1])*180/PI + atan(matrixABC[3])*180/PI;
	return sum_Angles;
}


/*############################### MÉTODOS para a TROCA de PISTA###############################
	
	Métodos empregados no processo de troca de pista. 

################################################################################################*/

void CLane::fd_get_New_Boundary(){
//	DWORD dwStart = GetTickCount();
	id_cleanVariaveis();
	fd_gradient_Function_New();
	fd_sum_Magnitudes_New();
	id_filter_Angles();
	id_get_Peaks();
	fd_get_Pairs_New();
	fd_applying_HT_New();
	
	if (iBoundary == 1){
		for(int x = 0; x < tamMatrixMain; x++){
			matrixMainEsq[x].posI = matrixMainDir[x].posI;
			matrixMainEsq[x].posJ = matrixMainDir[x].posJ;
		}		
		this->fd_rate_ABDDir_New(matrixABDDir);
		this->gd_solveEquationLBM(matrixABDDir,matrixMainDir);
	}
	else{
		for(int x = 0; x < tamMatrixMain; x++){
			matrixMainDir[x].posI = matrixMainEsq[x].posI;
			matrixMainDir[x].posJ = matrixMainEsq[x].posJ;
		}
		this->fd_rate_ABDEsq_New(matrixABDEsq);
		this->gd_solveEquationLBM(matrixABDEsq,matrixMainEsq);
	}
}
void CLane::fd_gradient_Function_New(){
	iMiddlef = matrixABC[0] + (matrixABC[1] * (iXv-iXm));
	if (iBoundary == 1){
		for (int i = 1; i < altTotal-1; i++){
 			for(int j = iMiddlef; j < largMax-largMin-1; j++){
				matrix_Weight[j][i] = fd_rate_Sobel_New(j,i);
			}
		}	
		fd_rate_Orientation_New();
	}
	else{
		for (int i = 1; i < altTotal-1; i++){
 			for(int j = 1; j < iMiddlef; j++){
				matrix_Weight[j][i] = fd_rate_Sobel_New(j,i);
			}
		}	
		fd_rate_Orientation_New();
	}
}
void CLane::fd_rate_Orientation_New(){
	double dX = 0.0f, dY = 0.0f;
	if (iBoundary == 1)	{//Saida para DIREITA
		for (int y = 1; y < altTotal-1; y++){
 			for(int x = iMiddlef; x < largMax-largMin-1; x++){
				dX = 0.0f;dY = 0.0f;
				dX=(double)((2*matrix_TmpX[x-1][y-1]*matrix_TmpY[x-1][y-1])+(2*matrix_TmpX[x][y-1]*matrix_TmpY[x][y-1])+(2*matrix_TmpX[x+1][y-1]*matrix_TmpY[x+1][y-1])+(2*matrix_TmpX[x-1][y]*matrix_TmpY[x-1][y])+(2*matrix_TmpX[x][y]*matrix_TmpY[x][y])+(2*matrix_TmpX[x+1][y]*matrix_TmpY[x+1][y])+
					(2*matrix_TmpX[x-1][y+1]*matrix_TmpY[x-1][y+1])+(2*matrix_TmpX[x][y+1]*matrix_TmpY[x][y+1])+(2*matrix_TmpX[x+1][y+1]*matrix_TmpY[x+1][y+1]));
				dY=(double)((matrix_TmpY[x-1][y-1]*matrix_TmpY[x-1][y-1] - matrix_TmpX[x-1][y-1]*matrix_TmpX[x-1][y-1])+(matrix_TmpY[x+1][y-1]*matrix_TmpY[x+1][y-1]-matrix_TmpX[x+1][y-1]*matrix_TmpX[x+1][y-1])+(matrix_TmpY[x-1][y]*matrix_TmpY[x-1][y]-matrix_TmpX[x-1][y]*matrix_TmpX[x-1][y])+
					(matrix_TmpY[x][y-1]*matrix_TmpY[x][y+1]-matrix_TmpX[x][y-1]*matrix_TmpX[x][y-1])+(matrix_TmpY[x][y]*matrix_TmpY[x][y]-matrix_TmpX[x][y]*matrix_TmpX[x][y])+(matrix_TmpY[x][y+1]*matrix_TmpY[x][y+1]-matrix_TmpX[x][y+1]*matrix_TmpX[x][y+1])+
					(matrix_TmpY[x+1][y]*matrix_TmpY[x+1][y]-matrix_TmpX[x+1][y]*matrix_TmpX[x+1][y])+(matrix_TmpY[x-1][y+1]*matrix_TmpY[x-1][y+1]-matrix_TmpX[x-1][y+1]*matrix_TmpX[x-1][y+1])+(matrix_TmpY[x+1][y+1]*matrix_TmpY[x+1][y+1]-matrix_TmpX[x+1][y+1]*matrix_TmpX[x+1][y+1]));
				matrix_Orientation[x][y] = ((atan(dX/dY+0.000001f))/2 * 180/PI) ;
			}
		}
	}
	else{//Saida para a ESQUERDA
		for (int y = 1; y < altTotal-1; y++){
 			for(int x = 1; x < iMiddlef; x++){
				dX = 0.0f;dY = 0.0f;
				dX=(double)((2*matrix_TmpX[x-1][y-1]*matrix_TmpY[x-1][y-1])+(2*matrix_TmpX[x][y-1]*matrix_TmpY[x][y-1])+(2*matrix_TmpX[x+1][y-1]*matrix_TmpY[x+1][y-1])+(2*matrix_TmpX[x-1][y]*matrix_TmpY[x-1][y])+(2*matrix_TmpX[x][y]*matrix_TmpY[x][y])+(2*matrix_TmpX[x+1][y]*matrix_TmpY[x+1][y])+
					(2*matrix_TmpX[x-1][y+1]*matrix_TmpY[x-1][y+1])+(2*matrix_TmpX[x][y+1]*matrix_TmpY[x][y+1])+(2*matrix_TmpX[x+1][y+1]*matrix_TmpY[x+1][y+1]));
				dY=(double)((matrix_TmpY[x-1][y-1]*matrix_TmpY[x-1][y-1] - matrix_TmpX[x-1][y-1]*matrix_TmpX[x-1][y-1])+(matrix_TmpY[x+1][y-1]*matrix_TmpY[x+1][y-1]-matrix_TmpX[x+1][y-1]*matrix_TmpX[x+1][y-1])+(matrix_TmpY[x-1][y]*matrix_TmpY[x-1][y]-matrix_TmpX[x-1][y]*matrix_TmpX[x-1][y])+
					(matrix_TmpY[x][y-1]*matrix_TmpY[x][y+1]-matrix_TmpX[x][y-1]*matrix_TmpX[x][y-1])+(matrix_TmpY[x][y]*matrix_TmpY[x][y]-matrix_TmpX[x][y]*matrix_TmpX[x][y])+(matrix_TmpY[x][y+1]*matrix_TmpY[x][y+1]-matrix_TmpX[x][y+1]*matrix_TmpX[x][y+1])+
					(matrix_TmpY[x+1][y]*matrix_TmpY[x+1][y]-matrix_TmpX[x+1][y]*matrix_TmpX[x+1][y])+(matrix_TmpY[x-1][y+1]*matrix_TmpY[x-1][y+1]-matrix_TmpX[x-1][y+1]*matrix_TmpX[x-1][y+1])+(matrix_TmpY[x+1][y+1]*matrix_TmpY[x+1][y+1]-matrix_TmpX[x+1][y+1]*matrix_TmpX[x+1][y+1]));
				matrix_Orientation[x][y] = ((atan(dX/dY+0.000001f))/2 * 180/PI) ;
			}
		}
	}
}
float CLane::fd_rate_Sobel_New(int x, int y){
	float tmpX = 0.0f, tmpY = 0.0f;
	float tmp;
	tmpX=(float)(-matrix_Main[x-1][y-1]-2*matrix_Main[x][y-1]-matrix_Main[x+1][y-1]+
				matrix_Main[x-1][y+1]+2*matrix_Main[x][y+1]+matrix_Main[x+1][y+1]);
	tmpY=(float)(-matrix_Main[x-1][y-1]+matrix_Main[x+1][y-1]-2*matrix_Main[x-1][y]+
				2*matrix_Main[x+1][y]-matrix_Main[x-1][y+1]+matrix_Main[x+1][y+1]);
	matrix_TmpX[x][y] = tmpX;
	matrix_TmpY[x][y] = tmpY;
	return tmp =float(abs(tmpX) + abs(tmpY));
}

void CLane::fd_sum_Magnitudes_New(){
	int tamanhoVetor = 180/SUBINTERVAL, indice = 0; //armazena o tamanho q serah o vetor das Magnitudes
	il_sum_Mag.resize(tamanhoVetor);				//redimensiona o vetor	
	for(unsigned int x = 0; x < il_sum_Mag.size(); x++){		//
		il_sum_Mag[x] = 0.0f;
	}

	if (iBoundary == 1){
		for (int i = 1; i < altTotal-1; i++){
 			for(int j = iMiddlef; j < largMax-largMin-1; j++){
				if((matrix_Orientation[j][i] >=-90) && (matrix_Orientation[j][i] <= 90)){
					indice = 0;
					indice = int((matrix_Orientation[j][i]+90)/SUBINTERVAL);
					float temp = 0.0f;
					temp = il_sum_Mag[indice] + matrix_Weight[j][i];
					il_sum_Mag[indice] = temp;
				}
			}
		}
	}
	else{
		for (int i = 1; i < altTotal-1; i++){
 			for(int j = 1; j < iMiddlef; j++){
				if((matrix_Orientation[j][i] >=-90) && (matrix_Orientation[j][i] <= 90)){
					indice = 0;
					indice = int((matrix_Orientation[j][i]+90)/SUBINTERVAL);
					float temp = 0.0f;
					temp = il_sum_Mag[indice] + matrix_Weight[j][i];
					il_sum_Mag[indice] = temp;
				}
			}
		}
	}
}
void CLane::fd_get_Pairs_New(){
	pos_Peak.resize(90);
	neg_Peak.resize(90);
	int m = 0, n = 0;
	if (iBoundary == 1)	
	{//Saida para DIREITA
		for(unsigned int l = 0; l < peak_Mag.size(); l++) //separa os angulos em positivos e negativos
		{
//			int temp2 = peak_Mag[l].getAngle();
			if (peak_Mag[l].getAngle() < 0){		//se eh menor que zero armazena em neg_Peak vector
				neg_Peak[n].setAngle(peak_Mag[l].getAngle());
				neg_Peak[n++].setMagnitude(peak_Mag[l].getMagnitude());
			}
		}		
		pos_Peak[0].setAngle(atan(matrixABC[1])*180/PI);
//		int temp = pos_Peak[0].getAngle();
		neg_Peak.resize(n);
		
		peak_Mag.clear(); //limpa o vetor maior
		for(unsigned m = 0; m < neg_Peak.size(); m++) //percorre o vetor negativo para comparar com o positivo
		{
//			int negMA = neg_Peak[m].getAngle();
			// se a diferença do ang neg e pos eh menor que a dif do neg e pos em uma posiçao a frente
			// armazena esses angulos senão armazena os seguintes
			if(abs(neg_Peak[m].getAngle() + pos_Peak[0].getAngle()) < 2){
				peak_Mag.push_back(neg_Peak[m]);
				peak_Mag.push_back(pos_Peak[0]);
			}
		}
	}
	else
	{//Saida para ESQUERDA
		for(unsigned int l = 0; l < peak_Mag.size(); l++) //separa os angulos em positivos e negativos
		{
//			int temp2 = peak_Mag[l].getAngle();
			if (peak_Mag[l].getAngle() > 0){		//se eh menor que zero armazena em neg_Peak vector
				pos_Peak[n].setAngle(peak_Mag[l].getAngle());
				pos_Peak[n++].setMagnitude(peak_Mag[l].getMagnitude());
			}
		}
		neg_Peak[0].setAngle(atan(matrixABC[3])*180/PI);
//		int temp = neg_Peak[0].getAngle();
		pos_Peak.resize(n);
		peak_Mag.clear(); //limpa o vetor maior
		for(unsigned m = 0; m < pos_Peak.size(); m++) //percorre o vetor negativo para comparar com o positivo
		{
//			int posMA = pos_Peak[m].getAngle();
			// se a diferença do ang neg e pos eh menor que a dif do neg e pos em uma posiçao a frente
			// armazena esses angulos senão armazena os seguintes
			if(abs(pos_Peak[m].getAngle() + neg_Peak[0].getAngle()) < 2) 
			{
				peak_Mag.push_back(neg_Peak[0]);
				peak_Mag.push_back(pos_Peak[m]);
			}
		}
	}
	m = 0;
	while(peak_Mag.size() > 2)
	{
		if ((peak_Mag[0].getAngle() + peak_Mag[1].getAngle()) == (peak_Mag[2].getAngle() + peak_Mag[3].getAngle())){
			peak_Mag.erase (peak_Mag.begin()+2);
			peak_Mag.erase (peak_Mag.begin()+2);
		}
		else{
			if (abs(peak_Mag[0].getAngle() + peak_Mag[1].getAngle()) > abs(peak_Mag[2].getAngle() + peak_Mag[3].getAngle())){
				peak_Mag.erase (peak_Mag.begin());
				peak_Mag.erase (peak_Mag.begin());
			}
			else{
				peak_Mag.erase (peak_Mag.begin()+2);
				peak_Mag.erase (peak_Mag.begin()+2);
			}
		}
	}
	pos_Peak.clear();
	neg_Peak.clear();
}

void CLane::fd_applying_HT_New(){
//	int y = 0;
//	int x = 0;
	float maxHistoEsq, maxHistoDir;	//armazenaram o valores máximos dos histogramas da esquerda e da direita
	alphaDireita = -peak_Mag[1].getAngle();//armazena o angulo da direita
	alphaEsquerda = -peak_Mag[0].getAngle();//armazena o angulo da esquerda
	float maxEsq = 0.0f, maxDir = 0.0f, minEsq = 0.0f, minDir = 0.0f;
	float maxWeightEsq = 0.0f, maxWeightDir = 0.0f;

	if (iBoundary == 1){
		for (int y = 0; y < altTotal; y++){
			for (int x = iMiddlef; x < largTotal; x++){
				if (abs(matrix_Orientation[x][y] + alphaDireita) < DELTA_ALPHA){
					if (maxWeightDir < matrix_Weight[x][y])
						maxWeightDir  = matrix_Weight[x][y];	
				}
			}		
		}
		float dezPCWeightDir = maxWeightDir/10;
		rhoDir.clear();
		for (int y = 1; y < altTotal; y++){
			for (int x = iMiddlef; x < largTotal; x++){
				if (abs(matrix_Orientation[x][y] + alphaDireita) < DELTA_ALPHA){
					if (matrix_Weight[x][y] > dezPCWeightDir){
						matrixADirWeight.push_back(matrix_Weight[x][y]);
						rhoDir.push_back ((float)y*cos((float)alphaDireita/180.0f*PI)+x*sin((float)alphaDireita/180.0f*PI));
					}				
				}
			}		
		}
		float temp = 0.0f;
		rhoDirEnd.resize(rhoDir.size()); 
		for(unsigned int x = 3; x < rhoDir.size()-4; x++){
			temp = 0.0f;
			for(int y = -3; y < 4; y++){
				temp += rhoDir[y+x] * pesos[y+3];			
			}
			rhoDirEnd[x] = temp;
		}

		// variáveis para armazenar o menor e o maior rho da esquerda e da direita
		minDir = rhoDirEnd[0]; maxDir = rhoDirEnd[0];
		//verifica qual o menor e maior rho da esquerda
		int indice = 0;
		for (unsigned int x=0; x<rhoDirEnd.size();x++){
			if(rhoDirEnd[x] < minDir){
				minDir = rhoDirEnd[x]; 
			}
			if(rhoDirEnd[x] > maxDir){
				maxDir = rhoDirEnd[x]; 
			}		
		}
		float var_Dir = float(maxDir - minDir);
		histoDir.clear();
		histoDir.resize(int(var_Dir)+1);
		for (unsigned int x=0;x<histoDir.size();x++){
			histoDir[x] = 0.0f; 
		}
		for (unsigned int x=0;x<rhoDirEnd.size();x++){
			indice = int(rhoDirEnd[x] - minDir);
			histoDir[indice] += matrixADirWeight[x];
		}
		maxHistoDir = histoDir[0];
		indiceMaxDir = 0;
		for (unsigned int x=0;x<histoDir.size();x++){
			if(maxHistoDir < histoDir[x]){
				maxHistoDir = histoDir[x];
				indiceMaxDir = x;
			}
		}
		indiceMaxDir += minDir;
	}
	else 	
	{
		for (int y = 0; y < altTotal; y++){
			for (int x = 0; x < iMiddlef; x++){
				if (abs(matrix_Orientation[x][y] + alphaEsquerda) < DELTA_ALPHA){
					if (maxWeightEsq < matrix_Weight[x][y])
						maxWeightEsq  = matrix_Weight[x][y];				
				}			
			}		
		}
		float dezPCWeightEsq = maxWeightEsq/10;
		rhoEsq.clear();
		for (int y = 1; y < altTotal; y++){
			for (int x = 1; x < iMiddlef; x++){
				if (abs(matrix_Orientation[x][y] + alphaEsquerda) < DELTA_ALPHA){
					if (matrix_Weight[x][y] > dezPCWeightEsq){
						matrixAEsqWeight.push_back(matrix_Weight[x][y]);
						rhoEsq.push_back((float) y*cos((float)alphaEsquerda/180.0f*PI)+x*sin((float)alphaEsquerda/180.0f*PI));
					}
				}			
			}		
		}
		float temp = 0.0f;
		rhoEsqEnd.resize(rhoEsq.size());
		for(unsigned int x = 3; x < rhoEsq.size()-4; x++){
			temp = 0.0f;
			for(int y = -3; y < 4; y++){
				temp += rhoEsq[y+x] * pesos[y+3];			
			}
			rhoEsqEnd[x] = temp;		
		}

		// variáveis para armazenar o menor e o maior rho da esquerda e da direita
		minEsq = rhoEsqEnd[0]; maxEsq = rhoEsqEnd[0];
		//verifica qual o menor e maior rho da esquerda
		for (unsigned int x=0;x<rhoEsqEnd.size();x++){
			if(rhoEsqEnd[x] < minEsq){
				minEsq = rhoEsqEnd[x]; 
			}
			if(rhoEsqEnd[x] > maxEsq){
				maxEsq = rhoEsqEnd[x]; 
			}		
		}
		int indice = 0;
		float var_Esq = float(maxEsq - minEsq);

		histoEsq.clear(); 
		histoEsq.resize(int(var_Esq)+1);
		for (unsigned int x=0;x<histoEsq.size();x++){
			histoEsq[x] = 0.0f; 
		}
		for (unsigned int x=0;x<rhoEsqEnd.size();x++){
			indice = int(rhoEsqEnd[x] - minEsq);
			histoEsq[indice] += matrixAEsqWeight[x];
		}
		maxHistoEsq = histoEsq[0];
		indiceMaxEsq =0;
		for (unsigned int x=0;x<histoEsq.size();x++){
			if(maxHistoEsq < histoEsq[x]){
				maxHistoEsq = histoEsq[x];
				indiceMaxEsq = x;
			}
		}
		indiceMaxEsq+= minEsq;
		for (unsigned int x=0;x<rhoDirEnd.size();x++){
			if(rhoDirEnd[x] < minDir){
				minDir = rhoDirEnd[x]; 
			}
			if(rhoDirEnd[x] > maxDir){
				maxDir = rhoDirEnd[x]; 
			}		
		}
		indiceMaxEsq += minEsq;
	}
}

void CLane::fd_rate_ABDEsq_New(float matrix_ABDEsq[3]){
	matrix_ABDEsq[0] = float(indiceMaxEsq)/sin((float(alphaEsquerda)/180.f*PI));
	matrix_ABDEsq[1] = -1/tan(alphaEsquerda/180.0f*PI);
	matrix_ABDEsq[2] = matrix_ABDEsq[1];
}
void CLane::fd_rate_ABDDir_New(float matrix_ABDDir[3]){
	matrix_ABDDir[0] = float(indiceMaxDir)/sin((float(alphaDireita)/180.0f*PI));
	matrix_ABDDir[1] = -1/tan(alphaDireita/180.0f*PI);
	matrix_ABDDir[2] = matrix_ABDDir[1];
}





