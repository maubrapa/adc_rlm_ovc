/* ********************************************************************
		UNISINOS - Universidade do Vale do Rio dos Sinos   
 Sistema de Apoio ao Motorista baseados em Visao Computacional
 Coordenador - Cláudio Rosito Jung
 Bolsista CNPq - Leandro Lorenzett Dihl
 Bolsista - Vinicius

 Projeto CNPQ - Processo 506350/2004-5			Maio/2005
**********************************************************************/

#pragma once

/*##########################  Includes  #############################*/

#include <math.h>
#include <iostream>
#include <vector>
#include <iostream>
#include "opencv2/opencv.hpp"
#include <cstdio>
#include "CMatrix.h"
#include "CPeak.h"

using namespace std;
using namespace cv;

#ifndef PI
	const float PI = 3.141592653589793f;
#endif

const int	SUBINTERVAL = 2; //define o subintervalo dos angulos
const int	DELTA_ALPHA = 2;
const int	LINESKY = 10; //define o tamanho do LINESKY
//const float XM_MAX = 75.0f; // 75.0f (low res. videos) | 130 for high res.
const float XM_MAX = 130.0f; // 75.0f (low res. videos) | 130 for high res.
const float XM_MIN = 40.0f;
const int	TRAINNING_OD = 10;//define a quantidade de frames de treinamento para a detecção de obstaculos;

/* Estados que pode se encontrar o sistema */
const int	INITIAL_STATE = 1;
const int	TRAINING_STATE = 2;
const int	ON_ROAD_STATE = 3;
const int	OFF_ROAD_STATE = 4;
const int	SWITCH_ROAD_STATE = 5;
const int	ERROR_STATE = 6;

const int	BLOCKS_OD = 3;	//define a quantidade de blocos que serah dividida a estrada;

class CLane
{

    //private:
public:
    /*##########################  Def. Tipos  ###########################*/
    
    /*Estruturas utilizadas na fase de following_detection. */
        
    
    struct MatrixAux		//MatrixAux armazena:
    {
        float posI;			//a posicaoo I(linha do pixel)
        float posJ;			//a posicao J(coluna do pixel)
        float weight;		//e weight (o valor do pixel)*/
    };
    
    struct MatrixMain		//MatrixMain armazena:
    {
        float posI;			//a posicao I(linha do pixel)
        float posJ;			//a posicao J(coluna do pixel)
    };
    

    /*#######################################################################################################

    VARIÁVEIS GLOBAIS DO PROGRAMA
    
    #######################################################################################################*/
    Matrix<float> matrix_Main;//Matriz principal para armazenar os pixels da imagem durante todo o programa
    IplImage* image_Main;
    
    int stateSystem;
    float matrixABDEsq[3];	//Matriz que armazena os valores calculados para ABD do lado Esquerdo da pista
    float matrixABDDir[3];	//Matriz que armazena os valores calculados para ABD do lado Direito da pista

    char arqEnt[13];	//Matriz para armazenar o nome do arquivo da imagem da pista a ser lida(arq de Entrada) 
    char arqSai[13];	//Matriz para armazenar o nome do arquivo da imagem da pista a ser escrita(arq de Saida)
            

    int altMax;		//ALTMAX - Determina a altura máxima da tela
    int altMin;		//ALTMIN - Determina a altura mínima da tela
    int largMin;	//LARGMIN - Determina a largura mínima da tela
    int largMax;	//LARGMAX - Determina a largura máxima da tela
    
    int altTotal;
    int largTotal;

    int contFrame;	// controla a quantidade de frames para o treinamento
    int trainingFrames;	// armazena a quantidade de frames para o treinamento
    int quadro;
    float lateralOffset;

    /*#######################################################################################################

    VARIÁVEIS GLOBAIS USADAS NA FASE DA DETECÇÃO INICIAL DAS BORDAS DA PISTA
    
    #######################################################################################################*/

    
    int alphaDireita, alphaEsquerda;//armazenam os valores dos angulos da esquerda e da direita
    int indiceMaxEsq, indiceMaxDir;	//armazenam os indices dos valores máximos dos histogramas
    float pesos[8];					//Vetor para armazenar os valores que serão utilizados na filtragem. Nos métodos: id_filter_Angles e id_applying_HT

    Matrix<float> matrix_Weight;	//Cópia da Matriz principal para armazenar os pesos
    Matrix<float> matrix_Orientation;
    Matrix<float> matrix_TmpX;
    Matrix<float> matrix_TmpY;

    vector<float> il_sum_Mag;
    vector<float> il_sum_MagEnd;
    vector<Peak> peak_Mag;
    vector<Peak> pos_Peak;
    vector<Peak> neg_Peak;
    vector<float> matrixAEsqWeight;//vetores para armazenar os pesos
    vector<float> matrixADirWeight;
    vector<float> rhoEsq;
    vector<float> rhoDir;
    vector<float> rhoEsqEnd;
    vector<float> rhoDirEnd;
    vector<float> histoEsq;
    vector<float> histoDir;

    /*#######################################################################################################

    VARIÁVEIS GLOBAIS USADAS NA FASE DA DETECÇÃO DAS BORDAS NO PROSSEGUIMENTO DA PISTA - ALGORITMO INDEPENDENTE
    
    #######################################################################################################*/

    float matrixB1Esq[3][3];	//Matriz Bidimensional q armazena os valores de B1 do lado Esquerdo da Pista
    float matrixB2Esq[3][3];	//Matriz Bidimensional q armazena os valores de B2 do lado Esquerdo da Pista
    float matrixL1Esq[3];		//Matriz Unidimensional q armazena os valores de L1 do lado Esquerdo da Pista
    float matrixL2Esq[3];		//Matriz Unidimensional q armazena os valores de L2 do lado Esquerdo da Pista

    float matrixB1Dir[3][3];	//Matriz Bidimensional q armazena os valores de B1 do lado Direito da Pista
    float matrixB2Dir[3][3];	//Matriz Bidimensional q armazena os valores de B2 do lado Direito da Pista
    float matrixL1Dir[3];		//Matriz Unidimensional q armazena os valores de L1 do lado Direito da Pista
    float matrixL2Dir[3];		//Matriz Unidimensional q armazena os valores de L2 do lado Direito da Pista
    
    //Ponteiros para percorrerem a as matrizes especificadas acima
    float *apMatrixB1Esq, *apMatrixB2Esq, *apMatrixL1Esq, *apMatrixL2Esq;
    float *apMatrixB1Dir, *apMatrixB2Dir, *apMatrixL1Dir, *apMatrixL2Dir;

    int tamMatrixMain, tamMatrixAuxEsq, tamMatrixAuxDir; //Armazenará o tamanho que deve ser a matrix Auxiliar;


    /*#######################################################################################################

    VARIÁVEIS GLOBAIS USADAS NA FASE DA DETECÇÃO DAS BORDAS NO PROSSEGUIMENTO DA PISTA - ALGORITMO DEPENDENTE
    
    #######################################################################################################*/

    vector<float> vLateralOffSet;
    vector<float> vFilteredLateralOffSet;
    double matrixABC[4];
    float matrixTotal[4][4];
    float vetorTotal[4];
    float discreteGaussianWeights[5];	//Discrete Gaussian Weights usados para filtrar os valores do offset

    float matrixNearLeft[4][4];		//Matriz Bidimensional q armazena os valores do near field esquerdo da Pista
    float matrixNearRight[4][4];	//Matriz Bidimensional q armazena os valores do near field direito da Pista
    float vetorNLeft[4];			//Matriz Unidimensional q armazena os valores de L1 do lado Esquerdo da Pista
    float vetorNRight[4];			//Matriz Unidimensional q armazena os valores de L2 do lado Esquerdo da Pista

    //Ponteiros para percorrerem as matrizes especificadas acima
    float *apMatrixNLeft, *apMatrixNRight, *apVetorNLeft, *apVetorNRight;

    float matrixFarLeft[4][4];	//Matriz Bidimensional q armazena os valores do far field do lado esquerdo da Pista
    float matrixFarRight[4][4];	//Matriz Bidimensional q armazena os valores do far field do lado direito da Pista
    float vetorFLeft[4];		//Matriz Unidimensional q armazena os valores de L1 do lado Direito da Pista
    float vetorFRight[4];		//Matriz Unidimensional q armazena os valores de L2 do lado Direito da Pista

    //Ponteiros para percorrerem a as matrizes especificadas acima
    float *apMatrixFLeft, *apMatrixFRight,  *apVetorFLeft, *apVetorFRight;

    vector<MatrixMain>matrixObstacles;
    int iControllerOD;//variavel para controle do metodo da deteccao de obstaculos, controla quantos quadros
    
    float aMedians[BLOCKS_OD];
    float aStandartDev[BLOCKS_OD];
    vector<float> vMedian[BLOCKS_OD];
    int contBlocks[BLOCKS_OD];// contadores
    
    //armazenará valores, em que quadro fara os calculos e em quais verificará os obstáculos.
    float fMi; //varia´veis para a comparação dos valores q podem ser possiveis obstáculos
    float fTi;

    /*VERIFICAR*/
    MatrixAux *matrixAuxTempEsq, *matrixAuxTempDir; //cria uma matriz dinamica auxiliar para armazenar os valores maiores ou iguais a media

    float compMax, compMin;	//valores q armazenam o comprimento Máximo e Mínimo do lbroi
    int iXm;//armazena o valor que divide a imagem em dois segmentos: far field e near field
    int constXM;
    int iXv;//
    float acummXV;
    int iBoundary;
    int iMiddlef;

    MatrixMain *matrixMainEsq, *matrixMainDir;	//Armazena os pontos dos dois lados da pista
    float mediaEsq, mediaDir;//Armazenará as médias
    vector<MatrixAux> matrixAEsq;//vetores para armazenar os pesos
    vector<MatrixAux> matrixADir;

    Matrix<float> matrixWeight;//Cópia da Matriz principal para armazenar os 

    /*#######################################################################################################

    DEFINIÇÃO DAS FUNÇÕES UTILIZADAS EM TODAS AS FASES DO SISTEMA
    
    #######################################################################################################*/
    
    void gd_solveEquationLBM(float matrixABD[3], MatrixMain matrixMain[]);
    void gd_paintImage();
    IplImage* gd_matrixToImg(Matrix<float>* mat);

//	Matrix<float> gd_imgToMatInt(CVisGrayByteImage* imagem);
//	CVisGrayByteImage gd_matIntToImgByte(Matrix<float> mat);

    /*#######################################################################################################

    DEFINIÇÃO DAS FUNÇÕES UTILIZADAS NA DETECÇÃO INICIAL DAS BORDAS DA PISTA
    
    #######################################################################################################*/
    
    void id_gradient_Function();
    float id_rate_Sobel(int x, int y);
    void id_fill_MatrixMain(void);
    void id_fill_MatrixMain(Matrix<float> matrix);
    void id_fill_MatrixMain(IplImage* ipl_image);
    void id_print_Matrix(void);
    void id_sum_Magnitudes(void);
    void id_print_Matrix_Weight(void);
    void id_filter_Angles(void);
    void id_get_Peaks(void);
    void id_rate_Orientation(void);
    void id_get_Pairs(void);
    void id_applying_HT(void);
    void id_rate_ABD(float matrix_ABDEsq[3],float matrix_ABDDir[3]);
    void id_cleanVariaveis(void);

    /*#######################################################################################################

    DEFINIÇÃO DAS FUNÇÕES UTILIZADAS NA DETECÇÃO DAS BORDAS NO PROSSEGUIMENTO DA PISTA
    
    #######################################################################################################*/

    void fd_thresholderEdge(float mediaEsq, float mediaDir, int *tamEsq, int *tamDir);
    void fd_meanMagnitude(int *tamEsq, int *tamDir, float *mediaEsq, float *mediaDir);
    float fd_sobel(int x, int y);
    
    inline void addWeight(float *apMatrix, float weight){*apMatrix += (float)(weight);}
    inline void addWeightY(float* apMatrix, float y, float weight){*apMatrix += (float)(y * weight);}

    inline void fillMatrixAux(MatrixAux matrixAux[], int& indice, float i, float j, float weight)
    {
        matrixAux[indice].posI = i;
        matrixAux[indice].posJ = j;
        matrixAux[indice++].weight = weight;
    }

    /*#######################################################################################################

    DEFINIÇÃO DAS FUNÇÕES UTILIZADAS NA DETECÇÃO DAS BORDAS NO PROSSEGUIMENTO DA PISTA - MODO INDEPENDENTE
    
    #######################################################################################################*/


    void fd_sumMatrix(float  matrixB1[3][3], float  matrixB2[3][3]);
    void fd_sumMatrix(float  matrixL1[3], float  matrixL2[3]);
    void fd_fillMatrixIndependent();
    void fd_solveLinear(float matrixB[3][3], float matrixL[3], float matrixABD[3]);
    void fd_cleanVariaveisIndependent();
    void fd_acummXV(float matrixABDl[3],float matrixABDr[3]);
    void fd_solveEquationIndep(float matrixABD[3], MatrixMain matrixAux[]);
    
    inline void addWeightX(float *apMatrix, float weight, float x){*apMatrix += (float)(weight*x);}
    inline void addWeightPowX(float *apMatrix, float weight, float x){*apMatrix += (float)(weight*(x*x));}
    inline void addWeightAlphaX(float *apMatrix, float weight, float x){*apMatrix += (float)(weight * (((x*x)+(constXM*constXM))/constXM/2));}
    inline void addWeightPowAlphaX(float *apMatrix, float weight, float x){*apMatrix += (float)(weight* ((((x*x)+(constXM*constXM))/constXM)/2)*((((x*x)+(constXM*constXM))/constXM)/2));}
    inline void addWeightBetaX(float *apMatrix, float weight, float x){*apMatrix += (float)(weight*(-(((x - constXM)*(x - constXM))/constXM)/2));}
    inline void addWeightPowBetaX(float *apMatrix, float weight, float x){*apMatrix += (float)(weight*((((x - constXM)*(x - constXM))/constXM)/2) *((((x - constXM)*(x - constXM))/constXM)/2));}
    inline void addWeightAlphaBetaX(float *apMatrix, float weight, float x){*apMatrix += (float)(weight*(((((x*x)+(constXM*constXM))/constXM)/2))*(-((((x - constXM)*(x - constXM))/constXM)/2)));}


    inline void updateMatrixB1(float *apMatrixB1, float matrixB1[3][3],float weight, float x)
    {
        apMatrixB1 = &matrixB1[0][0];
        addWeight(apMatrixB1, weight);//incrementa o primeiro elemento
        apMatrixB1++;
        addWeightX(apMatrixB1, weight, x);//incrementa o segundo
        apMatrixB1+=2;
        addWeightX(apMatrixB1, weight, x);//incrementa o quarto 
        apMatrixB1++;
        addWeightPowX(apMatrixB1, weight, x);//incrementa o quinto
    }
    inline void updateMatrixB2(float *apMatrixB2, float matrixB2[3][3], float weight, float x)
    {
        apMatrixB2 = &matrixB2[0][0];
        addWeight(apMatrixB2, weight);//incrementa o primeiro elemento
        apMatrixB2++;
        addWeightAlphaX(apMatrixB2, weight, x);//incrementa o segundo e o quarto
        matrixB2[1][0] = *apMatrixB2;
        apMatrixB2++;
        addWeightBetaX(apMatrixB2, weight, x);//incrementa o terceiro e o setimo
        matrixB2[2][0] = *apMatrixB2;
        apMatrixB2 += 2;
        addWeightPowAlphaX(apMatrixB2, weight, x);
        apMatrixB2++;
        addWeightAlphaBetaX(apMatrixB2, weight, x); //incrementa o quinto
        matrixB2[2][1] = *apMatrixB2;
        apMatrixB2+=3;
        addWeightPowBetaX(apMatrixB2, weight, x); //incrementa o quinto
    }
    inline void addWeightXY(float* apMatrix, float y, float x, float weight){*apMatrix += (float)(y * weight * x);}
    inline void addWeightAlphaY(float* apMatrix, float y, float x, float weight){*apMatrix += (float)(y * weight *((((x*x)+(constXM*constXM))/constXM)/2));}
    inline void addWeightBetaY(float* apMatrix, float y, float x, float weight){*apMatrix += (float)(y * weight * (-((((x - constXM)*(x - constXM))/constXM)/2)));}

    inline void updateMatrixL1(float *apMatrixL1, float matrixL1[3],float weight, float y, float x)
    {
        apMatrixL1 = &matrixL1[0];
        addWeightY(apMatrixL1, y, weight);//incrementa o primeiro elemento
        apMatrixL1++;
        addWeightXY(apMatrixL1, y, x, weight);//incrementa o segundo
    }
    inline void updateMatrixL2(float *apMatrixL2, float matrixL2[3],float weight,float y, float x)
    {
        apMatrixL2 = &matrixL2[0];
        addWeightY(apMatrixL2, y, weight);//incrementa o primeiro elemento
        apMatrixL2++;
        addWeightAlphaY(apMatrixL2, y, x, weight);//incrementa o segundo
        apMatrixL2++;
        addWeightBetaY(apMatrixL2, y, x, weight);//incrementa o terceiro
    }

    /*#######################################################################################################

    DEFINIÇÃO DAS FUNÇÕES UTILIZADAS NA DETECÇÃO DAS BORDAS NO PROSSEGUIMENTO DA PISTA - MODO DEPENDENTE
    
    #######################################################################################################*/

    void fd_multMatrices();
    void fd_sumMatrices();
    void fd_sumVectors();
    void fd_fillMatrixDependent();
    void fd_solveLinear(double matrixABC[4]);
    void fd_solveEquation(double matrixABC[4],MatrixMain matrixMainEsq[], MatrixMain matrixMainDir[]);
    void fd_solveXM();
    void fd_solveOffSetLateral();
    void fd_filter_LateralOffset();
    int fd_lane_Departure_Detection();
    bool fd_distance_Increasing();
    bool fd_distance_Decreasing();
    float fd_standard_Deviation();
    int fd_get_Angles();
    void fd_divXV();
    void fd_cleanVariaveisDependent();

    float fd_obstacles_Detection();
    void fd_quicksort(vector <float> & arr, int start, int end);

    void fd_get_New_Boundary();
    void fd_gradient_Function_New();
    void fd_rate_Orientation_New();
    float fd_rate_Sobel_New(int x, int y);
    void fd_sum_Magnitudes_New();
    void fd_applying_HT_New();
    void fd_rate_ABDDir_New(float matrix_ABDDir[3]);
    void fd_rate_ABDEsq_New(float matrix_ABDEsq[3]);
    void fd_get_Pairs_New();


    inline void addWeightXminusXM(float *apMatrix, float weight, float x){*apMatrix += (float)(weight*(x-iXm));}
    inline void addWeightPowXminusXM(float *apMatrix, float weight, float x){*apMatrix += (float)(weight*((x-iXm)*(x-iXm)));}
    inline void addWeight3PowXminusXM(float *apMatrix, float weight, float x){*apMatrix += (float)(weight*((x-iXm)*(x-iXm)*(x-iXm)));}
    inline void addWeight4PowXminusXM(float *apMatrix, float weight, float x){*apMatrix += (float)(weight*((x-iXm)*(x-iXm)*(x-iXm)*(x-iXm)));}
    inline void addWeightXminusXV(float *apMatrix, float weight, float x){*apMatrix += (float)(weight*(x-iXv));}
    inline void addWeightPowXminusXV(float *apMatrix, float weight, float x){*apMatrix += (float)(weight*((x-iXv)*(x-iXv)));}
    inline void addWeightXminusXVPowXminusXM(float *apMatrix, float weight, float x){*apMatrix += (float)(weight*(x-iXv)*((x-iXm)*(x-iXm)));}

    inline void updateMatrixNearRight(float weight, float x)
    {
        apMatrixNRight = &matrixNearRight[0][0];
        addWeight(apMatrixNRight, weight);//incrementa o primeiro elemento
        apMatrixNRight++;
        addWeightXminusXM(apMatrixNRight, weight, x);//incrementa o segundo
        matrixNearRight[1][0] = *apMatrixNRight;
        apMatrixNRight+=4;
        addWeightPowXminusXM(apMatrixNRight,weight, x);//incrementa o quarto 			

    }
    /*********************************************************************/
    inline void updateMatrixNearLeft(float weight, float x)
    {
        apMatrixNLeft = &matrixNearLeft[0][0];
        addWeight(apMatrixNLeft, weight);//incrementa o primeiro elemento
        matrixNearLeft[0][1] = *apMatrixNLeft;
        matrixNearLeft[1][0] = *apMatrixNLeft;
        matrixNearLeft[1][1] = *apMatrixNLeft;
        apMatrixNLeft+=3;
        addWeightXminusXV(apMatrixNLeft, weight,x);
        matrixNearLeft[3][0] = *apMatrixNLeft;
        matrixNearLeft[1][3] = *apMatrixNLeft;
        matrixNearLeft[3][1] = *apMatrixNLeft;
        apMatrixNLeft+=12;
        addWeightPowXminusXV(apMatrixNLeft, weight,x);
    }
    /**********************************************************************/
    inline void updateMatrixFarRight(float weight, float x)
    {
        apMatrixFRight = &matrixFarRight[0][0];
        addWeight(apMatrixFRight, weight);//incrementa o primeiro elemento
        apMatrixFRight++;
        addWeightXminusXM(apMatrixFRight, weight, x);
        matrixFarRight[1][0] = *apMatrixFRight;
        apMatrixFRight++;
        addWeightPowXminusXM(apMatrixFRight, weight, x);
        matrixFarRight[1][1] = *apMatrixFRight;
        matrixFarRight[2][0] = *apMatrixFRight;				
        apMatrixFRight+=4;
        addWeight3PowXminusXM(apMatrixFRight, weight, x);
        matrixFarRight[2][1] = *apMatrixFRight;
        apMatrixFRight+=4;
        addWeight4PowXminusXM(apMatrixFRight, weight, x);
    }
    /**********************************************************************/
    inline void updateMatrixFarLeft(float weight, float x)
    {
        apMatrixFLeft = &matrixFarLeft[0][0];
        addWeight(apMatrixFLeft, weight);//incrementa o primeiro elemento
        matrixFarLeft[0][1] = *apMatrixFLeft;
        matrixFarLeft[1][0] = *apMatrixFLeft;
        matrixFarLeft[1][1] = *apMatrixFLeft;
        apMatrixFLeft+=2;
        addWeightPowXminusXM(apMatrixFLeft, weight,x);
        matrixFarLeft[1][2] = *apMatrixFLeft;
        matrixFarLeft[2][1] = *apMatrixFLeft;
        matrixFarLeft[2][0] = *apMatrixFLeft;
        apMatrixFLeft++;
        addWeightXminusXV(apMatrixFLeft, weight,x);
        matrixFarLeft[1][3] = *apMatrixFLeft;
        matrixFarLeft[3][0] = *apMatrixFLeft;
        matrixFarLeft[3][1] = *apMatrixFLeft;
        apMatrixFLeft+=7;
        addWeight4PowXminusXM(apMatrixFLeft, weight, x);
        apMatrixFLeft++;
        addWeightXminusXVPowXminusXM(apMatrixFLeft, weight,x);
        matrixFarLeft[3][2] = *apMatrixFLeft;
        apMatrixFLeft+=4;
        addWeightPowXminusXV(apMatrixFLeft, weight,x);
    }

    inline void addWeightYXminusXM(float* apMatrix, float y, float x, float weight){*apMatrix += (float)(y * weight * (x - iXm));}
    inline void addWeightYXminusXV(float* apMatrix, float y, float x, float weight){*apMatrix += (float)(y * weight * (x - iXv));}
    inline void addWeightYPowXminusXM(float* apMatrix, float y, float x, float weight){*apMatrix += (float)(y * weight * (x - iXm) * (x - iXm));}
    inline void updateVetorNearRight(float weight, float y, float x)
    {
        apVetorNRight = &vetorNRight[0];
        addWeightY(apVetorNRight, y, weight);//incrementa o primeiro elemento
        apVetorNRight++;
        addWeightYXminusXM(apVetorNRight, y, x, weight);//incrementa o segundo
    }
    inline void updateVetorNearLeft(float weight,float y, float x)
    {
        apVetorNLeft = &vetorNLeft[0];
        addWeightY(apVetorNLeft,y, weight);//incrementa o primeiro elemento
        vetorNLeft[1] = *apVetorNLeft;//incrementa o segundo elemento
        apVetorNLeft+=3;//pula o terceiro
        addWeightYXminusXV(apVetorNLeft, y, x, weight);//incrementa o quarto
    }
    inline void updateVetorFarRight(float weight, float y, float x)
    {
        apVetorFRight = &vetorFRight[0];
        addWeightY(apVetorFRight, y, weight);//incrementa o primeiro elemento
        apVetorFRight++;
        addWeightYXminusXM(apVetorFRight, y, x, weight);//incrementa o segundo
        apVetorFRight++;
        addWeightYPowXminusXM(apVetorFRight, y, x, weight);//incrementa o terceiro
    }
    inline void updateVetorFarLeft(float weight,float y, float x)
    {
        apVetorFLeft = &vetorFLeft[0];
        addWeightY(apVetorFLeft, y, weight);//incrementa o primeiro elemento
        vetorFLeft[1] = *apVetorFLeft;//incrementa o segundo elemento
        apVetorFLeft+=2;
        addWeightYPowXminusXM(apVetorFLeft, y, x, weight);//incrementa o terceiro
        apVetorFLeft++;
        addWeightYXminusXV(apVetorFLeft, y, x, weight);//incrementa o quarto
    }

public:
    
    CLane();//construtor padrão
    CLane(int lgMin, int lgMax, int alMin, int alMax, int xm, int xv, int training, float cpMax, float cpMin);//construtor
    void setLargMin(int lgMin);		//metodo que configura a largura minima
    void setLargMax(int lgMax);		//metodo que configura a largura máxima
    void setAltMin(int alMin);		//metodo que configura a altura mínima
    void setAltMax(int alMax);		//metodo que configura a altura máxima
    void setCompMax(float cpMax);	
    void setCompMin(float cpMin);
    void imprimeXM(FILE* file);
    void setXM(int xm);
    float getXV();
    int getXM();
//		CString getStateAlgorithm();
    IplImage* gd_Run(IplImage* imagem);
    IplImage* id_initialLane();
    IplImage* id_initialLane(IplImage* image);
    IplImage* fd_followingIndependent();
    IplImage* fd_followingDependent();
};

