/* ********************************************************************
		UNISINOS - Universidade do Vale do Rio dos Sinos   
 Sistema de Apoio ao Motorista baseados em Visao Computacional
 Coordenador - Cláudio Rosito Jung
 Bolsista CNPq - Leandro Lorenzett Dihl

 Projeto CNPQ - Processo 506350/2004-5			Agosto/2005

Classe utilizada como matriz din‰mica

**********************************************************************/

/*##########################  Includes  #############################*/
#if !defined (MATRIX_DEFINED)

#include <math.h>
#include <iostream>
#include <vector>
#include <stdexcept>

using namespace std;

template <class T>
class Matrix
{
public:
   Matrix():m_dimRow(0), m_dimCol(0){;}
   Matrix(int nRow, int nCol) {
      m_dimRow = nRow;
      m_dimCol = nCol;
      for (int i=0; i < nRow; i++){
         vector<T> x(nCol);
//         int y = x.size();
         m_2DVector.push_back(x);
      }
   }

   int GetCol(){
		return m_dimRow;
   }
    int GetRow(){
		return m_dimCol;
   }

   void SetAt(int nRow, int nCol, const T& value) throw(std::out_of_range) {
      if(nRow >= m_dimRow || nCol >= m_dimCol)
         throw out_of_range("Array out of bound");
      else
         m_2DVector[nRow][nCol] = value;
   };
	
   T GetAt(unsigned int nRow, unsigned int nCol) {
       if (nRow >= m_dimRow || nCol >= m_dimCol) {
		  out_of_range("Array out of bound");
           return false;
       } else
         return m_2DVector[nRow][nCol];
   }
	
   void GrowRow(int newSize) {
      if (newSize <= m_dimRow)
         return;
      m_dimRow = newSize;
	  for(int i = 0 ; i < newSize - m_dimCol; i++)   {
		 vector<float> x(m_dimRow);
		 m_2DVector.push_back(x);


	  }
   }
	
	void GrowCol(int newSize) {
      if(newSize <= m_dimCol)
         return;
      m_dimCol = newSize;
      for (int i=0; i <m_dimRow; i++)
         m_2DVector[i].resize(newSize);
	}
   
	vector<T>& operator[](int x) {
      return m_2DVector[x];
	}
	
private:
   vector< vector <T> > m_2DVector;
   unsigned int m_dimRow;
   unsigned int m_dimCol;
};

#define MATRIX_DEFINED true

#endif
