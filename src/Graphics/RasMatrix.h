#ifndef RASMATRIX_H
#define RASMATRIX_H

#include <QList>

class RasMatrix
{

public:
    RasMatrix(int nRow, int nCol);
    RasMatrix(double value);
    RasMatrix(bool identity, int size);

    RasMatrix* getColumn(int col);
    RasMatrix* getRow(int row);

    RasMatrix* getSubMatrix(int fromRow, int fromCol, int toRow, int toCol);
    void getSubMatrix(RasMatrix *mat, int fromRow, int fromCol, int toRow, int toCol);

    void setColumn(int col, RasMatrix* colMat);
    void setRow(int row, RasMatrix* rowMat);

    RasMatrix* getTranspose();
    void getTranspose(RasMatrix *mat);

    ~RasMatrix();

    int rows;
    int cols;
    bool isScalar;
    QList<double*> data;
};

#endif // RASMATRIX_H
