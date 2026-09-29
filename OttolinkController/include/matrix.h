#pragma once
#include <math.h>

template<int R, int C>

class Matrix{

    private:
    float data[R][C];

    public:

    // 1. Constructor
    Matrix(){
        for (int i = 0; i<R ; i++){
            for (int j = 0; j<C; j++){
                data[i][j] = 0;
            }
        }
    }

    template<typename... Args>
    Matrix(Args... values)
    {
        float temp[] = {static_cast<float>(values)...};

        int index = 0;

        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                data[i][j] = temp[index++];
            }
        }
    }

    // 2. Element access
    float& operator()(int row, int col)
    {
        return data[row][col];
    }

    const float& operator()(int row, int col) const
    {
        return data[row][col];
    }

    // 3. Matrix multiplication
    template<int C2>
    Matrix<R, C2> operator*(const Matrix<C, C2>& other) const{
        Matrix<R,C2> T;

        for (int i = 0; i < R ; i++){
            for (int j = 0; j < C2; j++){
                float summation = 0.0f;

                for (int k = 0; k < C ; k++){
                    summation += data[i][k] * other(k,j);
                }

                T(i, j) =  summation;
            }
        }

        return T;
    }

    // 4. Scalar multiplication
    Matrix<R, C> operator*(float scalar) const{
        Matrix<R,C> T;
        
         for (int i = 0; i < R ; i++){
            for (int j = 0; j < C; j++){
                T(i,j) = data[i][j] * scalar;
            }
        }

        return T;
    }

    // 5. Matrix addition
    Matrix<R, C> operator+(const Matrix<R, C>& other) const{
        Matrix<R,C> T;
        
        for (int i = 0; i < R ; i++){
            for (int j = 0; j < C; j++){
                T(i,j) = data[i][j] + other(i,j);
            }
        }

        return T;
    }

    // 6. Matrix subtraction
    Matrix<R, C> operator-(const Matrix<R, C>& other) const{
        Matrix<R,C> T;
        
        for (int i = 0; i < R ; i++){
            for (int j = 0; j < C; j++){
                T(i,j) = data[i][j] - other(i,j);
            }
        }

        return T;
    }

    // 7. Transpose
    Matrix<C, R> transpose() const
    {
        Matrix<C, R> T;

        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                T(j, i) = data[i][j];
            }
        }

        return T;
    }

    // 8. Identity matrix
    static Matrix<R, C> identity()
    {
        Matrix<R, C> T;

        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                if (i == j)
                    T(i,j) = 1.0f;
                else
                    T(i,j) = 0.0f;
            }
        }

        return T;
    }

    // 9. Zero matrix
    static Matrix<R, C> zeros()
    {
        return Matrix<R, C>();
    }

    // 10. Magnitude
    float magnitude() const
    {
        float sum = 0.0f;

        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                sum += data[i][j] * data[i][j];
            }
        }

        return sqrt(sum);
    }

    // 11. Division
    Matrix<R, C> operator/(float scalar) const
    {
        Matrix<R,C> T;

        for (int i = 0; i < R; i++)
        {
            for (int j = 0; j < C; j++)
            {
                T(i,j) = data[i][j] / scalar;
            }
        }

        return T;
    }
};